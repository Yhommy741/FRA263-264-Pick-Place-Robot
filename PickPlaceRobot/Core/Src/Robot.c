/*
 * Robot.c
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 */

#include "Robot.h"
#include <string.h>

/* ── Private helpers ─────────────────────────────────────────────────────── */

static void set_state(Robot_t *robot, Robot_State_t s, uint32_t timeout_ms)
{
    robot->state      = s;
    robot->state_tick = HAL_GetTick();
    robot->timeout_ms = timeout_ms;
}

static void reset_control(Robot_t *robot)
{
    PID_Reset(&robot->pid_vel);
    PID_Reset(&robot->pid_pos);
    robot->omega_target = 0.0f;
    robot->u_prev       = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Robot_Init  —  all parameters sourced from RobotConfig.h
 * ═══════════════════════════════════════════════════════════════════════════ */
void Robot_Init(Robot_t *robot)
{
    memset(robot, 0, sizeof(Robot_t));

    /* ── Cache runtime-accessible config fields ──────────────────────────── */
    robot->htim_ctrl = CTRL_HTIM_PTR;
    robot->Ts        = CTRL_PERIOD;
    robot->V_max     = MOTOR_V_MAX;
    robot->omega_max = RBT_MAX_SPEED;
    robot->N         = SPEED_RATIO;
    robot->ls_port   = LIM_SW_PORT;
    robot->ls_pin    = LIM_SW_PIN;

    /* ── DC Motor model ───────────────────────────────────────────────────── */
    robot->motor.Rm = MOTOR_RM;
    robot->motor.Lm = MOTOR_LM;
    robot->motor.Ke = MOTOR_KE;
    robot->motor.Kt = MOTOR_KT;
    robot->motor.J  = MOTOR_J;
    robot->motor.b  = MOTOR_B;

    /* ── Motor driver (MD20A / PWM) ──────────────────────────────────────── */
    MD20A_init(&robot->driver, MDRV_HTIM_PTR, MDRV_DIR_CH, MDRV_PWM_CH);

    /* ── Quadrature encoder ──────────────────────────────────────────────── */
    QEI_init(&robot->encoder,
             ENC_HTIM_PTR,
             CTRL_HTIM_PTR,
             ENC_PPR,
             ENC_X,
             ENC_OVERFLOW,
             CTRL_PERIOD);

    /* ── Kalman filter ───────────────────────────────────────────────────── */
    KalmanFilterDCMotor_Init             (&robot->kalman, &robot->motor);
    KalmanFilterDCMotor_Set_ObserverPeriod  (&robot->kalman, CTRL_PERIOD);
    KalmanFilterDCMotor_Set_ProcessNoise    (&robot->kalman, KF_VAR_TAU_D);
    KalmanFilterDCMotor_Set_MeasurementNoise(&robot->kalman, KF_VAR_THETA);
    KalmanFilterDCMotor_Start               (&robot->kalman, 0.0f, 0.0f, 0.0f, 0.0f);

    /* ── Velocity PID (fast loop — runs every Ts) ────────────────────────── */
    PID_Init(&robot->pid_vel,
             KP_VEL, KI_VEL, KD_VEL,
             CTRL_PERIOD,
             -MOTOR_V_MAX, MOTOR_V_MAX);

    /* ── Position PID (slow loop — runs every Ts × CTRL_LOOP_MULTI) ─────── */
    PID_Init(&robot->pid_pos,
             KP_POS, KI_POS, KD_POS,
             CTRL_PERIOD * CTRL_LOOP_MULTI,
             -RBT_MAX_SPEED, RBT_MAX_SPEED);

    /* ── Feedforward ─────────────────────────────────────────────────────── */
    FF_Init(&robot->ff, &robot->motor);

    /* ── Trajectory generator ────────────────────────────────────────────── */
    Trajectory_Init(&robot->traj,
                    CTRL_PERIOD * CTRL_LOOP_MULTI,
                    RBT_MAX_SPEED,
                    RBT_MAX_ACCEL);

    /* ── Gripper ─────────────────────────────────────────────────────────── */
    Gripper_Init(&robot->gripper,
                 /* Outputs */
                 GRP_UP_PORT_OUT,    GRP_UP_PIN_OUT,
                 GRP_DOWN_PORT_OUT,  GRP_DOWN_PIN_OUT,
                 GRP_OPEN_PORT_OUT,  GRP_OPEN_PIN_OUT,
                 GRP_CLOSE_PORT_OUT, GRP_CLOSE_PIN_OUT,
                 /* Inputs  */
                 GRP_UP_PORT_IN,     GRP_UP_PIN_IN,
                 GRP_DOWN_PORT_IN,   GRP_DOWN_PIN_IN,
                 GRP_CLAW_PORT_IN,   GRP_CLAW_PIN_IN);

    set_state(robot, ROBOT_IDLE, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motion commands
 * ═══════════════════════════════════════════════════════════════════════════ */

void Robot_Move(Robot_t *robot, float target_rad)
{
    Trajectory_SetTargetConstrained(&robot->traj, robot->theta, target_rad);
    uint32_t timeout = (uint32_t)(robot->traj.T_f * 1000.0f) + 2000;
    set_state(robot, ROBOT_MOVE, timeout);
}

void Robot_Home(Robot_t *robot)
{
    /* Move target = RBT_DEFAULT_HOMING_OFFSET (output shaft) */
    robot->home_goto    = RBT_DEFAULT_HOMING_OFFSET;
    robot->ls_hit       = 0;
    reset_control(robot);
    robot->omega_target = RBT_HOMING_FAST;
    set_state(robot, ROBOT_HOMING_FAST_STATE, 15000);
}

void Robot_SetHome(Robot_t *robot, float new_home_rad)
{
    /* Move target = RBT_DEFAULT_HOMING_OFFSET + new_home_rad (output shaft) */
    robot->home_goto    = RBT_DEFAULT_HOMING_OFFSET + new_home_rad;
    robot->ls_hit       = 0;
    reset_control(robot);
    robot->omega_target = RBT_HOMING_FAST;
    set_state(robot, ROBOT_HOMING_FAST_STATE, 15000);
}

void Robot_Stop(Robot_t *robot)
{
    Trajectory_Stop(&robot->traj);
    robot->theta_target = robot->theta;
    reset_control(robot);
    set_state(robot, ROBOT_IDLE, 0);
}

void Robot_EStop(Robot_t *robot)
{
    MD20A_stop(&robot->driver);
    reset_control(robot);
    set_state(robot, ROBOT_ESTOP, 0);
}

/* ── Jog ─────────────────────────────────────────────────────────────────── */

void Robot_JogVel(Robot_t *robot, float speed_rad_s)
{
    robot->jog_speed = Saturate(speed_rad_s, -robot->omega_max, robot->omega_max);
    reset_control(robot);
    set_state(robot, ROBOT_JOG_VEL, 0);
}

void Robot_JogStep(Robot_t *robot, float step_rad)
{
    robot->jog_step = step_rad;
    float target = robot->theta + step_rad;
    Trajectory_SetTargetConstrained(&robot->traj, robot->theta, target);
    uint32_t timeout = (uint32_t)(robot->traj.T_f * 1000.0f) + 2000;
    set_state(robot, ROBOT_JOG_STEP, timeout);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper commands  (thin wrappers around Gripper.h API)
 * ═══════════════════════════════════════════════════════════════════════════ */

void Robot_Gripper_MoveUp  (Robot_t *robot) { Gripper_MoveUp  (&robot->gripper); }
void Robot_Gripper_MoveDown(Robot_t *robot) { Gripper_MoveDown(&robot->gripper); }
void Robot_Gripper_Open    (Robot_t *robot) { Gripper_Open    (&robot->gripper); }
void Robot_Gripper_Close   (Robot_t *robot) { Gripper_Close   (&robot->gripper); }

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper state readers
 * ═══════════════════════════════════════════════════════════════════════════ */

GripperState_t Robot_Gripper_GetUpState  (Robot_t *robot) { return Gripper_Up_State  (&robot->gripper); }
GripperState_t Robot_Gripper_GetDownState(Robot_t *robot) { return Gripper_Down_State(&robot->gripper); }
GripperState_t Robot_Gripper_GetClawState(Robot_t *robot) { return Gripper_Claw_State(&robot->gripper); }

/* ═══════════════════════════════════════════════════════════════════════════
 *  Robot_EXTI_Callback
 * ═══════════════════════════════════════════════════════════════════════════ */
void Robot_EXTI_Callback(Robot_t *robot, uint16_t GPIO_Pin)
{
    if (robot->ls_port != NULL && GPIO_Pin == robot->ls_pin)
        robot->ls_hit = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Robot_Update
 * ═══════════════════════════════════════════════════════════════════════════ */
void Robot_Update(Robot_t *robot, TIM_HandleTypeDef *htim)
{
    if (htim != robot->htim_ctrl)  return;
    if (!robot->kalman.started)    return;

    /* ── 1. Encoder ──────────────────────────────────────────────────────── */
    QEI_update(&robot->encoder, htim);

    /* ── 2. Kalman ───────────────────────────────────────────────────────── */
    KalmanFilterDCMotor_Update(&robot->kalman,
                               (float)robot->encoder.Rad,
                               robot->u_prev);

    robot->theta = (robot->encoder.Rad - robot->home_offset) / robot->N;
    robot->omega =  KalmanFilterDCMotor_Get_Velocity   (&robot->kalman) / robot->N;
    robot->tau_d =  KalmanFilterDCMotor_Get_Disturbance(&robot->kalman);

    /* ── 3. SLOW LOOP ────────────────────────────────────────────────────── */
    if (++robot->pos_tick >= CTRL_LOOP_MULTI)
    {
        robot->pos_tick = 0;

        /* Timeout watchdog */
        if (robot->state != ROBOT_IDLE               &&
            robot->state != ROBOT_JOG_VEL            &&
            robot->state != ROBOT_ESTOP              &&
            robot->timeout_ms > 0)
        {
            if ((HAL_GetTick() - robot->state_tick) > robot->timeout_ms)
                Robot_Stop(robot);
        }

        switch (robot->state)
        {
            /* ── IDLE: hold position ─────────────────────────────────────── */
            case ROBOT_IDLE:
                robot->omega_target = Saturate(
                    PID_Update(&robot->pid_pos,
                               robot->theta_target, robot->theta),
                    -robot->omega_max, robot->omega_max);
                break;

            /* ── MOVE: quintic trajectory ────────────────────────────────── */
            case ROBOT_MOVE:
                Trajectory_Update(&robot->traj);
                robot->omega_target = Saturate(
                    PID_Update(&robot->pid_pos,
                               robot->traj.theta_ref, robot->theta)
                    + robot->traj.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->traj.active)
                {
                    robot->theta_target = robot->traj.theta_f;
                    set_state(robot, ROBOT_IDLE, 0);
                }
                break;

            /* ── JOG VEL: continuous velocity, no position loop ──────────── */
            case ROBOT_JOG_VEL:
                robot->omega_target = robot->jog_speed;
                break;

            /* ── JOG STEP: one constrained move then hold ────────────────── */
            case ROBOT_JOG_STEP:
                Trajectory_Update(&robot->traj);
                robot->omega_target = Saturate(
                    PID_Update(&robot->pid_pos,
                               robot->traj.theta_ref, robot->theta)
                    + robot->traj.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->traj.active)
                {
                    robot->theta_target = robot->traj.theta_f;
                    set_state(robot, ROBOT_IDLE, 0);
                }
                break;

            /* ── HOMING: fast approach ───────────────────────────────────── */
            case ROBOT_HOMING_FAST_STATE:
                robot->omega_target = RBT_HOMING_FAST;
                if (robot->ls_hit)
                {
                    robot->ls_hit = 0;
                    Trajectory_SetTargetConstrained(&robot->traj,
                                                    robot->theta,
                                                    robot->theta + RBT_HOMING_BACKOFF);
                    set_state(robot, ROBOT_HOMING_BACKOFF_STATE, 3000);
                }
                break;

            /* ── HOMING: backoff ─────────────────────────────────────────── */
            case ROBOT_HOMING_BACKOFF_STATE:
                Trajectory_Update(&robot->traj);
                robot->omega_target = Saturate(
                    PID_Update(&robot->pid_pos,
                               robot->traj.theta_ref, robot->theta)
                    + robot->traj.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->traj.active)
                {
                    robot->ls_hit       = 0;
                    robot->omega_target = RBT_HOMING_SLOW;
                    set_state(robot, ROBOT_HOMING_SLOW_STATE, 10000);
                }
                break;

            /* ── HOMING: slow creep ──────────────────────────────────────── */
            case ROBOT_HOMING_SLOW_STATE:
                robot->omega_target = RBT_HOMING_SLOW;
                if (robot->ls_hit)
                {
                    robot->ls_hit = 0;
                    /* Step 1: Set zero at limit switch */
                    robot->home_offset  = robot->encoder.Rad;
                    robot->theta_target = 0.0f;
                    /* Step 2: Move to home_goto (set by Robot_Home or Robot_SetHome) */
                    Trajectory_SetTargetConstrained(&robot->traj, 0.0f, robot->home_goto);
                    set_state(robot, ROBOT_HOMING_OFFSET_STATE, 5000);
                }
                break;

            /* ── HOMING: move to final offset then re-zero ───────────────── */
            case ROBOT_HOMING_OFFSET_STATE:
                Trajectory_Update(&robot->traj);
                robot->omega_target = Saturate(
                    PID_Update(&robot->pid_pos,
                               robot->traj.theta_ref, robot->theta)
                    + robot->traj.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->traj.active)
                {
                    /* Robot has arrived at the offset position.
                     * Re-zero here so this becomes the true theta = 0. */
                    robot->home_offset  = robot->encoder.Rad;
                    robot->theta_target = 0.0f;
                    set_state(robot, ROBOT_IDLE, 0);
                }
                break;

            case ROBOT_ESTOP:
                robot->omega_target = 0.0f;
                break;

            default:
                break;
        }
    }

    /* ── 4. FAST LOOP ────────────────────────────────────────────────────── */
    float u_total = 0.0f;

    if (robot->state != ROBOT_ESTOP)
    {
        float u_pid = PID_Update(&robot->pid_vel,
                                 robot->omega_target,
                                 robot->omega);

        float omega_ff = 0.0f;
        if (robot->state == ROBOT_MOVE                 ||
            robot->state == ROBOT_JOG_STEP             ||
            robot->state == ROBOT_HOMING_BACKOFF_STATE  ||
            robot->state == ROBOT_HOMING_OFFSET_STATE)
        {
            /* traj.omega_ref is output shaft — convert to motor shaft for FF */
            omega_ff = robot->traj.omega_ref * robot->N;
        }

        u_total = u_pid + FF_Compute(&robot->ff, omega_ff, robot->tau_d);
    }

    robot->u_prev = u_total;

    MD20A_setSpeed(&robot->driver,
                   (Saturate(u_total, -robot->V_max, robot->V_max)
                    / robot->V_max) * 100.0f);
}

/* ── Getters ─────────────────────────────────────────────────────────────── */
float         Robot_GetPosition   (const Robot_t *robot) { return robot->theta; }
float         Robot_GetVelocity   (const Robot_t *robot) { return robot->omega; }
float         Robot_GetDisturbance(const Robot_t *robot) { return robot->tau_d; }
Robot_State_t Robot_GetState      (const Robot_t *robot) { return robot->state; }
uint8_t       Robot_IsIdle        (const Robot_t *robot) { return robot->state == ROBOT_IDLE; }
