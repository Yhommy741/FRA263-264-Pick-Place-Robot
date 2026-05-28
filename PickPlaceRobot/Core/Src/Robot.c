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
    Controller_Reset(&robot->ctr);
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

    /* ── Controller (Cascade PID + Feedforward) ─────────────────────────── */
    Controller_Init(&robot->ctr, &robot->motor,
                    KP_VEL, KI_VEL, KD_VEL,
                    KP_POS, KI_POS, KD_POS,
                    CTRL_PERIOD,
                    CTRL_LOOP_MULTI,
                    RBT_MAX_SPEED * SPEED_RATIO,
                    MOTOR_V_MAX);
    robot->ctr.ControlMode = CTRL_MODE_CASCADE_FF_ALL;

    /* ── Trajectory generator (S-curve, jerk-limited) ───────────────────── */
    Trajectory_Init(&robot->traj,
                    CTRL_PERIOD * CTRL_LOOP_MULTI,
                    RBT_MAX_SPEED,
                    RBT_MAX_ACCEL,
                    RBT_MAX_JERK);

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
    /* ── Shortest-path wrap ───────────────────────────────────────────────
     * target_rad is a logical position (0–2π range).
     * robot->theta accumulates absolutely and never resets.
     * Normalise delta to (−π, +π] so the robot always travels the
     * shorter arc — e.g. 0°→340° goes CW by 20° not CCW by 340°.
     * ──────────────────────────────────────────────────────────────────── */
    const float TWO_PI = 6.28318530718f;
    float delta = target_rad - robot->theta;

    while (delta >  3.14159265359f) delta -= TWO_PI;
    while (delta < -3.14159265359f) delta += TWO_PI;

    float abs_target = robot->theta + delta;

    Trajectory_SetTargetConstrained(&robot->traj, robot->theta, abs_target);
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

    /* ── 3. SLOW LOOP (outer position loop, runs every CTRL_LOOP_MULTI ticks) ─ */
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
                    PID_Update(&robot->ctr.pid_pos,
                               robot->theta_target, robot->theta),
                    -robot->omega_max, robot->omega_max);
                break;

            /* ── MOVE / JOG_STEP / HOMING trajectory states ─────────────── */
            case ROBOT_MOVE:
            case ROBOT_JOG_STEP:
            case ROBOT_HOMING_BACKOFF_STATE:
            case ROBOT_HOMING_OFFSET_STATE:
                Trajectory_Update(&robot->traj);
                robot->omega_target = Saturate(
                    PID_Update(&robot->ctr.pid_pos,
                               robot->traj.theta_ref, robot->theta)
                    + robot->traj.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->traj.active)
                {
                    robot->theta_target = robot->traj.theta_f;
                    if (robot->state == ROBOT_HOMING_OFFSET_STATE)
                    {
                        robot->home_offset  = robot->encoder.Rad;
                        robot->theta_target = 0.0f;
                    }
                    else if (robot->state == ROBOT_HOMING_BACKOFF_STATE)
                    {
                        robot->ls_hit = 0;
                        set_state(robot, ROBOT_HOMING_SLOW_STATE, 10000);
                        break;
                    }
                    set_state(robot, ROBOT_IDLE, 0);
                }
                break;

            /* ── JOG VEL: bypass pos loop, command velocity directly ──────── */
            case ROBOT_JOG_VEL:
                robot->omega_target = robot->jog_speed;
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

            /* ── HOMING: slow creep ──────────────────────────────────────── */
            case ROBOT_HOMING_SLOW_STATE:
                robot->omega_target = RBT_HOMING_SLOW;
                if (robot->ls_hit)
                {
                    robot->ls_hit       = 0;
                    robot->home_offset  = robot->encoder.Rad;
                    robot->theta_target = 0.0f;
                    Trajectory_SetTargetConstrained(&robot->traj, 0.0f, robot->home_goto);
                    set_state(robot, ROBOT_HOMING_OFFSET_STATE, 5000);
                }
                break;

            case ROBOT_ESTOP:
                robot->omega_target = 0.0f;
                break;

            default:
                break;
        }
    }

    /* ── 4. FAST LOOP (inner velocity loop, runs every tick) ────────────── */
    float u_total = 0.0f;

    if (robot->state != ROBOT_ESTOP)
    {
        /* Motor-shaft velocity reference */
        float omega_ref_motor = robot->omega_target * robot->N;

        /* Motor-shaft measured velocity (direct from Kalman — no division) */
        float omega_meas_motor = KalmanFilterDCMotor_Get_Velocity(&robot->kalman);

        float u_pid = PID_Update(&robot->ctr.pid_vel,
                                 omega_ref_motor,
                                 omega_meas_motor);

        float omega_ff = 0.0f;
        if (robot->state == ROBOT_MOVE                  ||
            robot->state == ROBOT_JOG_STEP              ||
            robot->state == ROBOT_HOMING_BACKOFF_STATE  ||
            robot->state == ROBOT_HOMING_OFFSET_STATE)
        {
            /* traj.omega_ref is output-shaft — scale to motor-shaft for FF */
            omega_ff = robot->traj.omega_ref * robot->N;
        }

        float u_ff = FF_Compute(&robot->ctr.ff, omega_ff, robot->tau_d,
                                robot->ctr.pid_vel.Ts);

        u_total = Saturate(u_pid + u_ff, -robot->V_max, robot->V_max);
    }

    robot->u_prev = u_total;

    MD20A_setSpeed(&robot->driver, (u_total / robot->V_max) * 100.0f);

    /* ── 5. Gripper pulse timeout (non-blocking) ─────────────────────────── */
    Gripper_Update(&robot->gripper);
}

/* ── Getters ─────────────────────────────────────────────────────────────── */
float         Robot_GetPosition   (const Robot_t *robot) { return robot->theta; }
float         Robot_GetVelocity   (const Robot_t *robot) { return robot->omega; }
float         Robot_GetDisturbance(const Robot_t *robot) { return robot->tau_d; }
Robot_State_t Robot_GetState      (const Robot_t *robot) { return robot->state; }
uint8_t       Robot_IsIdle        (const Robot_t *robot) { return robot->state == ROBOT_IDLE; }
