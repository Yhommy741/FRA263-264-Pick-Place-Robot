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
 *  Robot_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void Robot_Init(Robot_t *robot, const Robot_Config_t *cfg)
{
    memset(robot, 0, sizeof(Robot_t));

    robot->htim_ctrl = cfg->htim_ctrl;
    robot->Ts        = cfg->Ts;
    robot->V_max     = cfg->V_max;
    robot->omega_max = cfg->omega_max;
    robot->N         = (cfg->N > 0.0f) ? cfg->N : 1.0f;
    robot->ls_port   = cfg->ls_port;
    robot->ls_pin    = cfg->ls_pin;

    DCMotor_Init(&robot->motor,
                 cfg->Rm, cfg->Lm,
                 cfg->Ke, cfg->Kt,
                 cfg->J,  cfg->b);

    MD20A_init(&robot->driver, cfg->htim_pwm, cfg->ch_dir, cfg->ch_pwm);

    QEI_init(&robot->encoder,
             cfg->htim_encoder,
             cfg->htim_ctrl,
             cfg->enc_ppr,
             cfg->enc_x,
             cfg->enc_overflow,
             cfg->Ts);

    KalmanFilterDCMotor_Init            (&robot->kalman, &robot->motor);
    KalmanFilterDCMotor_Set_ObserverPeriod  (&robot->kalman, cfg->Ts);
    KalmanFilterDCMotor_Set_ProcessNoise    (&robot->kalman, cfg->kf_var_tau_d);
    KalmanFilterDCMotor_Set_MeasurementNoise(&robot->kalman, cfg->kf_var_theta);
    KalmanFilterDCMotor_Start               (&robot->kalman, 0.0f, 0.0f, 0.0f, 0.0f);

    PID_Init(&robot->pid_vel,
             cfg->Kp_vel, cfg->Ki_vel, cfg->Kd_vel,
             cfg->Ts,
             -cfg->V_max, cfg->V_max);

    PID_Init(&robot->pid_pos,
             cfg->Kp_pos, cfg->Ki_pos, cfg->Kd_pos,
             cfg->Ts * ROBOT_POS_DIVIDER,
             -cfg->omega_max, cfg->omega_max);

    FF_Init(&robot->ff, &robot->motor);

    Trajectory_Init(&robot->traj,
                    cfg->Ts * ROBOT_POS_DIVIDER,
                    cfg->omega_max,
                    cfg->alpha_max);

    set_state(robot, ROBOT_IDLE, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Commands
 * ═══════════════════════════════════════════════════════════════════════════ */

void Robot_Move(Robot_t *robot, float target_rad)
{
    Trajectory_SetTargetConstrained(&robot->traj, robot->theta, target_rad);
    uint32_t timeout = (uint32_t)(robot->traj.T_f * 1000.0f) + 2000;
    set_state(robot, ROBOT_MOVE, timeout);
}

void Robot_Home(Robot_t *robot)
{
    robot->home_goto    = 0.0f;   /* always return to 0 after homing */
    robot->ls_hit       = 0;
    reset_control(robot);
    robot->omega_target = ROBOT_HOMING_VEL_FAST_RAD;
    set_state(robot, ROBOT_HOMING_FAST, 15000);
}

void Robot_SetHome(Robot_t *robot)
{
    robot->home_offset  = KalmanFilterDCMotor_Get_Position(&robot->kalman);
    robot->theta        = 0.0f;
    robot->theta_target = 0.0f;
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
    set_state(robot, ROBOT_JOG_VEL, 0);   /* no timeout — runs until Stop */
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

    /* ── 3. SLOW LOOP ─────────────────────────────────────────────────────── */
    if (++robot->pos_tick >= ROBOT_POS_DIVIDER)
    {
        robot->pos_tick = 0;

        /* Timeout watchdog */
        if (robot->state != ROBOT_IDLE  &&
            robot->state != ROBOT_JOG_VEL &&
            robot->state != ROBOT_ESTOP &&
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
            case ROBOT_HOMING_FAST:
                robot->omega_target = ROBOT_HOMING_VEL_FAST_RAD;
                if (robot->ls_hit)
                {
                    robot->ls_hit = 0;
                    Trajectory_SetTarget(&robot->traj,
                                         robot->theta,
                                         robot->theta + ROBOT_HOMING_BACKOFF_RAD,
                                         1.0f);
                    set_state(robot, ROBOT_HOMING_BACKOFF, 3000);
                }
                break;

            /* ── HOMING: backoff ─────────────────────────────────────────── */
            case ROBOT_HOMING_BACKOFF:
                Trajectory_Update(&robot->traj);
                robot->omega_target = Saturate(
                    PID_Update(&robot->pid_pos,
                               robot->traj.theta_ref, robot->theta)
                    + robot->traj.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->traj.active)
                {
                    robot->ls_hit       = 0;
                    robot->omega_target = ROBOT_HOMING_VEL_SLOW_RAD;
                    set_state(robot, ROBOT_HOMING_SLOW, 10000);
                }
                break;

            /* ── HOMING: slow creep ──────────────────────────────────────── */
            case ROBOT_HOMING_SLOW:
                robot->omega_target = ROBOT_HOMING_VEL_SLOW_RAD;
                if (robot->ls_hit)
                {
                    robot->ls_hit = 0;
                    Robot_SetHome(robot);
                    Trajectory_SetTarget(&robot->traj, 0.0f, robot->home_goto, 2.0f);
                    set_state(robot, ROBOT_HOMING_OFFSET, 5000);
                }
                break;

            /* ── HOMING: move to final offset ────────────────────────────── */
            case ROBOT_HOMING_OFFSET:
                Trajectory_Update(&robot->traj);
                robot->omega_target = Saturate(
                    PID_Update(&robot->pid_pos,
                               robot->traj.theta_ref, robot->theta)
                    + robot->traj.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->traj.active)
                {
                    robot->theta_target = robot->home_goto;
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

    /* ── 4. FAST LOOP ─────────────────────────────────────────────────────── */
    float u_total = 0.0f;

    if (robot->state != ROBOT_ESTOP)
    {
        float u_pid = PID_Update(&robot->pid_vel,
                                 robot->omega_target,
                                 robot->omega);

        float omega_ff = 0.0f;
        if (robot->state == ROBOT_MOVE          ||
            robot->state == ROBOT_JOG_STEP      ||
            robot->state == ROBOT_HOMING_BACKOFF ||
            robot->state == ROBOT_HOMING_OFFSET)
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
