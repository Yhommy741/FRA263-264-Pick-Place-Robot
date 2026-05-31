/*
 * Robot.c
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 */

#include "Robot.h"
#include <string.h>
#include <math.h>

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
 *  Robot_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void Robot_Init(Robot_t *robot)
{
    memset(robot, 0, sizeof(Robot_t));

    robot->htim_ctrl = CTRL_HTIM_PTR;
    robot->Ts        = CTRL_PERIOD;
    robot->V_max     = MOTOR_V_MAX;
    robot->omega_max = RBT_MAX_SPEED;
    robot->N         = SPEED_RATIO;
    robot->ls_port   = LIM_SW_PORT;
    robot->ls_pin    = LIM_SW_PIN;

    robot->motor.Rm = MOTOR_RM;
    robot->motor.Lm = MOTOR_LM;
    robot->motor.Ke = MOTOR_KE;
    robot->motor.Kt = MOTOR_KT;
    robot->motor.J  = MOTOR_J;
    robot->motor.b  = MOTOR_B;

    MD20A_init(&robot->driver, MDRV_HTIM_PTR, MDRV_DIR_CH, MDRV_PWM_CH);

    QEI_init(&robot->encoder,
             ENC_HTIM_PTR, CTRL_HTIM_PTR,
             ENC_PPR, ENC_X, ENC_OVERFLOW, CTRL_PERIOD);

    KalmanFilterDCMotor_Init             (&robot->kalman, &robot->motor);
    KalmanFilterDCMotor_Set_ObserverPeriod  (&robot->kalman, CTRL_PERIOD);
    KalmanFilterDCMotor_Set_ProcessNoise    (&robot->kalman, KF_VAR_TAU_D);
    KalmanFilterDCMotor_Set_MeasurementNoise(&robot->kalman, KF_VAR_THETA);
    KalmanFilterDCMotor_Start               (&robot->kalman, 0.0f, 0.0f, 0.0f, 0.0f);

    Controller_Init(&robot->ctr, &robot->motor,
                    KP_VEL, KI_VEL, KD_VEL,
                    KP_POS, KI_POS, KD_POS,
                    CTRL_PERIOD, CTRL_LOOP_MULTI,
                    RBT_MAX_SPEED * SPEED_RATIO, MOTOR_V_MAX);
    robot->ctr.ControlMode = CTRL_MODE_CASCADE_FF_ALL;

    SCurve_Init(&robot->scurve,
                CTRL_PERIOD * CTRL_LOOP_MULTI,
                RBT_MAX_SPEED, RBT_MAX_ACCEL, RBT_MAX_JERK);

    Trapezoid_Init(&robot->trap,
                   CTRL_PERIOD * CTRL_LOOP_MULTI,
                   RBT_MAX_SPEED, RBT_MAX_ACCEL);

    Gripper_Init(&robot->gripper,
                 GRP_UP_PORT_OUT,    GRP_UP_PIN_OUT,
                 GRP_DOWN_PORT_OUT,  GRP_DOWN_PIN_OUT,
                 GRP_OPEN_PORT_OUT,  GRP_OPEN_PIN_OUT,
                 GRP_CLOSE_PORT_OUT, GRP_CLOSE_PIN_OUT,
                 GRP_UP_PORT_IN,     GRP_UP_PIN_IN,
                 GRP_DOWN_PORT_IN,   GRP_DOWN_PIN_IN,
                 GRP_CLAW_PORT_IN,   GRP_CLAW_PIN_IN);

    set_state(robot, ROBOT_IDLE, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Robot_SoftReset
 *
 * Resets robot to the same state as after Robot_Init, WITHOUT re-initialising
 * any hardware peripherals (no MD20A_init, QEI_init, etc.).
 * home_offset is preserved so re-homing is not required.
 * ═══════════════════════════════════════════════════════════════════════════ */
void Robot_SoftReset(Robot_t *robot)
{
    /* ── CRITICAL STEP: Cut physical motor power immediately like an E-Stop ── */
    MD20A_stop(&robot->driver);

    /* Save everything set up by hardware init calls */
    TIM_HandleTypeDef     *htim_ctrl   = robot->htim_ctrl;
    float                  Ts          = robot->Ts;
    float                  V_max       = robot->V_max;
    float                  omega_max   = robot->omega_max;
    float                  N           = robot->N;
    GPIO_TypeDef          *ls_port     = robot->ls_port;
    uint16_t               ls_pin      = robot->ls_pin;
    float                  home_offset = robot->home_offset;

    /* Copy data profiles from the robot pointer handle */
    MD20A_t                driver      = robot->driver;
    QEI_t                  encoder     = robot->encoder;
    DCMotor_t              motor       = robot->motor;
    KalmanFilterDCMotor_t  kalman      = robot->kalman;
    Controller_t           ctr         = robot->ctr;
    Gripper_t              gripper     = robot->gripper;

    /* Compute current position BEFORE memset so theta_target is correct */
    float theta_now = ((float)encoder.Rad - home_offset) / N;

    /* ── Disable IRQ during memset to prevent Robot_Update reading             *
     * zeroed state (theta_target=0 would command a move to 0°).            */
    __disable_irq();

    memset(robot, 0, sizeof(Robot_t));

    /* Restore hardware handles and config */
    robot->htim_ctrl   = htim_ctrl;
    robot->Ts          = Ts;
    robot->V_max       = V_max;
    robot->omega_max   = omega_max;
    robot->N           = N;
    robot->ls_port     = ls_port;
    robot->ls_pin      = ls_pin;
    robot->home_offset = home_offset;
    robot->driver      = driver;
    robot->encoder     = encoder;
    robot->motor       = motor;
    robot->kalman      = kalman;
    robot->ctr         = ctr;
    robot->gripper     = gripper;

    /* Set targets safely to current location */
    robot->theta        = theta_now;
    robot->theta_target = theta_now;
    robot->omega_prev   = 0.0f;
    robot->u_prev       = 0.0f; /* Keep motor drive tracking dead at 0V */
    robot->omega_target = 0.0f;

    /* Re-init soft state */
    robot->ctr.ControlMode = CTRL_MODE_CASCADE_FF_ALL;

    /* LOCK STATE TO ESTOP: This blocks background loops from re-enabling the driver */
    set_state(robot, ROBOT_ESTOP, 0);

    __enable_irq();

    /* These can run after IRQ re-enabled — they don't touch theta_target */
    SCurve_Init(&robot->scurve, CTRL_PERIOD * CTRL_LOOP_MULTI, RBT_MAX_SPEED, RBT_MAX_ACCEL, RBT_MAX_JERK);
    Trapezoid_Init(&robot->trap, CTRL_PERIOD * CTRL_LOOP_MULTI, RBT_MAX_SPEED, RBT_MAX_ACCEL);
    Controller_Reset(&robot->ctr);
    KalmanFilterDCMotor_Start(&robot->kalman, (float)robot->encoder.Rad, 0.0f, 0.0f, 0.0f);
}
/* ═══════════════════════════════════════════════════════════════════════════
 *  Motion commands
 * ═══════════════════════════════════════════════════════════════════════════ */

void Robot_Move(Robot_t *robot, float target_rad)
{
    const float TWO_PI = 6.28318530718f;
    float delta = target_rad - robot->theta;
    while (delta >  3.14159265359f) delta -= TWO_PI;
    while (delta < -3.14159265359f) delta += TWO_PI;
    float abs_target = robot->theta + delta;

    SCurve_SetTarget(&robot->scurve, robot->theta, abs_target);
    uint32_t timeout = (uint32_t)(robot->scurve.T_f * 1000.0f) + 2000;
    set_state(robot, ROBOT_MOVE, timeout);
}

void Robot_MoveConstrained(Robot_t *robot, float target_rad,
                            float omega_max, float alpha_max)
{
    if (omega_max <= 0.0f || omega_max > RBT_MAX_SPEED)  omega_max = RBT_MAX_SPEED;
    if (alpha_max <= 0.0f || alpha_max > RBT_MAX_ACCEL)  alpha_max = RBT_MAX_ACCEL;

    float saved_omega = robot->scurve.omega_max;
    float saved_alpha = robot->scurve.alpha_max;
    robot->scurve.omega_max = omega_max;
    robot->scurve.alpha_max = alpha_max;

    SCurve_SetTarget(&robot->scurve, robot->theta, target_rad);
    uint32_t timeout = (uint32_t)(robot->scurve.T_f * 1000.0f) + 2000;
    set_state(robot, ROBOT_MOVE, timeout);

    robot->scurve.omega_max = saved_omega;
    robot->scurve.alpha_max = saved_alpha;
}

/* ── Performance test ────────────────────────────────────────────────────── */

void Robot_PerfTest_Start(Robot_t *robot, float target_rad,
                           float v_target, float a_target)
{
    if (v_target <= 0.0f || v_target > RBT_MAX_SPEED)  v_target = RBT_MAX_SPEED;
    if (a_target <= 0.0f || a_target > RBT_MAX_ACCEL)  a_target = RBT_MAX_ACCEL;

    float stroke = fabsf(target_rad - robot->theta);
    if (stroke < 1e-4f) return;

    /* Guarantee cruise phase: boost a if needed, reduce v only if hw limit hit */
    float a_needed = (v_target * v_target) / stroke;
    if (a_needed <= RBT_MAX_ACCEL)
    {
        if (a_needed > a_target) a_target = a_needed;
    }
    else
    {
        a_target = RBT_MAX_ACCEL;
        v_target = sqrtf(a_target * stroke);
    }

    Trapezoid_SetTargetConstrained(&robot->trap,
                                   robot->theta, target_rad,
                                   v_target, a_target);

    uint32_t timeout = (uint32_t)(robot->trap.T_f * 1000.0f) + 2000;
    set_state(robot, ROBOT_PERF_TEST, timeout);
}

/* ── Homing ──────────────────────────────────────────────────────────────── */

void Robot_Home(Robot_t *robot)
{
    robot->home_goto    = RBT_DEFAULT_HOMING_OFFSET;
    robot->ls_hit       = 0;
    reset_control(robot);
    robot->omega_target = RBT_HOMING_FAST;
    set_state(robot, ROBOT_HOMING_FAST_STATE, 15000);
}

void Robot_SetHome(Robot_t *robot, float new_home_rad)
{
    robot->home_goto    = RBT_DEFAULT_HOMING_OFFSET + new_home_rad;
    robot->ls_hit       = 0;
    reset_control(robot);
    robot->omega_target = RBT_HOMING_FAST;
    set_state(robot, ROBOT_HOMING_FAST_STATE, 15000);
}

void Robot_Stop(Robot_t *robot)
{
    SCurve_Stop(&robot->scurve);
    Trapezoid_Stop(&robot->trap);
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
    float target = robot->theta + step_rad;
    SCurve_SetTarget(&robot->scurve, robot->theta, target);
    uint32_t timeout = (uint32_t)(robot->scurve.T_f * 1000.0f) + 2000;
    set_state(robot, ROBOT_JOG_STEP, timeout);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper
 * ═══════════════════════════════════════════════════════════════════════════ */

void Robot_Gripper_MoveUp  (Robot_t *robot) { Gripper_MoveUp  (&robot->gripper); }
void Robot_Gripper_MoveDown(Robot_t *robot) { Gripper_MoveDown(&robot->gripper); }
void Robot_Gripper_Open    (Robot_t *robot) { Gripper_Open    (&robot->gripper); }
void Robot_Gripper_Close   (Robot_t *robot) { Gripper_Close   (&robot->gripper); }

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

    robot->alpha      = (robot->omega - robot->omega_prev) / CTRL_PERIOD;
    robot->omega_prev =  robot->omega;

    /* ── 3. SLOW LOOP ────────────────────────────────────────────────────── */
    if (++robot->pos_tick >= CTRL_LOOP_MULTI)
    {
        robot->pos_tick = 0;

        if (robot->state != ROBOT_IDLE    &&
            robot->state != ROBOT_JOG_VEL &&
            robot->state != ROBOT_ESTOP   &&
            robot->timeout_ms > 0)
        {
            if ((HAL_GetTick() - robot->state_tick) > robot->timeout_ms)
                Robot_Stop(robot);
        }

        switch (robot->state)
        {
            case ROBOT_IDLE:
                robot->omega_target = Saturate(
                    PID_Update(&robot->ctr.pid_pos,
                               robot->theta_target, robot->theta),
                    -robot->omega_max, robot->omega_max);
                break;

            case ROBOT_MOVE:
            case ROBOT_JOG_STEP:
            case ROBOT_HOMING_BACKOFF_STATE:
            case ROBOT_HOMING_OFFSET_STATE:
                SCurve_Update(&robot->scurve);
                robot->omega_target = Saturate(
                    PID_Update(&robot->ctr.pid_pos,
                               robot->scurve.theta_ref, robot->theta)
                    + robot->scurve.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->scurve.active)
                {
                    robot->theta_target = robot->scurve.theta_f;
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

            case ROBOT_PERF_TEST:
                Trapezoid_Update(&robot->trap, robot->theta);
                robot->omega_target = Saturate(
                    PID_Update(&robot->ctr.pid_pos,
                               robot->trap.theta_ref, robot->theta)
                    + robot->trap.omega_ref,
                    -robot->omega_max, robot->omega_max);

                if (!robot->trap.active)
                {
                    robot->theta_target = robot->trap.theta_f;
                    set_state(robot, ROBOT_IDLE, 0);
                }
                break;

            case ROBOT_JOG_VEL:
                robot->omega_target = robot->jog_speed;
                break;

            case ROBOT_HOMING_FAST_STATE:
                robot->omega_target = RBT_HOMING_FAST;
                if (robot->ls_hit)
                {
                    robot->ls_hit = 0;
                    SCurve_SetTarget(&robot->scurve,
                                     robot->theta,
                                     robot->theta + RBT_HOMING_BACKOFF);
                    set_state(robot, ROBOT_HOMING_BACKOFF_STATE, 3000);
                }
                break;

            case ROBOT_HOMING_SLOW_STATE:
                robot->omega_target = RBT_HOMING_SLOW;
                if (robot->ls_hit)
                {
                    robot->ls_hit       = 0;
                    robot->home_offset  = robot->encoder.Rad;
                    robot->theta_target = 0.0f;
                    SCurve_SetTarget(&robot->scurve, 0.0f, robot->home_goto);
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

    /* ── 4. FAST LOOP ────────────────────────────────────────────────────── */
    float u_total = 0.0f;

    if (robot->state != ROBOT_ESTOP)
    {
        float omega_ref_motor  = robot->omega_target * robot->N;
        float omega_meas_motor = KalmanFilterDCMotor_Get_Velocity(&robot->kalman);

        float u_pid = PID_Update(&robot->ctr.pid_vel,
                                 omega_ref_motor,
                                 omega_meas_motor);

        float omega_ff = 0.0f;
        if (robot->state == ROBOT_MOVE                 ||
            robot->state == ROBOT_JOG_STEP             ||
            robot->state == ROBOT_HOMING_BACKOFF_STATE ||
            robot->state == ROBOT_HOMING_OFFSET_STATE)
        {
            omega_ff = robot->scurve.omega_ref * robot->N;
        }
        else if (robot->state == ROBOT_PERF_TEST)
        {
            omega_ff = robot->trap.omega_ref * robot->N;
        }

        float u_ff = FF_Compute(&robot->ctr.ff, omega_ff, robot->tau_d,
                                robot->ctr.pid_vel.Ts);

        u_total = Saturate(u_pid + u_ff, -robot->V_max, robot->V_max);
    }

    robot->u_prev = u_total;
    MD20A_setSpeed(&robot->driver, (u_total / robot->V_max) * 100.0f);

    /* ── 5. Gripper ──────────────────────────────────────────────────────── */
    Gripper_Update(&robot->gripper);
}

/* ── Getters ─────────────────────────────────────────────────────────────── */
float         Robot_GetPosition    (const Robot_t *robot) { return robot->theta; }
float         Robot_GetVelocity    (const Robot_t *robot) { return robot->omega; }
float         Robot_GetAcceleration(const Robot_t *robot) { return robot->alpha; }
float         Robot_GetDisturbance (const Robot_t *robot) { return robot->tau_d; }
Robot_State_t Robot_GetState       (const Robot_t *robot) { return robot->state; }
uint8_t       Robot_IsIdle         (const Robot_t *robot) { return robot->state == ROBOT_IDLE; }
