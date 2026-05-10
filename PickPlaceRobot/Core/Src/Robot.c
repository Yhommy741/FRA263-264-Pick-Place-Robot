#include "Robot.h"

/* ── Private Tuning Constants (Now evaluated in Output Shaft Space) ──────── */
#define HOMING_VEL_FAST_CW   ( 1.0f)   /* rad/s (at output shaft) */
#define HOMING_VEL_SLOW_CW   ( 0.2f)   /* rad/s (at output shaft) */
#define HOMING_BACKOFF_RAD   (-0.5f)   /* rad to back up (at output shaft) */

/* ── Helper: Transition State ────────────────────────────────────────────── */
static void Set_State(Robot_t *robot, Robot_State_t new_state, uint32_t timeout) {
    robot->state = new_state;
    robot->state_start_tick = HAL_GetTick();
    robot->timeout_ms = timeout;
}

/* ── Initialization ──────────────────────────────────────────────────────── */
void Robot_Init(Robot_t *robot, MD20A_t *driver, QEI_t *encoder,
                KalmanFilterDCMotor_t *kalman, DCMotor_t *motor,
                TIM_HandleTypeDef *htim, float Ts,
                GPIO_TypeDef *LS_Port, uint16_t LS_Pin,
                float N)
{
    robot->driver = driver;
    robot->encoder = encoder;
    robot->kalman = kalman;
    robot->motor = motor;

    robot->htim = htim;
    robot->Ts = Ts;
    robot->pos_tick_counter = 0;

    /* Set Gear Reduction Ratio */
    robot->N = N;

    robot->LS_GPIO_Port = LS_Port;
    robot->LS_Pin = LS_Pin;
    robot->limit_switch_hit_flag = 0;

    /* Initialize Controllers */
    /* Velocity runs at Ts, Position runs at Ts * 10 */
    PID_Init(&robot->pid_vel, 1.5f, 5.0f, 0.0f, Ts, -24.0f, 24.0f);
    PID_Init(&robot->pid_pos, 8.0f, 0.0f, 0.2f, Ts * 10.0f, -15.0f, 15.0f);

    Trajectory_Init(&robot->traj, Ts * 10.0f);

    robot->theta_offset = 0.0f;
    robot->V_max = 24.0f;
    robot->u_prev_volts = 0.0f;

    Set_State(robot, ROBOT_STATE_IDLE, 0);
    robot->theta_target = 0.0f;
    robot->omega_target = 0.0f;
}

void Robot_Set_Home(Robot_t *robot) {
    /* Offset is calculated in the Output Shaft Space */
    float motor_theta = KalmanFilterDCMotor_Get_Position(robot->kalman);
    robot->theta_offset = motor_theta / robot->N;
    robot->theta_target = 0.0f;
}

/* ── Commands ────────────────────────────────────────────────────────────── */
void Robot_Command_Home(Robot_t *robot, float offset_rad) {
    robot->home_offset_target = offset_rad;
    robot->limit_switch_hit_flag = 0;
    Set_State(robot, ROBOT_STATE_HOMING_FAST_CW, 10000);
    robot->omega_target = HOMING_VEL_FAST_CW;
}

void Robot_Command_Move(Robot_t *robot, float target_pos_rad, float time_s) {
    Trajectory_SetTarget(&robot->traj, robot->theta, target_pos_rad, time_s);
    Set_State(robot, ROBOT_STATE_MOVE, (uint32_t)(time_s * 1000.0f) + 2000);
}

/* ── EXTI Limit Switch Handler ───────────────────────────────────────────── */
void Robot_EXTI_Callback(Robot_t *robot, uint16_t GPIO_Pin) {
    if (GPIO_Pin == robot->LS_Pin) {
        robot->limit_switch_hit_flag = 1;
    }
}

/* ── Main Dual-Rate Update Loop ──────────────────────────────────────────── */
void Robot_Update(Robot_t *robot, TIM_HandleTypeDef *htim) {

    /* Only execute if the callback was triggered by the Robot's designated timer */
    if (htim == robot->htim) {

        /* 1. Hardware Read: Update Encoder */
        QEI_update(robot->encoder, htim);
        float raw_theta = (float)robot->encoder->Rad;

        /* 2. State Estimation: Update Kalman Filter */
        KalmanFilterDCMotor_Update(robot->kalman, raw_theta, robot->u_prev_volts);

        /* 3. Fetch Local State Estimates & Scale to Output Shaft using N */
        float motor_theta = KalmanFilterDCMotor_Get_Position(robot->kalman);
        robot->theta = (motor_theta / robot->N) - robot->theta_offset;

        float motor_omega = KalmanFilterDCMotor_Get_Velocity(robot->kalman);
        robot->omega = motor_omega / robot->N;

        /* Disturbance remains in motor space because compensation is applied to motor voltage */
        robot->tau_d = KalmanFilterDCMotor_Get_Disturbance(robot->kalman);

        /* =========================================================================
         * SLOW LOOP (Position & State Machine) -- Runs at Ts * 10
         * ========================================================================= */
        if (++robot->pos_tick_counter >= 10) {
            robot->pos_tick_counter = 0;

            uint32_t now = HAL_GetTick();

            /* Check Timeouts */
            if (robot->state != ROBOT_STATE_IDLE && robot->timeout_ms > 0) {
                if ((now - robot->state_start_tick) > robot->timeout_ms) {
                    robot->theta_target = robot->theta;
                    Set_State(robot, ROBOT_STATE_IDLE, 0);
                }
            }

            switch (robot->state) {
                case ROBOT_STATE_IDLE:
                    robot->omega_target = PID_Update(&robot->pid_pos, robot->theta_target, robot->theta);
                    break;

                case ROBOT_STATE_HOMING_FAST_CW:
                    robot->omega_target = HOMING_VEL_FAST_CW;
                    if (robot->limit_switch_hit_flag) {
                        robot->limit_switch_hit_flag = 0;
                        Trajectory_SetTarget(&robot->traj, robot->theta, robot->theta + HOMING_BACKOFF_RAD, 1.0f);
                        Set_State(robot, ROBOT_STATE_HOMING_BACKOFF_CCW, 3000);
                    }
                    break;

                case ROBOT_STATE_HOMING_BACKOFF_CCW:
                    Trajectory_Update(&robot->traj);
                    robot->theta_target = robot->traj.theta_ref;
                    robot->omega_target = PID_Update(&robot->pid_pos, robot->theta_target, robot->theta) + robot->traj.omega_ref;

                    if (!robot->traj.active) {
                        robot->omega_target = HOMING_VEL_SLOW_CW;
                        robot->limit_switch_hit_flag = 0;
                        Set_State(robot, ROBOT_STATE_HOMING_SLOW_CW, 5000);
                    }
                    break;

                case ROBOT_STATE_HOMING_SLOW_CW:
                    robot->omega_target = HOMING_VEL_SLOW_CW;
                    if (robot->limit_switch_hit_flag) {
                        robot->limit_switch_hit_flag = 0;
                        Robot_Set_Home(robot);
                        Trajectory_SetTarget(&robot->traj, 0.0f, robot->home_offset_target, 2.0f);
                        Set_State(robot, ROBOT_STATE_HOMING_GOTO_OFFSET, 5000);
                    }
                    break;

                case ROBOT_STATE_HOMING_GOTO_OFFSET:
                case ROBOT_STATE_MOVE:
                    Trajectory_Update(&robot->traj);
                    robot->theta_target = robot->traj.theta_ref;
                    robot->omega_target = PID_Update(&robot->pid_pos, robot->theta_target, robot->theta) + robot->traj.omega_ref;

                    if (!robot->traj.active) {
                        Set_State(robot, ROBOT_STATE_IDLE, 0);
                    }
                    break;
            }
        }

        /* =========================================================================
         * FAST LOOP (Velocity Control) -- Runs every Ts
         * ========================================================================= */
        float u_volts = PID_Update(&robot->pid_vel, robot->omega_target, robot->omega);

        /* Disturbance Feedforward Compensation (Calculated at Motor Shaft) */
        u_volts += (robot->motor->Rm / robot->motor->Kt) * robot->tau_d;

        /* Save voltage to predict next Kalman Filter state, then limit to bounds */
        robot->u_prev_volts = u_volts;

        if (u_volts > robot->V_max) u_volts = robot->V_max;
        if (u_volts < -robot->V_max) u_volts = -robot->V_max;

        /* Apply Output to Driver */
        float duty_cycle = (u_volts / robot->V_max) * 100.0f;
        MD20A_setSpeed(robot->driver, duty_cycle);
    }
}
