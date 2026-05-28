/*
 * Robot.h
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "main.h"
#include "RobotConfig.h"
#include "QEI.h"
#include "MD20A.h"
#include "KalmanFilterDCMotor.h"   /* pulls in DCMotor.h transitively — Robot.h
                                      does NOT need to include it directly     */
#include "Controller.h"
#include "TrajectoryGen.h"
#include "Gripper.h"

/* ─────────────────────────────────────────────────────────────────────────
 * NOTE: RBT_HOMING_FAST / RBT_HOMING_SLOW / RBT_HOMING_BACKOFF are
 * float #defines in RobotConfig.h (velocity / distance values).
 * The state-machine enum uses a _STATE suffix to avoid name collision.
 * ───────────────────────────────────────────────────────────────────────── */

/* ── States ──────────────────────────────────────────────────────────────── */
typedef enum {
    ROBOT_IDLE,
    ROBOT_MOVE,
    ROBOT_JOG_VEL,              /* continuous velocity — runs until Robot_Stop()  */
    ROBOT_JOG_STEP,             /* single incremental step then → IDLE            */
    ROBOT_HOMING_FAST_STATE,    /* fast CW approach to limit switch               */
    ROBOT_HOMING_BACKOFF_STATE, /* CCW backoff after first LS hit                 */
    ROBOT_HOMING_SLOW_STATE,    /* slow CW creep for precise LS detection         */
    ROBOT_HOMING_OFFSET_STATE,  /* move to final home position                    */
    ROBOT_ESTOP
} Robot_State_t;

/* ── Handle ──────────────────────────────────────────────────────────────── */
typedef struct {
    /* Sub-modules */
    MD20A_t                 driver;
    QEI_t                   encoder;
    DCMotor_t               motor;      /* populated directly from RobotConfig.h */
    KalmanFilterDCMotor_t   kalman;
    Controller_t            ctr;
    Trajectory_t            traj;
    Gripper_t               gripper;

    /* Config (cached at init) */
    TIM_HandleTypeDef *htim_ctrl;
    float  Ts;
    float  V_max;
    float  omega_max;
    float  N;

    /* Limit switch */
    GPIO_TypeDef    *ls_port;
    uint16_t         ls_pin;
    volatile uint8_t ls_hit;

    /* State machine */
    Robot_State_t state;
    uint32_t      state_tick;
    uint32_t      timeout_ms;

    /* Measurements */
    float theta;
    float omega;
    float tau_d;

    /* Targets */
    float theta_target;
    float omega_target;
    float home_offset;
    float home_goto;

    /* Voltage fed back to Kalman each tick */
    float u_prev;

    /* Slow-loop counter (increments every fast tick, resets at CTRL_LOOP_MULTI) */
    uint8_t pos_tick;

    /* Jog parameters */
    float jog_speed;    /* [rad/s] — set by Robot_JogVel, sign = direction   */
    float jog_step;     /* [rad]   — set by Robot_JogStep, sign = direction  */
} Robot_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialisation — all config taken from RobotConfig.h
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise the entire robot (motor, encoder, Kalman, PID,
 *         trajectory, and gripper) using constants from RobotConfig.h.
 *         No config struct needed — just pass the robot handle.
 */
void Robot_Init(Robot_t *robot);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motion commands
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief  Smooth constrained move to target [rad]. */
void Robot_Move    (Robot_t *robot, float target_rad);

/** @brief  Full homing sequence. After done: theta=0, then move to 0. */
void Robot_Home    (Robot_t *robot);

/**
 * @brief  Declare the current physical position as the home reference.
 *         The coordinate frame is shifted so the limit-switch position
 *         becomes -(RBT_DEFAULT_HOMING_OFFSET + new_home_rad) and
 *         theta=0 lands at the clearance point.
 *         Pass 0.0f for standard homing; pass an additional offset if needed.
 */
void Robot_SetHome (Robot_t *robot, float new_home_rad);

/** @brief  Stop motion and hold current position. */
void Robot_Stop    (Robot_t *robot);

/** @brief  Emergency stop — disable motor immediately. */
void Robot_EStop   (Robot_t *robot);

/**
 * @brief  Continuous velocity jog.  Positive = CCW, Negative = CW.
 *         Runs until Robot_Stop().
 */
void Robot_JogVel  (Robot_t *robot, float speed_rad_s);

/**
 * @brief  Single incremental step jog using constrained trajectory.
 *         Positive = CCW, Negative = CW.  Returns to IDLE when done.
 */
void Robot_JogStep (Robot_t *robot, float step_rad);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper commands
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief  Pulse the elevator-up output (100 ms). */
void Robot_Gripper_MoveUp  (Robot_t *robot);

/** @brief  Pulse the elevator-down output (100 ms). */
void Robot_Gripper_MoveDown(Robot_t *robot);

/** @brief  Pulse the claw-open output (100 ms). */
void Robot_Gripper_Open    (Robot_t *robot);

/** @brief  Pulse the claw-close output (100 ms). */
void Robot_Gripper_Close   (Robot_t *robot);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper state readers
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief  Returns GRP_STATE_HIGH when elevator-up reed switch is triggered. */
GripperState_t Robot_Gripper_GetUpState  (Robot_t *robot);

/** @brief  Returns GRP_STATE_HIGH when elevator-down reed switch is triggered. */
GripperState_t Robot_Gripper_GetDownState(Robot_t *robot);

/** @brief  Returns GRP_STATE_HIGH when claw reed switch is triggered. */
GripperState_t Robot_Gripper_GetClawState(Robot_t *robot);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Callbacks
 * ═══════════════════════════════════════════════════════════════════════════ */

/** @brief  Call from HAL_TIM_PeriodElapsedCallback. */
void Robot_Update       (Robot_t *robot, TIM_HandleTypeDef *htim);

/** @brief  Call from HAL_GPIO_EXTI_Callback. */
void Robot_EXTI_Callback(Robot_t *robot, uint16_t GPIO_Pin);

/* ═══════════════════════════════════════════════════════════════════════════
 *  Getters
 * ═══════════════════════════════════════════════════════════════════════════ */
float         Robot_GetPosition   (const Robot_t *robot);
float         Robot_GetVelocity   (const Robot_t *robot);
float         Robot_GetDisturbance(const Robot_t *robot);
Robot_State_t Robot_GetState      (const Robot_t *robot);
uint8_t       Robot_IsIdle        (const Robot_t *robot);

#endif /* INC_ROBOT_H_ */
