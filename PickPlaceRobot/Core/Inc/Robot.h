/*
 * Robot.h
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "main.h"
#include "DCMotor.h"
#include "QEI.h"
#include "MD20A.h"
#include "KalmanFilterDCMotor.h"
#include "Controller.h"
#include "TrajectoryGen.h"

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define ROBOT_POS_DIVIDER   10

/* ── Homing constants ────────────────────────────────────────────────────── */
#define ROBOT_HOMING_VEL_FAST_RAD   (-1.0f)   /* rad/s  fast approach  CW   */
#define ROBOT_HOMING_VEL_SLOW_RAD   (-0.4f)   /* rad/s  slow creep     CW   */
#define ROBOT_HOMING_BACKOFF_RAD    ( 1.0f)   /* rad    backoff        CCW  */

/* ── States ──────────────────────────────────────────────────────────────── */
typedef enum {
    ROBOT_IDLE,
    ROBOT_MOVE,
    ROBOT_JOG_VEL,          /* continuous velocity — runs until Robot_Stop() */
    ROBOT_JOG_STEP,         /* single incremental step then → IDLE           */
    ROBOT_HOMING_FAST,
    ROBOT_HOMING_BACKOFF,
    ROBOT_HOMING_SLOW,
    ROBOT_HOMING_OFFSET,
    ROBOT_ESTOP
} Robot_State_t;

/* ── Config ──────────────────────────────────────────────────────────────── */
typedef struct {
    TIM_HandleTypeDef *htim_encoder;
    TIM_HandleTypeDef *htim_pwm;
    TIM_HandleTypeDef *htim_ctrl;

    uint32_t ch_dir;
    uint32_t ch_pwm;

    uint16_t enc_ppr;
    uint8_t  enc_x;
    uint32_t enc_overflow;

    float Rm, Lm, Ke, Kt, J, b;

    float N;            /* gear ratio: output_rad = motor_rad / N            */

    float Ts;
    float V_max;
    float omega_max;    /* [rad/s]   */
    float alpha_max;    /* [rad/s²]  */

    float kf_var_tau_d;
    float kf_var_theta;

    float Kp_vel, Ki_vel, Kd_vel;
    float Kp_pos, Ki_pos, Kd_pos;

    GPIO_TypeDef *ls_port;  /* NULL to disable limit switch */
    uint16_t      ls_pin;
} Robot_Config_t;

/* ── Handle ──────────────────────────────────────────────────────────────── */
typedef struct {
    /* Sub-modules */
    MD20A_t                 driver;
    QEI_t                   encoder;
    DCMotor_t               motor;
    KalmanFilterDCMotor_t   kalman;
    PID_t                   pid_vel;
    PID_t                   pid_pos;
    FeedforwardController_t ff;
    Trajectory_t            traj;

    /* Config */
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

    /* Controller internal */
    float   u_prev;
    uint8_t pos_tick;

    /* Jog parameters */
    float jog_speed;    /* [rad/s] — set by Robot_JogVel, sign = direction   */
    float jog_step;     /* [rad]   — set by Robot_JogStep, sign = direction  */
} Robot_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

void Robot_Init    (Robot_t *robot, const Robot_Config_t *cfg);

/** @brief  Smooth constrained move to target [rad]. */
void Robot_Move    (Robot_t *robot, float target_rad);

/** @brief  Full homing sequence. After done: theta=0, then move to position 0. */
void Robot_Home    (Robot_t *robot);

/** @brief  Declare current position as zero. */
void Robot_SetHome (Robot_t *robot);

/** @brief  Stop motion and hold current position. */
void Robot_Stop    (Robot_t *robot);

/** @brief  Emergency stop — disable motor immediately. */
void Robot_EStop   (Robot_t *robot);

/**
 * @brief  Continuous velocity jog.
 *         Positive = CCW, Negative = CW.
 *         Runs until Robot_Stop().
 */
void Robot_JogVel  (Robot_t *robot, float speed_rad_s);

/**
 * @brief  Single incremental step jog using constrained trajectory.
 *         Positive = CCW, Negative = CW.
 *         Returns to IDLE when step completes.
 */
void Robot_JogStep (Robot_t *robot, float step_rad);

/** @brief  Call from HAL_TIM_PeriodElapsedCallback. */
void Robot_Update  (Robot_t *robot, TIM_HandleTypeDef *htim);

/** @brief  Call from HAL_GPIO_EXTI_Callback. */
void Robot_EXTI_Callback(Robot_t *robot, uint16_t GPIO_Pin);

/* ── Getters ─────────────────────────────────────────────────────────────── */
float         Robot_GetPosition   (const Robot_t *robot);
float         Robot_GetVelocity   (const Robot_t *robot);
float         Robot_GetDisturbance(const Robot_t *robot);
Robot_State_t Robot_GetState      (const Robot_t *robot);
uint8_t       Robot_IsIdle        (const Robot_t *robot);

#endif /* INC_ROBOT_H_ */
