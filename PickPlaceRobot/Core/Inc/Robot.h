/*
 * Robot.h
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  Self-contained revolute-joint robot library.
 *
 *  Responsibilities:
 *    - Owns and inits QEI, MD20A/PWM, DCMotor, Kalman, PID, FF, Trajectory
 *    - Runs dual-rate cascade control (pos @ Ts*10, vel @ Ts)
 *    - Manages homing state machine
 *
 *  NOT responsible for:
 *    - SerialFrame / telemetry  (handled by caller / App layer)
 *    - BaseSystem / Joystick    (handled by Comm layer)
 *    - High-level state machine (handled by App layer)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Usage:
 *    Robot_Init(&robot, &cfg);           // once — inits all sub-modules
 *    HAL_TIM_Base_Start_IT(&htim3);      // start ISR after init
 *
 *    // In HAL_TIM_PeriodElapsedCallback:
 *    if (htim == &htim3) Robot_Update(&robot);
 *
 *    // Commands:
 *    Robot_Move(&robot, target_rad, duration_s);
 *    Robot_Home(&robot, offset_rad);
 *    Robot_Stop(&robot);
 *    Robot_EStop(&robot);
 *
 *  Control architecture:
 *    SLOW @ Ts*POS_DIV:
 *      omega_pid = pid_pos(theta_ref, theta)
 *      omega_sum = omega_pid + omega_ref        ← trajectory FF
 *      omega_sat = clamp(omega_sum, ±omega_max) ← Velocity SAT
 *
 *    FAST @ Ts:
 *      u_pid   = pid_vel(omega_sat, omega)
 *      u_ff    = G_ff·omega_ref + G_aff·tau_d   ← Ref FF + Dist FF
 *      u_total = u_pid + u_ff
 *      u_apply = clamp(u_total, ±V_max)          ← Voltage SAT
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

/* ── Timing ratios ───────────────────────────────────────────────────────── */
#define ROBOT_POS_DIVIDER   10      /* position loop runs at Ts * 10         */

/* ── Homing velocity / distance constants ────────────────────────────────── */
#define ROBOT_HOMING_VEL_FAST_RAD   1.0f    /* rad/s — fast approach         */
#define ROBOT_HOMING_VEL_SLOW_RAD   0.2f    /* rad/s — slow creep            */
#define ROBOT_HOMING_BACKOFF_RAD   -0.5f    /* rad   — back off distance     */

/* ── Robot states ────────────────────────────────────────────────────────── */
typedef enum {
    ROBOT_IDLE,
    ROBOT_MOVE,
    ROBOT_HOMING_FAST,
    ROBOT_HOMING_BACKOFF,
    ROBOT_HOMING_SLOW,
    ROBOT_HOMING_OFFSET,
    ROBOT_ESTOP
} Robot_State_t;

/* ── Hardware + tuning config ────────────────────────────────────────────── */
typedef struct {

    /* Timers */
    TIM_HandleTypeDef *htim_encoder;   /* QEI counter timer  (e.g. TIM1)    */
    TIM_HandleTypeDef *htim_pwm;       /* MD20A PWM timer    (e.g. TIM2)    */
    TIM_HandleTypeDef *htim_ctrl;      /* Control ISR timer  (e.g. TIM3)    */

    /* MD20A channels */
    uint32_t ch_dir;   /* e.g. TIM_CHANNEL_1                                 */
    uint32_t ch_pwm;   /* e.g. TIM_CHANNEL_2                                 */

    /* Encoder */
    uint16_t enc_ppr;
    uint8_t  enc_x;
    uint32_t enc_overflow;

    /* Motor parameters */
    float Rm, Lm, Ke, Kt, J, b;

    /* Control */
    float Ts;           /* fast loop sample period [s]                       */
    float V_max;        /* supply voltage [V]                                */
    float omega_max;    /* velocity SAT limit [rad/s]  (also traj ω_max)    */
    float alpha_max;    /* acceleration limit [rad/s²] (traj constraint)    */

    /* Kalman noise variances */
    float kf_var_tau_d;
    float kf_var_theta;

    /* PID gains */
    float Kp_vel, Ki_vel, Kd_vel;
    float Kp_pos, Ki_pos, Kd_pos;

    /* Limit switch — set ls_port=NULL to disable */
    GPIO_TypeDef *ls_port;
    uint16_t      ls_pin;

} Robot_Config_t;

/* ── Robot handle — all state is here ───────────────────────────────────── */
typedef struct {

    /* Sub-modules (owned by Robot) */
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

    /* Limit switch */
    GPIO_TypeDef    *ls_port;
    uint16_t         ls_pin;
    volatile uint8_t ls_hit;

    /* State machine */
    Robot_State_t state;
    uint32_t      state_tick;
    uint32_t      timeout_ms;

    /* Measurements (motor shaft, relative to home) */
    float theta;    /* position    [rad]   */
    float omega;    /* velocity    [rad/s] */
    float tau_d;    /* disturbance [N·m]   */

    /* Motion targets */
    float theta_target;     /* IDLE hold target          */
    float omega_target;     /* velocity setpoint → vel PID */
    float home_offset;      /* raw Kalman pos at home [rad] */
    float home_goto;        /* final position after homing  */

    /* Controller internal */
    float u_prev;           /* pre-clamp voltage last tick → Kalman          */

    /* Decimation */
    uint8_t pos_tick;

} Robot_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise robot and all sub-modules.
 *         Call once before HAL_TIM_Base_Start_IT.
 */
void Robot_Init(Robot_t *robot, const Robot_Config_t *cfg);

/**
 * @brief  Command a smooth position move.
 *         Duration is computed automatically from omega_max and alpha_max
 *         set in Robot_Config_t — no manual time needed.
 */
void Robot_Move(Robot_t *robot, float target_rad);

/**
 * @brief  Run full homing sequence.
 * @param  offset_rad  Final position to move to after finding home [rad]
 */
void Robot_Home(Robot_t *robot, float offset_rad);

/**
 * @brief  Declare current position as zero.
 */
void Robot_SetHome(Robot_t *robot);

/**
 * @brief  Stop motion and hold current position.
 */
void Robot_Stop(Robot_t *robot);

/**
 * @brief  Emergency stop — disable motor immediately.
 */
void Robot_EStop(Robot_t *robot);

/**
 * @brief  Main control tick. Call inside HAL_TIM_PeriodElapsedCallback.
 *         Internally checks htim == htim_ctrl — safe to call unconditionally.
 */
void Robot_Update(Robot_t *robot, TIM_HandleTypeDef *htim);

/**
 * @brief  Limit switch callback. Call inside HAL_GPIO_EXTI_Callback.
 */
void Robot_EXTI_Callback(Robot_t *robot, uint16_t GPIO_Pin);

/* ── Getters ─────────────────────────────────────────────────────────────── */
float         Robot_GetPosition(const Robot_t *robot);  /* [rad]   */
float         Robot_GetVelocity(const Robot_t *robot);  /* [rad/s] */
float         Robot_GetDisturbance(const Robot_t *robot); /* [N·m] */
Robot_State_t Robot_GetState   (const Robot_t *robot);
uint8_t       Robot_IsIdle     (const Robot_t *robot);

#endif /* INC_ROBOT_H_ */
