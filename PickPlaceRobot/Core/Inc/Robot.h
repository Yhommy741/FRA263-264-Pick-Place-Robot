/*
 * Robot.h
 *
 * High-Level Robot State Machine & Dual-Rate Cascade Integration
 * (Output Shaft Space using Gear Ratio N)
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_

#include "main.h"
#include "DCMotor.h"
#include "QEI.h"
#include "KalmanFilterDCMotor.h"
#include "Controller.h"
#include "TrajectoryGen.h"
#include "MD20A.h"

/* ── High-Level States ───────────────────────────────────────────────────── */
typedef enum {
    ROBOT_STATE_IDLE,
    ROBOT_STATE_HOMING_FAST_CW,
    ROBOT_STATE_HOMING_BACKOFF_CCW,
    ROBOT_STATE_HOMING_SLOW_CW,
    ROBOT_STATE_HOMING_GOTO_OFFSET,
    ROBOT_STATE_MOVE
} Robot_State_t;

/* ── Top-Level Robot Object ──────────────────────────────────────────────── */
typedef struct {
    /* Hardware & Observer Dependencies */
    MD20A_t               *driver;
    QEI_t                 *encoder;
    KalmanFilterDCMotor_t *kalman;
    DCMotor_t             *motor;

    /* Timing & Interrupts */
    TIM_HandleTypeDef *htim;
    float             Ts;               /* Observer & Velocity Sample Time */
    uint8_t           pos_tick_counter; /* Decimation counter for position loop */

    /* Mechanics */
    float N; /* Gear Reduction Ratio (e.g., N=10 means 10 motor turns = 1 output turn) */

    /* Limit Switch Hardware */
    GPIO_TypeDef    *LS_GPIO_Port;
    uint16_t         LS_Pin;
    volatile uint8_t limit_switch_hit_flag; /* Set by EXTI */

    /* Control & Motion Planning */
    PID_t        pid_vel;
    PID_t        pid_pos;
    Trajectory_t traj;

    /* State Machine Variables */
    Robot_State_t state;
    uint32_t      state_start_tick;
    uint32_t      timeout_ms;

    /* Positions & Offsets (All in Output Shaft Space) */
    float theta_offset;       /* Software zero reference */
    float home_offset_target; /* Target offset from limit switch */

    /* Measurements (All in Output Shaft Space except tau_d) */
    float theta;
    float omega;
    float tau_d;

    /* Targets & Outputs */
    float theta_target;
    float omega_target;
    float u_prev_volts;       /* Stores previous control output for Kalman update */
    float V_max;

} Robot_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

void Robot_Init(Robot_t *robot, MD20A_t *driver, QEI_t *encoder,
                KalmanFilterDCMotor_t *kalman, DCMotor_t *motor,
                TIM_HandleTypeDef *htim, float Ts,
                GPIO_TypeDef *LS_Port, uint16_t LS_Pin,
                float N); /* Added Gear Ratio N */

void Robot_Command_Home(Robot_t *robot, float offset_rad);
void Robot_Command_Move(Robot_t *robot, float target_pos_rad, float time_s);
void Robot_Set_Home(Robot_t *robot);

/**
 * @brief Call inside HAL_TIM_PeriodElapsedCallback.
 * Updates Encoder, Kalman Filter, State Machine, and Cascade Control.
 * @param htim The timer that triggered the interrupt.
 */
void Robot_Update(Robot_t *robot, TIM_HandleTypeDef *htim);

/**
 * @brief Call inside HAL_GPIO_EXTI_Callback.
 */
void Robot_EXTI_Callback(Robot_t *robot, uint16_t GPIO_Pin);

#endif /* INC_ROBOT_H_ */
