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
#include "KalmanFilterDCMotor.h"
#include "Controller.h"
#include "SCurve.h"        /* 7-segment S-curve — replaces TrajectoryGen.h    */
#include "Trapezoid.h"     /* 3-phase trapezoidal — performance test          */
#include "CANBus.h"        /* CAN protocol driver (v1.0.1)                    */
#include "Gripper.h"

/* ── States ──────────────────────────────────────────────────────────────── */
typedef enum {
    ROBOT_IDLE,
    ROBOT_MOVE,
    ROBOT_JOG_VEL,
    ROBOT_JOG_STEP,
    ROBOT_HOMING_FAST_STATE,
    ROBOT_HOMING_BACKOFF_STATE,
    ROBOT_HOMING_SLOW_STATE,
    ROBOT_HOMING_OFFSET_STATE,
    ROBOT_PERF_TEST,
    ROBOT_ESTOP
} Robot_State_t;

/* ── Handle ──────────────────────────────────────────────────────────────── */
typedef struct {
    MD20A_t                 driver;
    QEI_t                   encoder;
    DCMotor_t               motor;
    KalmanFilterDCMotor_t   kalman;
    Controller_t            ctr;
    SCurve_t                scurve;   /* normal moves          */
    Trapezoid_t             trap;     /* performance test      */
    CANBus_t                can_bus;  /* CAN driver (used when GRP_MODE==CANBUS) */
    uint8_t                 can_node_init_done; /* 1 = initial relay state sent  */
    Gripper_t               gripper;

    TIM_HandleTypeDef *htim_ctrl;
    float  Ts;
    float  V_max;
    float  omega_max;
    float  N;

    GPIO_TypeDef    *ls_port;
    uint16_t         ls_pin;
    volatile uint8_t ls_hit;

    Robot_State_t state;
    uint32_t      state_tick;
    uint32_t      timeout_ms;

    float theta;
    float omega;
    float omega_prev;
    float alpha;
    float tau_d;

    float theta_target;
    float omega_target;
    float home_offset;
    float home_goto;

    float u_prev;
    uint8_t pos_tick;

    float jog_speed;
    float jog_step;
} Robot_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  API
 * ═══════════════════════════════════════════════════════════════════════════ */

void Robot_Init(Robot_t *robot);

/** @brief  S-curve move to target [rad] — shortest-path wrap. */
void Robot_Move(Robot_t *robot, float target_rad);

/** @brief  S-curve move to absolute target, no wrap, caller-supplied limits. */
void Robot_MoveConstrained(Robot_t *robot, float target_rad,
                            float omega_max, float alpha_max);

/**
 * @brief  Trapezoidal performance test leg.
 *         Guarantees v_cruise = v_target (boosts a if needed).
 *         a_used >= a_target always.
 */
void Robot_PerfTest_Start(Robot_t *robot, float target_rad,
                           float v_target, float a_target);

/**
 * @brief  Soft reset — returns robot to default IDLE state without
 *         re-initialising hardware peripherals or losing home position.
 *         Resets: PID integrals, trajectories, Kalman, all motion state.
 */
void Robot_SoftReset(Robot_t *robot);

void Robot_Home   (Robot_t *robot);
void Robot_SetHome(Robot_t *robot, float new_home_rad);
void Robot_Stop   (Robot_t *robot);
void Robot_EStop  (Robot_t *robot);
void Robot_JogVel (Robot_t *robot, float speed_rad_s);
void Robot_JogStep(Robot_t *robot, float step_rad);

void Robot_Gripper_MoveUp  (Robot_t *robot);
void Robot_Gripper_MoveDown(Robot_t *robot);
void Robot_Gripper_Open    (Robot_t *robot);
void Robot_Gripper_Close   (Robot_t *robot);

GripperState_t Robot_Gripper_GetUpState  (Robot_t *robot);
GripperState_t Robot_Gripper_GetDownState(Robot_t *robot);
GripperState_t Robot_Gripper_GetClawState(Robot_t *robot);

void Robot_Update       (Robot_t *robot, TIM_HandleTypeDef *htim);
void Robot_CANBus_Update(Robot_t *robot);   /* Call from main loop — NOT from ISR */
void Robot_EXTI_Callback(Robot_t *robot, uint16_t GPIO_Pin);

float         Robot_GetPosition    (const Robot_t *robot);
float         Robot_GetVelocity    (const Robot_t *robot);
float         Robot_GetAcceleration(const Robot_t *robot);
float         Robot_GetDisturbance (const Robot_t *robot);
Robot_State_t Robot_GetState       (const Robot_t *robot);
uint8_t       Robot_IsIdle         (const Robot_t *robot);

#endif /* INC_ROBOT_H_ */
