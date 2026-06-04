/*
 * RobotConfig.h
 *
 * Created on: May 24, 2026
 * Author: Yhommy's Notebook
 *
 * Single source of truth for all robot hardware and tuning parameters.
 * Robot.c reads exclusively from this file — no other config is needed.
 */

#ifndef INC_ROBOTCONFIG_H_
#define INC_ROBOTCONFIG_H_

#include "main.h"   /* pulls in stm32g4xx_hal.h → TIM_HandleTypeDef, GPIO_TypeDef */
#include "tim.h"    /* extern TIM_HandleTypeDef htim1 / htim2 / htim3              */

/* ── DC Motor Parameters ─────────────────────────────────────────────────── */
#define MOTOR_RM                2.2940f     /* Ohm                          */
#define MOTOR_LM                0.0020f     /* Henry                        */
#define MOTOR_KE                0.73840f    /* V·s/rad                      */
#define MOTOR_KT                0.73840f    /* Nm/A                         */
#define MOTOR_J                 0.0067475f  /* kg·m²                        */
#define MOTOR_B                 0.0061979f  /* Nm·s/rad                     */
#define MOTOR_V_MAX             24.0f       /* V                            */

/* ── Encoder ─────────────────────────────────────────────────────────────── */
#define ENC_HTIM_PTR            (&htim1)
#define ENC_PPR                 2048
#define ENC_X                   4
#define ENC_OVERFLOW            65535

/* ── Transmission ────────────────────────────────────────────────────────── */
#define SPEED_RATIO             5.0f

/* ── Robot Kinematic Limits ──────────────────────────────────────────────── */
#define RBT_WORKSPACE_MIN       -90.0f      /* Degree                       */
#define RBT_WORKSPACE_MAX       450.0f      /* Degree                       */
#define RBT_MAX_SPEED           5.6f        /* Rad/s                        */
#define RBT_MAX_ACCEL           5.0f        /* Rad/s^2                      */
#define RBT_MAX_JERK            15.0f       /* Rad/s^3                       */


/* ── Homing Constants ────────────────────────────────────────────────────── */
#define RBT_HOMING_FAST         (-0.5f)     /* Rad/s, fast approach (CW)    */
#define RBT_HOMING_SLOW         (-0.3f)     /* Rad/s, slow creep (CW)       */
#define RBT_HOMING_BACKOFF      0.3f        /* Rad, backoff (CCW)           */
#define RBT_DEFAULT_HOMING_OFFSET 0.064577  /* Rad                        */

/* ── Kalman Filter Noise Covariances ─────────────────────────────────────── */
#define KF_VAR_TAU_D            4.98e-3f
#define KF_VAR_THETA            4.98e-9f

/* ── Control Loop Timing ─────────────────────────────────────────────────── */
#define CTRL_HTIM_PTR           (&htim3)
#define CTRL_PERIOD             0.0005f      /* Seconds (0.5 ms)               */
#define CTRL_LOOP_MULTI         10          /* Pos loop = CTRL_PERIOD × 10  */

/* ── PID Controller Gains ────────────────────────────────────────────────── */
#define KP_VEL                  1.5f
#define KI_VEL                  40.0f
#define KD_VEL                  0.0005f

#define KP_POS                  3.5f
#define KI_POS                  2.0f
#define KD_POS                  0.0f

/* ── Motor Driver MD20A ──────────────────────────────────────────────────── */
#define MDRV_HTIM_PTR           (&htim2)
#define MDRV_DIR_CH             TIM_CHANNEL_1
#define MDRV_PWM_CH             TIM_CHANNEL_2
#define MDRV_FREQ               2000.0f     /* Hz                           */

/* ── Gripper Actuators (Outputs) ─────────────────────────────────────────── */
#define GRP_UP_PORT_OUT         GPIOC
#define GRP_UP_PIN_OUT          GPIO_PIN_6

#define GRP_DOWN_PORT_OUT       GPIOC
#define GRP_DOWN_PIN_OUT        GPIO_PIN_1

#define GRP_OPEN_PORT_OUT       GPIOC
#define GRP_OPEN_PIN_OUT        GPIO_PIN_3

#define GRP_CLOSE_PORT_OUT      GPIOC
#define GRP_CLOSE_PIN_OUT       GPIO_PIN_0

/* ── Gripper Sensors (Inputs) ────────────────────────────────────────────── */
#define GRP_UP_PORT_IN          GPIOB
#define GRP_UP_PIN_IN           GPIO_PIN_0

#define GRP_DOWN_PORT_IN        GPIOB
#define GRP_DOWN_PIN_IN         GPIO_PIN_1

#define GRP_CLAW_PORT_IN        GPIOB
#define GRP_CLAW_PIN_IN         GPIO_PIN_2

/* ── Gripper Sensors Time Delay ────────────────────────────────────────────── */

#define GRP_WAIT_TIME			500
#define GRP_WAIT_PENDULUM_TIME  2500

/* ── Limit Switch ────────────────────────────────────────────────────────── */
#define LIM_SW_PORT             GPIOC
#define LIM_SW_PIN              GPIO_PIN_2

/* ── Emergency Stop Button (active-low) ──────────────────────────────────── */
#define ESTOP_PORT              GPIOC
#define ESTOP_PIN               GPIO_PIN_13

/* ── Mode Switch (active-low: LOW = Manual, HIGH = Auto) ─────────────────── */
#define MODE_SW_PORT            GPIOB
#define MODE_SW_PIN             GPIO_PIN_4

#endif /* INC_ROBOTCONFIG_H_ */
