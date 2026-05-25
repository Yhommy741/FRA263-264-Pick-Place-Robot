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
#define RBT_MAX_ACCEL           5.0f        /* Rad/s²                       */

/* ── Homing Constants ────────────────────────────────────────────────────── */
#define RBT_HOMING_FAST         (-0.4f)     /* Rad/s, fast approach (CW)    */
#define RBT_HOMING_SLOW         (-0.2f)     /* Rad/s, slow creep (CW)       */
#define RBT_HOMING_BACKOFF      0.2f        /* Rad, backoff (CCW)           */
#define RBT_DEFAULT_HOMING_OFFSET 0.05236f  /* Rad                        */

/* ── Kalman Filter Noise Covariances ─────────────────────────────────────── */
#define KF_VAR_TAU_D            4.98e-6f
#define KF_VAR_THETA            4.98e-9f

/* ── Control Loop Timing ─────────────────────────────────────────────────── */
#define CTRL_HTIM_PTR           (&htim3)
#define CTRL_PERIOD             0.001f      /* Seconds (1 ms)               */
#define CTRL_LOOP_MULTI         10          /* Pos loop = CTRL_PERIOD × 10  */

/* ── PID Controller Gains ────────────────────────────────────────────────── */
#define KP_VEL                  20.0f
#define KI_VEL                  175.0f
#define KD_VEL                  0.0f

#define KP_POS                  60.0f
#define KI_POS                  0.0f
#define KD_POS                  0.5f

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

/* ── Limit Switch ────────────────────────────────────────────────────────── */
#define LIM_SW_PORT             GPIOC
#define LIM_SW_PIN              GPIO_PIN_2

#endif /* INC_ROBOTCONFIG_H_ */
