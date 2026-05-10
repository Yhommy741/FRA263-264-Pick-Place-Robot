/*
 * MD20A.h
 *
 *  Created on: May 5, 2026
 *      Author: Yhommy's Notebook
 *
 *  MD20A Motor Driver Library
 *  ─────────────────────────────────────────────────────────────────────────
 *  The MD20A uses a DIR + PWM control scheme.
 *  Both pins are driven by PWM channels on the same timer:
 *
 *      DIR (PA0 / TIM_CHANNEL_1):
 *          Duty = 100 % → logic HIGH → Forward
 *          Duty =   0 % → logic LOW  → Backward
 *
 *      PWM (PA1 / TIM_CHANNEL_2):
 *          Duty = |speed| % → motor speed
 *
 *  Truth table:
 *      DIR duty    PWM duty    Motor
 *      ─────────   ─────────   ──────────────
 *      100 %       X %         Forward  at X %
 *        0 %       X %         Backward at X %
 *      100 %       0 %         Coast (stop)
 *
 *  Hardware connections:
 *      DIR → PA0  (TIM2 CH1)
 *      PWM → PA1  (TIM2 CH2)
 */

#ifndef INC_MD20A_H_
#define INC_MD20A_H_

#include "main.h"
#include "PWM.h"
#include <stdint.h>
#include <math.h>

/* ── Default PWM carrier frequency ─────────────────────────────────────── */
#define MD20A_DEFAULT_FREQ  2000.0f    /* 2 kHz                              */

/* ── Dead-band — speeds below this coast ───────────────────────────────── */
#define MD20A_DEADBAND      0.5f       /* %                                  */

/* ── Motor driver struct ────────────────────────────────────────────────── */
typedef struct {

    PWM_t   DIR;           /* PA0 — direction channel (0 % or 100 %)        */
    PWM_t   PWM;           /* PA1 — speed channel (0 – 100 %)               */

    float   Frequency;     /* PWM carrier frequency (Hz)                    */
    float   DutyCycle;     /* Last commanded |duty cycle| [0.0 – 100.0]     */
    int8_t  Direction;     /* +1 forward  ·  -1 backward  ·  0 stop         */

} MD20A_t;

/* ── Function prototypes ─────────────────────────────────────────────────── */

/**
 * @brief  Initialise the MD20A driver.
 * @param  motor    Pointer to MD20A_t instance.
 * @param  htim     Shared timer handle for DIR and PWM channels.
 * @param  CH_DIR   Timer channel for DIR pin  (e.g. TIM_CHANNEL_1 → PA0).
 * @param  CH_PWM   Timer channel for PWM pin  (e.g. TIM_CHANNEL_2 → PA1).
 */
void MD20A_init(MD20A_t *motor,
                TIM_HandleTypeDef *htim,
                uint16_t CH_DIR,
                uint16_t CH_PWM);

/**
 * @brief  Drive motor at a signed speed.
 * @param  motor   Pointer to MD20A_t instance.
 * @param  speed   Signed speed [-100.0 … +100.0] %.
 *                   > 0  → DIR = 100 %, PWM = speed %   (forward)
 *                   < 0  → DIR =   0 %, PWM = |speed| % (backward)
 *                   = 0  → coast
 */
void MD20A_setSpeed(MD20A_t *motor, float speed);

/**
 * @brief  Coast stop — PWM = 0 %.
 */
void MD20A_stop(MD20A_t *motor);

/**
 * @brief  Active brake — PWM = 100 %, DIR unchanged.
 */
void MD20A_brake(MD20A_t *motor);

#endif /* INC_MD20A_H_ */
