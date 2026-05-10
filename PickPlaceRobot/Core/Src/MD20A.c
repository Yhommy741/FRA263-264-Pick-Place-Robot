/*
 * MD20A.c
 *
 *  Created on: May 5, 2026
 *      Author: Yhommy's Notebook
 *
 *  MD20A Motor Driver — DIR + PWM Implementation
 *
 *  DIR pin (PA0 / TIM2 CH1) is driven by PWM at the same frequency as the
 *  speed channel but at a fixed duty cycle: 100 % = forward, 0 % = backward.
 *  This avoids any GPIO vs PWM pin conflict and keeps both outputs
 *  synchronised on the same timer.
 */

#include "MD20A.h"

/* ─────────────────────────────────────────────────────────────────────────
 * MD20A_init
 * ───────────────────────────────────────────────────────────────────────── */
void MD20A_init(MD20A_t *motor,
                TIM_HandleTypeDef *htim,
                uint16_t CH_DIR,
                uint16_t CH_PWM)
{
    motor->Frequency = MD20A_DEFAULT_FREQ;
    motor->DutyCycle = 0.0f;
    motor->Direction = 0.0;

    /* Initialise DIR channel first — this starts the timer base.
     * PWM channel init will skip HAL_TIM_Base_Start (guarded in PWM_init). */
    PWM_init(&motor->DIR, htim, CH_DIR);
    PWM_init(&motor->PWM, htim, CH_PWM);

    /* Coast at startup — DIR = forward, PWM = 0 % */
    PWM_write(&motor->DIR, motor->Frequency, 100.0f);
    PWM_write(&motor->PWM, motor->Frequency,   0.0f);
}

/* ─────────────────────────────────────────────────────────────────────────
 * MD20A_setSpeed
 *   speed > 0  → DIR = 100 % (forward),  PWM = speed %
 *   speed < 0  → DIR =   0 % (backward), PWM = |speed| %
 *   |speed| < deadband → coast
 * ───────────────────────────────────────────────────────────────────────── */
void MD20A_setSpeed(MD20A_t *motor, float speed)
{
    /* Clamp */
    if (speed >  100.0f) speed =  100.0f;
    if (speed < -100.0f) speed = -100.0f;

    /* Dead-band → coast */
    if (fabsf(speed) < MD20A_DEADBAND) {
        MD20A_stop(motor);
        return;
    }

    motor->DutyCycle = fabsf(speed);

    if (speed > 0.0f) {
        /* Forward: DIR = HIGH (100 %), then apply speed */
        motor->Direction = +1;
        PWM_write(&motor->DIR, motor->Frequency, 100.0f);
        PWM_write(&motor->PWM, motor->Frequency, motor->DutyCycle);
    } else {
        /* Backward: DIR = LOW (0 %), then apply speed */
        motor->Direction = -1;
        PWM_write(&motor->DIR, motor->Frequency,   0.0f);
        PWM_write(&motor->PWM, motor->Frequency, motor->DutyCycle);
    }
}

/* ─────────────────────────────────────────────────────────────────────────
 * MD20A_stop — coast (PWM = 0 %, DIR stays as-is)
 * ───────────────────────────────────────────────────────────────────────── */
void MD20A_stop(MD20A_t *motor)
{
    motor->DutyCycle = 0.0f;
    motor->Direction = 0;
    PWM_write(&motor->PWM, motor->Frequency, 0.0f);
}

/* ─────────────────────────────────────────────────────────────────────────
 * MD20A_brake — active brake (PWM = 100 %)
 * ───────────────────────────────────────────────────────────────────────── */
void MD20A_brake(MD20A_t *motor)
{
    motor->DutyCycle = 100.0f;
    PWM_write(&motor->PWM, motor->Frequency, 100.0f);
}
