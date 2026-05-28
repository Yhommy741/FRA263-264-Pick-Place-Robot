/*
 * PWM.c
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 *
 * Fix (May 2026):
 *   ROOT CAUSE — __HAL_TIM_SET_COUNTER() was called at the end of every
 *   PWM_write().  When two channels share one timer (MD20A: DIR on CH1,
 *   PWM on CH2, both on TIM2), writing DIR resets the counter to 0, then
 *   writing PWM resets it again immediately.  The counter therefore never
 *   advances and no pulse is generated on either pin.
 *
 *   FIX — Remove __HAL_TIM_SET_COUNTER().
 *   The ARR shadow register (TIM_AUTORELOAD_PRELOAD_ENABLE in MX_TIM2_Init)
 *   already guarantees a clean period update at the next overflow without
 *   any manual counter reset.
 *
 *   ADDITION — PWM_write_duty() updates only CCR (compare register).
 *   Use it for the fast path (same frequency, only duty changes) to avoid
 *   touching PSC/ARR and causing a glitch on the other channel.
 */

#include "PWM.h"

/* ─────────────────────────────────────────────────────────────────────────
 * PWM_init
 * ───────────────────────────────────────────────────────────────────────── */
void PWM_init(PWM_t *PWM,
              TIM_HandleTypeDef *PWM_htim,
              uint16_t PWM_Channel)
{
    PWM->PWM_htim    = PWM_htim;
    PWM->PWM_Channel = PWM_Channel;
    PWM->OC          = 0;
    PWM->Prescaler   = 0;
    PWM->Overflow    = 0;
    PWM->Period      = 0;

    /* Start the timer base only once — guard prevents double-start when
       two channels (DIR + PWM) share the same timer handle.             */
    if (PWM_htim->State == HAL_TIM_STATE_READY) {
        HAL_TIM_Base_Start(PWM_htim);
    }

    HAL_TIM_PWM_Start(PWM_htim, PWM_Channel);
}

/* ─────────────────────────────────────────────────────────────────────────
 * PWM_write  —  set frequency AND duty cycle
 *
 * Computes PSC and ARR from Freq, then sets CCR from DutyCycle.
 * Does NOT reset the counter (that was the bug).
 * ───────────────────────────────────────────────────────────────────────── */
void PWM_write(PWM_t *PWM, float Freq, float DutyCycle)
{
    if (Freq <= 0.0f) {
        /* Invalid frequency — force output low */
        __HAL_TIM_SET_COMPARE(PWM->PWM_htim, PWM->PWM_Channel, 0);
        return;
    }

    /* ── 1. Compute timer parameters ──────────────────────────────────── */
    PWM->Period    = (uint32_t)(ClockSpeed / Freq);

    /* Prescaler: smallest value so that ARR fits in 16 bits (≤ 65535).
       Formula: PSC = ceil(Period / 65536) - 1                           */
    PWM->Prescaler = (uint16_t)(((PWM->Period + 65535U) / 65536U) - 1U);

    /* ARR: actual counter top after applying the prescaler              */
    PWM->Overflow  = (uint16_t)((ClockSpeed /
                                 ((float)(PWM->Prescaler + 1U) * Freq)) - 1.0f);

    /* Compare value: ARR fraction corresponding to |DutyCycle| %        */
    PWM->OC = (uint32_t)((float)PWM->Overflow * fabsf(DutyCycle) / 100.0f);

    /* ── 2. Apply to timer registers ─────────────────────────────────── */
    __HAL_TIM_SET_PRESCALER  (PWM->PWM_htim, PWM->Prescaler);
    __HAL_TIM_SET_AUTORELOAD (PWM->PWM_htim, PWM->Overflow);
    __HAL_TIM_SET_COMPARE    (PWM->PWM_htim, PWM->PWM_Channel, PWM->OC);

    /* ── NO __HAL_TIM_SET_COUNTER() here ─────────────────────────────── */
    /* Resetting the counter prevented PWM output when two channels share
       a timer: the second write would zero the counter before it could
       count, starving both outputs of pulses.  The ARR preload shadow
       register (enabled in MX_TIM2_Init) handles clean period updates
       automatically at the next overflow.                               */
}

/* ─────────────────────────────────────────────────────────────────────────
 * PWM_write_duty  —  update duty cycle only (frequency already set)
 *
 * Only writes CCR.  Safe to call on one channel without affecting the
 * PSC/ARR shared with the other channel on the same timer.
 * ───────────────────────────────────────────────────────────────────────── */
void PWM_write_duty(PWM_t *PWM, float DutyCycle)
{
    if (PWM->Overflow == 0U) {
        /* PWM_write() has not been called yet — nothing to do */
        return;
    }

    PWM->OC = (uint32_t)((float)PWM->Overflow * fabsf(DutyCycle) / 100.0f);
    __HAL_TIM_SET_COMPARE(PWM->PWM_htim, PWM->PWM_Channel, PWM->OC);
}
