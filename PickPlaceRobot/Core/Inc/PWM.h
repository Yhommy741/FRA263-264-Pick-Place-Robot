/*
 * PWM.h
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 *
 * General-purpose PWM driver interface.
 * Configures timer ARR for frequency and CCR for duty cycle.
 * ClockSpeed = 170 MHz (STM32G474).
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"
#include "math.h"

/* ── System clock frequency ─────────────────────────────────────────────── */
#define ClockSpeed  170000000.0f    /* STM32G4 @ 170 MHz                    */

/* ── PWM handle ─────────────────────────────────────────────────────────── */
typedef struct {

    TIM_HandleTypeDef *PWM_htim;    /* Timer handle pointer                 */
    uint16_t           PWM_Channel; /* TIM_CHANNEL_1 … TIM_CHANNEL_4        */
    uint32_t           Period;      /* Ticks per PWM cycle                  */
    uint16_t           Prescaler;   /* PSC register value                   */
    uint16_t           Overflow;    /* ARR register value                   */
    uint32_t           OC;          /* Compare register value               */

} PWM_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise a PWM channel and start it.
 * @param  PWM          Pointer to PWM_t instance.
 * @param  PWM_htim     Timer handle.
 * @param  PWM_Channel  TIM_CHANNEL_x.
 */
void PWM_init(PWM_t *PWM,
              TIM_HandleTypeDef *PWM_htim,
              uint16_t PWM_Channel);

/**
 * @brief  Set PWM frequency and duty cycle.
 *         Reprograms PSC, ARR, and CCR.
 *         Safe to call from multiple channels on the same timer as long as
 *         the same Freq is used for all channels (shared PSC/ARR).
 *
 * @param  PWM        Pointer to PWM_t instance.
 * @param  Freq       Carrier frequency [Hz].  Must be > 0.
 * @param  DutyCycle  Duty cycle [0.0 – 100.0] %.  Sign is ignored (use fabsf).
 */
void PWM_write(PWM_t *PWM, float Freq, float DutyCycle);

/**
 * @brief  Update duty cycle only — does NOT touch PSC or ARR.
 *         Use this when the frequency is already set and you only want to
 *         change the duty cycle of one channel without disturbing the other.
 *
 * @param  PWM        Pointer to PWM_t instance.
 * @param  DutyCycle  Duty cycle [0.0 – 100.0] %.
 */
void PWM_write_duty(PWM_t *PWM, float DutyCycle);

#endif /* INC_PWM_H_ */
