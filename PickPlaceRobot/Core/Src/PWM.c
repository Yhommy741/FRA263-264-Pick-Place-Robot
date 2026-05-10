/*
 * PWM.c
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 */

#include "PWM.h"

void PWM_init(PWM_t* PWM, TIM_HandleTypeDef* PWM_htim, uint16_t PWM_Channel){
    PWM->PWM_htim = PWM_htim;
    PWM->PWM_Channel = PWM_Channel;
    PWM->OC = 0;
    if (PWM_htim->State == HAL_TIM_STATE_READY) {
        HAL_TIM_Base_Start(PWM_htim);
    }
    HAL_TIM_PWM_Start(PWM_htim, PWM_Channel);
}

void PWM_write(PWM_t* PWM, float Freq, float DutyCycle){
    if (Freq <= 0.0f || Freq > ClockSpeed) {
        __HAL_TIM_SET_COMPARE(PWM->PWM_htim, PWM->PWM_Channel, 0);
    } else {
        PWM->Period = (uint32_t) (ClockSpeed / Freq);
        PWM->Prescaler = (uint16_t) ((PWM->Period + 65535 - 1) / 65535) - 1;
        PWM->Overflow = (uint16_t) ((ClockSpeed / (float)(PWM->Prescaler + 1) / Freq) - 1);
        PWM->OC = (uint16_t) (PWM->Overflow * fabsf(DutyCycle) / 100.0f);

        __HAL_TIM_SET_PRESCALER(PWM->PWM_htim, PWM->Prescaler);
        __HAL_TIM_SET_AUTORELOAD(PWM->PWM_htim, PWM->Overflow);
        __HAL_TIM_SET_COMPARE(PWM->PWM_htim, PWM->PWM_Channel, PWM->OC);
        __HAL_TIM_SET_COUNTER(PWM->PWM_htim, 0);
    }
}
