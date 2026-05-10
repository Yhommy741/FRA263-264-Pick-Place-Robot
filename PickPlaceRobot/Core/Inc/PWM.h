/*
 * PWM.h
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"
#include "math.h"

#define ClockSpeed 170e6

typedef struct {

    TIM_HandleTypeDef* PWM_htim; 	// Timer Pointer
    uint16_t PWM_Channel; 			// Timer Channel
    uint32_t Period ; 				// The number of timer ticks in one PWM cycle
    uint16_t Prescaler ; 			// Prescaler
    uint16_t Overflow ; 			// ARR: Auto Reload Register (Counter Period)
    uint32_t OC;

} PWM_t;

// Function prototypes
void PWM_init(PWM_t* PWM, TIM_HandleTypeDef* PWM_htim, uint16_t PWM_Channel);
void PWM_write(PWM_t* PWM, float Freq, float DutyCycle);

#endif /* INC_PWM_H_ */
