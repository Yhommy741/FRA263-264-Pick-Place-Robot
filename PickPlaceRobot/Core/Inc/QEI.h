/*
 * QEI.h
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy / Updated by Gemini
 */

#ifndef QEI_H_
#define QEI_H_

#include "main.h"
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef enum {
    Prev = 0,
    Curr = 1
} QEI_State_t;

typedef struct {
    // Hardware Handles
    TIM_HandleTypeDef *QEI_htim;
    TIM_HandleTypeDef *Observer_htim;

    // Encoder Hardware Specs
    uint16_t PPR;
    uint16_t X;
    uint32_t CPR;
    uint32_t OverflowCount;

    // Timing
    float ObserverPeriod;

    // Raw Counters
    uint32_t Count[2];
    int32_t deltaCount;
    int64_t totalCount;

    // Absolute Position
    double Revs;
    double Rad;
    double Degree;

    // Raw Velocity
    float RPM;
    float Radps;

} QEI_t;

// Function Prototypes (Filter parameter removed)
void QEI_init(QEI_t *QEI, TIM_HandleTypeDef *QEI_htim, TIM_HandleTypeDef *Observer_htim,
              uint16_t PPR, uint16_t X, uint32_t OverflowCount, float ObserverPeriod);

void QEI_update(QEI_t *QEI, TIM_HandleTypeDef *Timer);

#endif /* QEI_H_ */
