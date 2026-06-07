/*
 * QEI.c
 *
 * Created on: Apr 11, 2026
 * Author: Yhommy
 *
 * Quadrature encoder interface driver.
 * Supports ×1, ×2, ×4 resolution modes.
 * Computes angular position (rad) and velocity (rad/s) each tick.
 */

#include "QEI.h"

void QEI_init(QEI_t *QEI, TIM_HandleTypeDef *QEI_htim, TIM_HandleTypeDef *Observer_htim,
              uint16_t PPR, uint16_t X, uint32_t OverflowCount, float ObserverPeriod) {

    QEI->QEI_htim = QEI_htim;
    QEI->Observer_htim = Observer_htim;

    QEI->PPR = PPR;
    QEI->X = X;
    QEI->CPR = (uint32_t)QEI->PPR * QEI->X;

    QEI->OverflowCount = OverflowCount;
    QEI->ObserverPeriod = ObserverPeriod;

    QEI->Count[Curr] = 0;
    QEI->Count[Prev] = 0;
    QEI->deltaCount = 0;
    QEI->totalCount = 0;

    QEI->Rad = 0.0;
    QEI->Degree = 0.0;
    QEI->Revs = 0.0;

    QEI->Radps = 0.0f;
    QEI->RPM = 0.0f;

    // Start the Encoder timer
    HAL_TIM_Encoder_Start(QEI_htim, TIM_CHANNEL_ALL);

    // Capture initial starting count
    QEI->Count[Prev] = __HAL_TIM_GET_COUNTER(QEI_htim);
}

void QEI_update(QEI_t *QEI, TIM_HandleTypeDef *Timer) {

    if (Timer == QEI->Observer_htim) {

        // 1. Read Current Count
        QEI->Count[Curr] = __HAL_TIM_GET_COUNTER(QEI->QEI_htim);

        // 2. Calculate Delta Count with explicit signed casting
        int32_t diff = (int32_t)(QEI->Count[Curr] - QEI->Count[Prev]);

        // 3. Anti-Wrap-Around Logic
        if (diff > (int32_t)(QEI->OverflowCount >> 1)) {
            diff -= (QEI->OverflowCount + 1);
        } else if (diff < -(int32_t)(QEI->OverflowCount >> 1)) {
            diff += (QEI->OverflowCount + 1);
        }

        QEI->deltaCount = diff;

        // 4. Update Absolute 64-bit count
        QEI->totalCount += QEI->deltaCount;

        // 5. Position Unit Conversion
        QEI->Revs   = (double)QEI->totalCount / (double)QEI->CPR;
        QEI->Rad    = QEI->Revs * 2.0 * (double)M_PI;
        QEI->Degree = QEI->Revs * 360.0;

        // 6. Raw Velocity Unit Conversion
        float delta_revs = (float)QEI->deltaCount / (float)QEI->CPR;
        QEI->RPM   = (delta_revs / QEI->ObserverPeriod) * 60.0f;
        QEI->Radps = (delta_revs * 2.0f * (float)M_PI) / QEI->ObserverPeriod;

        // 7. Keep Prev Count for next iteration
        QEI->Count[Prev] = QEI->Count[Curr];
    }
}
