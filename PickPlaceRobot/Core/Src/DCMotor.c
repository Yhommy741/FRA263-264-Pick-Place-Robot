/*
 * DCMotor.c
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * DC motor parameter struct initialiser.
 * Single source of truth consumed by KalmanFilterDCMotor and Robot.
 */

#include "DCMotor.h"

void DCMotor_Init(DCMotor_t *motor,
                  float Rm, float Lm,
                  float Ke, float Kt,
                  float J,  float b)
{
    motor->Rm = Rm;
    motor->Lm = Lm;
    motor->Ke = Ke;
    motor->Kt = Kt;
    motor->J  = J;
    motor->b  = b;
}
