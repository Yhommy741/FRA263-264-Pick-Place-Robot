/*
 * DCMotor.c
 *
 *  Created: May 2026
 *  Author : FRA263/264 Group 5
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
