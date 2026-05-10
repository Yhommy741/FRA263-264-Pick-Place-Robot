/*
 * Controller.h
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 *
 *  Control primitives used by Robot.
 *
 *  1. PID_t  — velocity-form (incremental) PID
 *       Δu[k] = Kp·(e[k]−e[k−1]) + Ki·Ts·e[k] + (Kd/Ts)·(e[k]−2e[k−1]+e[k−2])
 *       u[k]  = u[k−1] + Δu[k],  clamped to [out_min, out_max]
 *
 *  2. FeedforwardController_t
 *       a) Reference FF  (Section 2.4.2.3)
 *            G_ff  = Rm·b/Kt + Ke
 *            u_ff  = +G_ff · ω*
 *       b) Disturbance FF  (Section 2.4.2.2)
 *            G_aff = Rm/Kt
 *            u_aff = +G_aff · τ_d      (sign confirmed by Robot.c convention)
 *       Combined: FF_Compute = G_ff·ω* + G_aff·τ_d
 *
 *  3. Saturate() — inline symmetric clamp
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>
#include "DCMotor.h"

/* ── PID ─────────────────────────────────────────────────────────────────── */
typedef struct {
    float Kp, Ki, Kd;
    float Ts;
    float out_min, out_max;
    float e[3];      /* e[0]=e[k], e[1]=e[k-1], e[2]=e[k-2] */
    float u_prev;
} PID_t;

void  PID_Init  (PID_t *pid,
                 float Kp, float Ki, float Kd,
                 float Ts, float out_min, float out_max);
float PID_Update(PID_t *pid, float setpoint, float measured);
void  PID_Reset (PID_t *pid);

/* ── Feedforward ─────────────────────────────────────────────────────────── */
typedef struct {
    float G_ff;    /* Rm·b/Kt + Ke   [V·s/rad] */
    float G_aff;   /* Rm/Kt          [V/N·m]   */
} FeedforwardController_t;

void  FF_Init   (FeedforwardController_t *ff, const DCMotor_t *motor);
float FF_Compute(const FeedforwardController_t *ff, float omega_ref, float tau_d);

/* ── Saturate ─────────────────────────────────────────────────────────────── */
static inline float Saturate(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

#endif /* INC_CONTROLLER_H_ */
