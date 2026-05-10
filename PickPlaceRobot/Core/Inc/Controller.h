/*
 * Controller.h
 *
 *  Created: May 2026
 *  Author : FRA263/264 Group 5
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  Control primitives
 *
 *  1. PID_t                 — Velocity-form (incremental) PID
 *
 *  2. FeedforwardController_t
 *
 *     a) Reference Feedforward  — mathematically unambiguous ✓
 *          At SS (dω/dt=0, dI/dt=0, no disturbance):
 *            I = (b/Kt)·ω*
 *            V = Rm·I + Ke·ω* = (Rm·b/Kt + Ke)·ω*
 *          G_ff = Rm·b/Kt + Ke
 *          u_ref_ff = +G_ff · omega_ref
 *
 *     b) Disturbance Feedforward — sign depends on Kalman convention
 *          Kalman Fd row 1: dω/dt = ... + (1/J)·x[3]
 *          DCMotor.h model: dω/dt = ... − (1/J)·tau_load
 *
 *          Two valid interpretations:
 *            (A) x[3] = −tau_load  →  u_dist_ff = −(Rm/Kt)·x[3]  (MINUS)
 *            (B) x[3] = +tau_load* →  u_dist_ff = +(Rm/Kt)·x[3]  (PLUS, Robot.c)
 *                (* Kalman compensates sign internally during estimation)
 *
 *          EMPIRICAL DETERMINATION REQUIRED — see FF_DIST_SIGN below.
 *          Set FF_DIST_SIGN = +1.0f or −1.0f based on which gives better results.
 *          Set FF_DIST_ENABLE = 0 to disable disturbance FF entirely (safe default).
 *
 *  3. Saturate() — inline clamp
 * ═══════════════════════════════════════════════════════════════════════════
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>
#include <math.h>
#include "DCMotor.h"

/* ════════════════════════════════════════════════════════════════════════════
 *  Disturbance FF configuration
 *  Set FF_DIST_ENABLE = 1 only after verifying sign empirically:
 *    Apply known braking load → check sign of KalmanDisturbance in TX →
 *    if braking gives tau_d > 0: use FF_DIST_SIGN = +1.0f (Robot.c convention)
 *    if braking gives tau_d < 0: use FF_DIST_SIGN = -1.0f
 * ════════════════════════════════════════════════════════════════════════════ */
#define FF_DIST_ENABLE  0        /* 0 = off (safe), 1 = on                  */
#define FF_DIST_SIGN    1.0f     /* +1.0f or -1.0f — verify empirically      */

/* ════════════════════════════════════════════════════════════════════════════
 *  Velocity-Form (Incremental) PID
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float Kp, Ki, Kd;
    float Ts;
    float out_min, out_max;
    float e[3];
    float u_prev;
} PID_t;

void  PID_Init  (PID_t *pid, float Kp, float Ki, float Kd,
                 float Ts, float out_min, float out_max);
float PID_Update(PID_t *pid, float setpoint, float measured);
void  PID_Reset (PID_t *pid);

/* ════════════════════════════════════════════════════════════════════════════
 *  FeedforwardController_t
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float G_ff;    /* Reference FF gain    : Rm·b/Kt + Ke  [V·s/rad] */
    float G_aff;   /* Disturbance FF gain  : Rm/Kt         [V/N·m]   */
} FeedforwardController_t;

void  FF_Init   (FeedforwardController_t *ff, const DCMotor_t *motor);

/**
 * @brief Compute feedforward voltage.
 *   u_ff = +G_ff * omega_ref
 *        + FF_DIST_SIGN * G_aff * tau_d  (only if FF_DIST_ENABLE=1)
 *
 * @param omega_ref  Velocity reference [rad/s]
 * @param tau_d      Kalman state x[3] — disturbance estimate [N·m]
 * @return Feedforward voltage [V]
 */
float FF_Compute(const FeedforwardController_t *ff,
                 float omega_ref,
                 float tau_d);

#include "TrajectoryGen.h"

static inline float Saturate(float val, float min_val, float max_val)
{
    if (val >  max_val) return  max_val;
    if (val <  min_val) return  min_val;
    return val;
}

#endif /* INC_CONTROLLER_H_ */
