/*
 * Controller.c
 *
 *  Created: May 2026
 *  Author : FRA263/264 Group 5
 */

#include "Controller.h"
#include "TrajectoryGen.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Velocity-Form (Incremental) PID
 * ═══════════════════════════════════════════════════════════════════════════ */

void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float Ts,
              float out_min, float out_max)
{
    pid->Kp      = Kp;
    pid->Ki      = Ki;
    pid->Kd      = Kd;
    pid->Ts      = Ts;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->e[0]    = 0.0f;
    pid->e[1]    = 0.0f;
    pid->e[2]    = 0.0f;
    pid->u_prev  = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measured)
{
    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = setpoint - measured;

    float delta_u = pid->Kp * (pid->e[0] - pid->e[1])
                  + pid->Ki *  pid->Ts * pid->e[0]
                  + (pid->Kd / pid->Ts) * (pid->e[0] - 2.0f*pid->e[1] + pid->e[2]);

    float u = Saturate(pid->u_prev + delta_u, pid->out_min, pid->out_max);
    pid->u_prev = u;
    return u;
}

void PID_Reset(PID_t *pid)
{
    pid->e[0]   = 0.0f;
    pid->e[1]   = 0.0f;
    pid->e[2]   = 0.0f;
    pid->u_prev = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  FeedforwardController
 *
 *  Reference FF (unambiguous):
 *    G_ff = Rm·b/Kt + Ke
 *    u_ref_ff = +G_ff · omega_ref
 *
 *  Disturbance FF (sign requires empirical verification):
 *    G_aff = Rm/Kt
 *    u_dist_ff = FF_DIST_SIGN · G_aff · tau_d
 *    Disabled by default (FF_DIST_ENABLE = 0 in Controller.h)
 *
 *  To determine FF_DIST_SIGN empirically:
 *    1. Add KalmanDisturbance to TX frame temporarily
 *    2. Run motor at constant speed with a known braking load
 *    3. Read sign of tau_d:
 *       tau_d > 0 under braking → use FF_DIST_SIGN = +1.0f
 *       tau_d < 0 under braking → use FF_DIST_SIGN = -1.0f
 *    4. Enable with FF_DIST_ENABLE = 1
 * ═══════════════════════════════════════════════════════════════════════════ */

void FF_Init(FeedforwardController_t *ff, const DCMotor_t *motor)
{
    ff->G_ff  = (motor->Rm * motor->b / motor->Kt) + motor->Ke;
    ff->G_aff = motor->Rm / motor->Kt;
}

float FF_Compute(const FeedforwardController_t *ff,
                 float omega_ref,
                 float tau_d)
{
    float u = ff->G_ff * omega_ref;

#if FF_DIST_ENABLE
    u += FF_DIST_SIGN * ff->G_aff * tau_d;
#else
    (void)tau_d;   /* suppress unused warning when disabled */
#endif

    return u;
}
