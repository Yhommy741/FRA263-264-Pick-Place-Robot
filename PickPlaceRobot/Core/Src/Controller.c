/*
 * Controller.c
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 */

#include "Controller.h"

/* ── PID ─────────────────────────────────────────────────────────────────── */

void PID_Init(PID_t *pid,
              float Kp, float Ki, float Kd,
              float Ts, float out_min, float out_max)
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

    float du = pid->Kp * (pid->e[0] - pid->e[1])
             + pid->Ki *  pid->Ts * pid->e[0]
             + (pid->Kd / pid->Ts) * (pid->e[0] - 2.0f*pid->e[1] + pid->e[2]);

    float u = Saturate(pid->u_prev + du, pid->out_min, pid->out_max);
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

/* ── Feedforward ─────────────────────────────────────────────────────────── */

void FF_Init(FeedforwardController_t *ff, const DCMotor_t *motor)
{
    ff->G_ff  = (motor->Rm * motor->b / motor->Kt) + motor->Ke;
    ff->G_aff = motor->Rm / motor->Kt;
}

float FF_Compute(const FeedforwardController_t *ff, float omega_ref, float tau_d)
{
    /* +G_ff·ω*  : voltage needed to track reference velocity at steady state */
    /* +G_aff·τd : disturbance cancellation (sign convention: Robot.c confirmed) */
    return (ff->G_ff * omega_ref) + (ff->G_aff * tau_d);
}
