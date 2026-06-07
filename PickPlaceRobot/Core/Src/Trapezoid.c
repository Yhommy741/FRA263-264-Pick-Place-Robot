/*
 * Trapezoid.c
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * 3-phase position-triggered trapezoidal trajectory generator.
 * Phase transitions on distance travelled, not elapsed time.
 * Avoids overshoot caused by motor lag at deceleration entry.
 */

#include "Trapezoid.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private: plan
 * ═══════════════════════════════════════════════════════════════════════════ */
static void plan(Trapezoid_t *trap,
                 float theta_0, float theta_f,
                 float vmax,    float amax)
{
    trap->theta_0   = theta_0;
    trap->theta_f   = theta_f;
    trap->theta_ref = theta_0;
    trap->omega_ref = 0.0f;
    trap->alpha_ref = 0.0f;
    trap->phase     = 0;

    float delta = theta_f - theta_0;
    trap->dir    = (delta >= 0.0f) ? 1.0f : -1.0f;
    trap->stroke = fabsf(delta);

    if (trap->stroke < 1e-6f)
    {
        trap->v_cruise      = 0.0f;
        trap->a_used        = amax;
        trap->s_ramp        = 0.0f;
        trap->T_f           = 0.0f;
        trap->active        = 0;
        trap->is_triangular = 0;
        return;
    }

    float v_cruise, s_ramp;

    if ((vmax * vmax / amax) <= trap->stroke)
    {
        /* Trapezoidal — v_target reached */
        v_cruise            = vmax;
        s_ramp              = (vmax * vmax) / (2.0f * amax);
        trap->is_triangular = 0;
        /* T_f estimate: ta + tv_cruise + ta */
        float ta   = vmax / amax;
        float s_cr = trap->stroke - 2.0f * s_ramp;
        trap->T_f  = 2.0f * ta + s_cr / vmax;
    }
    else
    {
        /* Triangular — reduce v to fit stroke */
        v_cruise            = sqrtf(amax * trap->stroke);   /* = amax * ta */
        s_ramp              = trap->stroke / 2.0f;
        trap->is_triangular = 1;
        trap->T_f           = 2.0f * sqrtf(trap->stroke / amax);
    }

    trap->v_cruise = v_cruise;
    trap->a_used   = amax;
    trap->s_ramp   = s_ramp;
    trap->active   = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trapezoid_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trapezoid_Init(Trapezoid_t *trap, float dt,
                    float omega_max, float alpha_max)
{
    memset(trap, 0, sizeof(Trapezoid_t));
    trap->dt        = dt;
    trap->omega_max = omega_max;
    trap->alpha_max = alpha_max;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trapezoid_SetTarget / Trapezoid_SetTargetConstrained
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trapezoid_SetTarget(Trapezoid_t *trap, float theta_0, float theta_f)
{
    plan(trap, theta_0, theta_f, trap->omega_max, trap->alpha_max);
}

void Trapezoid_SetTargetConstrained(Trapezoid_t *trap,
                                    float theta_0, float theta_f,
                                    float omega_max, float alpha_max)
{
    plan(trap, theta_0, theta_f, omega_max, alpha_max);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trapezoid_ComputeTf
 * ═══════════════════════════════════════════════════════════════════════════ */
float Trapezoid_ComputeTf(const Trapezoid_t *trap, float dtheta)
{
    float s    = fabsf(dtheta);
    float vmax = trap->omega_max;
    float amax = trap->alpha_max;
    if (s < 1e-6f) return 0.0f;
    if ((vmax * vmax / amax) <= s)
        return s / vmax + vmax / amax;
    else
        return 2.0f * sqrtf(s / amax);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trapezoid_Update  —  position-triggered phase transitions
 *
 *  Called every slow-loop tick with the ACTUAL robot position theta_actual.
 *
 *  Phase logic:
 *    dist     = |theta_actual - theta_0|   distance already travelled
 *    remaining= stroke - dist              distance still to go
 *
 *    Phase 0 (ACCEL):
 *      omega_ref += a * dt * dir   (ramp up)
 *      cap at v_cruise
 *      theta_ref += omega_ref * dt (integrate)
 *      → CRUISE when dist >= s_ramp
 *
 *    Phase 1 (CRUISE):
 *      omega_ref  = v_cruise * dir (hold)
 *      theta_ref += omega_ref * dt
 *      → DECEL when remaining <= s_ramp
 *
 *    Phase 2 (DECEL):
 *      omega_ref -= a * dt * dir   (ramp down)
 *      clamp to zero crossing
 *      theta_ref += omega_ref * dt
 *      → DONE when remaining <= 0 OR omega crosses zero
 *
 *    Phase 3 (DONE):
 *      theta_ref  = theta_f, omega_ref = 0, active = 0
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trapezoid_Update(Trapezoid_t *trap, float theta_actual)
{
    if (!trap->active) return;

    const float dt    = trap->dt;
    const float a     = trap->a_used;
    const float dir   = trap->dir;
    const float v_max = trap->v_cruise;

    /* Distance travelled and remaining — always positive scalars */
    float dist      = (theta_actual - trap->theta_0) * dir;
    float remaining = trap->stroke - dist;

    switch (trap->phase)
    {
        case 0: /* ACCEL ───────────────────────────────────────────────── */
            trap->omega_ref += dir * a * dt;
            /* Cap at cruise velocity */
            if (dir > 0.0f && trap->omega_ref >  v_max) trap->omega_ref =  v_max;
            if (dir < 0.0f && trap->omega_ref < -v_max) trap->omega_ref = -v_max;
            trap->alpha_ref = dir * a;
            /* Transition to cruise when ramp distance covered */
            if (dist >= trap->s_ramp)
            {
                trap->omega_ref = dir * v_max;   /* snap to exact cruise vel */
                trap->phase     = 1;
            }
            break;

        case 1: /* CRUISE ──────────────────────────────────────────────── */
            trap->omega_ref = dir * v_max;
            trap->alpha_ref = 0.0f;
            /* Transition to decel when decel distance remains */
            if (remaining <= trap->s_ramp)
                trap->phase = 2;
            break;

        case 2: /* DECEL ───────────────────────────────────────────────── */
            trap->omega_ref -= dir * a * dt;
            trap->alpha_ref  = -dir * a;
            /* Clamp: don't command past zero (would reverse direction) */
            if (dir > 0.0f && trap->omega_ref < 0.0f) trap->omega_ref = 0.0f;
            if (dir < 0.0f && trap->omega_ref > 0.0f) trap->omega_ref = 0.0f;
            /* Transition to done when target reached or velocity zeroed */
            if (remaining <= 0.0f || trap->omega_ref == 0.0f)
                trap->phase = 3;
            break;

        case 3: /* DONE ────────────────────────────────────────────────── */
        default:
            trap->theta_ref = trap->theta_f;
            trap->omega_ref = 0.0f;
            trap->alpha_ref = 0.0f;
            trap->active    = 0;
            return;
    }

    /* Integrate omega_ref → theta_ref each tick */
    trap->theta_ref += trap->omega_ref * dt;

    /* Hard clamp: theta_ref must not overshoot theta_f */
    if (dir > 0.0f && trap->theta_ref > trap->theta_f)
        trap->theta_ref = trap->theta_f;
    if (dir < 0.0f && trap->theta_ref < trap->theta_f)
        trap->theta_ref = trap->theta_f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trapezoid_Stop / Trapezoid_Reset
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trapezoid_Stop(Trapezoid_t *trap)
{
    trap->active    = 0;
    trap->omega_ref = 0.0f;
    trap->alpha_ref = 0.0f;
    trap->phase     = 3;
}

void Trapezoid_Reset(Trapezoid_t *trap)
{
    float dt  = trap->dt;
    float om  = trap->omega_max;
    float al  = trap->alpha_max;
    memset(trap, 0, sizeof(Trapezoid_t));
    trap->dt        = dt;
    trap->omega_max = om;
    trap->alpha_max = al;
}
