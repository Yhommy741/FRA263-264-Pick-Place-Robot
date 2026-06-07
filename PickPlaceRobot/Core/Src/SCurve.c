/*
 * SCurve.c
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * 7-segment jerk-limited S-curve trajectory generator implementation.
 * Patterns 5 (full) and 6 (no cruise) with correct full-distance solve.
 * Position-triggered output: returns theta_ref and vel_ref each tick.
 */

#include "SCurve.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private: compute_phases
 * ═══════════════════════════════════════════════════════════════════════════ */
static void compute_phases(SCurve_t *sc)
{
    const float jmax = sc->jerk_max;
    const float dir  = sc->dir;

    float d1 = sc->t1;
    float d2 = sc->t2 - sc->t1;
    float d3 = sc->t3 - sc->t2;
    float d4 = sc->t4 - sc->t3;
    float d5 = sc->t5 - sc->t4;
    float d6 = sc->t6 - sc->t5;

    sc->p[0] = sc->theta_0;
    sc->v[0] = 0.0f;
    sc->a[0] = 0.0f;

    float j1 = +jmax * dir;
    sc->p[1] = sc->p[0] + sc->v[0]*d1 + 0.5f*sc->a[0]*d1*d1 + (1.0f/6.0f)*j1*d1*d1*d1;
    sc->v[1] = sc->v[0] + sc->a[0]*d1 + 0.5f*j1*d1*d1;
    sc->a[1] = sc->a[0] + j1*d1;

    sc->p[2] = sc->p[1] + sc->v[1]*d2 + 0.5f*sc->a[1]*d2*d2;
    sc->v[2] = sc->v[1] + sc->a[1]*d2;
    sc->a[2] = sc->a[1];

    float j3 = -jmax * dir;
    sc->p[3] = sc->p[2] + sc->v[2]*d3 + 0.5f*sc->a[2]*d3*d3 + (1.0f/6.0f)*j3*d3*d3*d3;
    sc->v[3] = sc->v[2] + sc->a[2]*d3 + 0.5f*j3*d3*d3;
    sc->a[3] = sc->a[2] + j3*d3;

    sc->p[4] = sc->p[3] + sc->v[3]*d4;
    sc->v[4] = sc->v[3];
    sc->a[4] = 0.0f;

    float j5 = -jmax * dir;
    sc->p[5] = sc->p[4] + sc->v[4]*d5 + 0.5f*sc->a[4]*d5*d5 + (1.0f/6.0f)*j5*d5*d5*d5;
    sc->v[5] = sc->v[4] + sc->a[4]*d5 + 0.5f*j5*d5*d5;
    sc->a[5] = sc->a[4] + j5*d5;

    sc->p[6] = sc->p[5] + sc->v[5]*d6 + 0.5f*sc->a[5]*d6*d6;
    sc->v[6] = sc->v[5] + sc->a[5]*d6;
    sc->a[6] = sc->a[5];
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private: plan_scurve
 * ═══════════════════════════════════════════════════════════════════════════ */
static void plan_scurve(SCurve_t *sc, float theta_0, float theta_f)
{
    const float vmax = sc->omega_max;
    const float amax = sc->alpha_max;
    const float jmax = sc->jerk_max;

    sc->theta_0 = theta_0;
    sc->theta_f = theta_f;

    float delta = theta_f - theta_0;
    sc->dir = (delta >= 0.0f) ? 1.0f : -1.0f;
    float s = fabsf(delta);

    if (s < 1e-6f)
    {
        sc->t1 = sc->t2 = sc->t3 = 0.0f;
        sc->t4 = sc->t5 = sc->t6 = sc->t7 = SCURVE_TF_MIN;
        sc->T_f      = SCURVE_TF_MIN;
        sc->theta_ref = theta_0;
        sc->omega_ref = 0.0f;
        sc->alpha_ref = 0.0f;
        sc->t         = 0.0f;
        sc->active    = 0;
        return;
    }

    float tj_nom = amax / jmax;
    float sa     = 2.0f * (amax * amax * amax) / (jmax * jmax);
    float sv     = vmax * (vmax / amax + amax / jmax);

    float tj, ta, tv;

    if (s >= sv)
    {
        /* Pattern 5: full 7-segment */
        tj = tj_nom;
        ta = vmax / amax;
        tv = s / vmax;
    }
    else if (s >= sa)
    {
        /* Pattern 6: no cruise */
        tj = tj_nom;
        ta = 0.5f * (-tj + sqrtf(tj * tj + 4.0f * s / amax));
        if (ta < tj) ta = tj;
        tv = ta + tj;
    }
    else
    {
        /* Pattern 2/3/4: jerk-only ramps */
        tj = cbrtf(s / (2.0f * jmax));
        ta = tj;
        tv = 2.0f * tj;
    }

    sc->t1 = tj;
    sc->t2 = ta;
    sc->t3 = ta + tj;
    sc->t4 = tv;
    sc->t5 = tv + tj;
    sc->t6 = tv + ta;
    sc->t7 = tv + ta + tj;

    if (sc->t7 < SCURVE_TF_MIN) sc->t7 = SCURVE_TF_MIN;
    sc->T_f = sc->t7;

    compute_phases(sc);

    sc->theta_ref = theta_0;
    sc->omega_ref = 0.0f;
    sc->alpha_ref = 0.0f;
    sc->t         = 0.0f;
    sc->active    = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SCurve_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void SCurve_Init(SCurve_t *sc, float dt,
                 float omega_max, float alpha_max, float jerk_max)
{
    memset(sc, 0, sizeof(SCurve_t));
    sc->dt        = dt;
    sc->omega_max = omega_max;
    sc->alpha_max = alpha_max;
    sc->jerk_max  = jerk_max;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SCurve_SetTarget
 * ═══════════════════════════════════════════════════════════════════════════ */
void SCurve_SetTarget(SCurve_t *sc, float theta_0, float theta_f)
{
    plan_scurve(sc, theta_0, theta_f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SCurve_ComputeTf
 * ═══════════════════════════════════════════════════════════════════════════ */
float SCurve_ComputeTf(const SCurve_t *sc, float dtheta)
{
    float s    = fabsf(dtheta);
    float vmax = sc->omega_max;
    float amax = sc->alpha_max;
    float jmax = sc->jerk_max;

    if (s < 1e-6f) return SCURVE_TF_MIN;

    float tj = amax / jmax;
    float ta = vmax / amax;
    float tv = s / vmax;
    float T_f = tv + ta + tj;
    return (T_f > SCURVE_TF_MIN) ? T_f : SCURVE_TF_MIN;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SCurve_Update
 * ═══════════════════════════════════════════════════════════════════════════ */
void SCurve_Update(SCurve_t *sc)
{
    if (!sc->active) return;

    float t = sc->t;

    if (t >= sc->T_f)
    {
        sc->theta_ref = sc->theta_f;
        sc->omega_ref = 0.0f;
        sc->alpha_ref = 0.0f;
        sc->active    = 0;
        return;
    }

    float tau, p_i, v_i, a_i, j_i;
    const float jmax = sc->jerk_max;
    const float dir  = sc->dir;

    if      (t <= sc->t1) { tau=t;          p_i=sc->p[0]; v_i=sc->v[0]; a_i=sc->a[0]; j_i=+jmax*dir; }
    else if (t <= sc->t2) { tau=t-sc->t1;   p_i=sc->p[1]; v_i=sc->v[1]; a_i=sc->a[1]; j_i=0.0f;      }
    else if (t <= sc->t3) { tau=t-sc->t2;   p_i=sc->p[2]; v_i=sc->v[2]; a_i=sc->a[2]; j_i=-jmax*dir; }
    else if (t <= sc->t4) { tau=t-sc->t3;   p_i=sc->p[3]; v_i=sc->v[3]; a_i=sc->a[3]; j_i=0.0f;      }
    else if (t <= sc->t5) { tau=t-sc->t4;   p_i=sc->p[4]; v_i=sc->v[4]; a_i=sc->a[4]; j_i=-jmax*dir; }
    else if (t <= sc->t6) { tau=t-sc->t5;   p_i=sc->p[5]; v_i=sc->v[5]; a_i=sc->a[5]; j_i=0.0f;      }
    else                  { tau=t-sc->t6;   p_i=sc->p[6]; v_i=sc->v[6]; a_i=sc->a[6]; j_i=+jmax*dir; }

    float tau2 = tau * tau;
    float tau3 = tau2 * tau;

    sc->theta_ref = p_i + v_i*tau  + 0.5f*a_i*tau2 + (1.0f/6.0f)*j_i*tau3;
    sc->omega_ref =       v_i      + a_i*tau        + 0.5f*j_i*tau2;
    sc->alpha_ref =                  a_i             + j_i*tau;

    sc->t += sc->dt;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  SCurve_Stop / SCurve_Reset
 * ═══════════════════════════════════════════════════════════════════════════ */
void SCurve_Stop(SCurve_t *sc)
{
    sc->active    = 0;
    sc->omega_ref = 0.0f;
    sc->alpha_ref = 0.0f;
}

void SCurve_Reset(SCurve_t *sc)
{
    float dt   = sc->dt;
    float om   = sc->omega_max;
    float al   = sc->alpha_max;
    float jk   = sc->jerk_max;
    memset(sc, 0, sizeof(SCurve_t));
    sc->dt        = dt;
    sc->omega_max = om;
    sc->alpha_max = al;
    sc->jerk_max  = jk;
}
