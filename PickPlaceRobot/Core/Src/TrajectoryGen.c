/*
 * TrajectoryGen.c
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 *
 *  Quintic trajectory with automatic Tf from velocity / acceleration limits.
 *
 *  Peak velocity derivation (quintic, zero-boundary conditions):
 *    ω(t) = 30·Δθ·t²·(Tf−t)² / Tf⁵
 *    Maximum at t = Tf/2:
 *      ω_peak = 30·Δθ·(Tf/2)²·(Tf/2)² / Tf⁵ = (15/8)·|Δθ|/Tf
 *    → Tf_vel = (15/8) · |Δθ| / ω_max
 *
 *  Peak acceleration derivation:
 *    α(t) = dω/dt — roots of dα/dt = 0 at t = Tf·(3 ± √3)/6
 *    Substituting t = Tf·(3−√3)/6 into α(t):
 *      α_peak = (10√3/3) · |Δθ| / Tf²
 *    → Tf_acc = sqrt( (10√3/3) · |Δθ| / α_max )
 *
 *  Duration:  Tf = max(Tf_vel, Tf_acc, TRAJ_TF_MIN)
 */

#include "TrajectoryGen.h"

/* ── Constants derived from quintic peak formulas ────────────────────────── */
#define QUINTIC_VEL_COEFF   1.875f          /* 15/8                          */
#define QUINTIC_ACC_COEFF   5.773502692f    /* 10*sqrt(3)/3                  */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private: compute coefficients from theta_0, theta_f, T_f
 * ═══════════════════════════════════════════════════════════════════════════ */
static void compute_coefficients(Trajectory_t *traj)
{
    float dth = traj->theta_f - traj->theta_0;
    float Tf  = traj->T_f;

    float Tf2 = Tf  * Tf;
    float Tf3 = Tf2 * Tf;
    float Tf4 = Tf3 * Tf;
    float Tf5 = Tf4 * Tf;

    traj->a[0] =  traj->theta_0;
    traj->a[1] =  0.0f;
    traj->a[2] =  0.0f;
    traj->a[3] =  10.0f * dth / Tf3;
    traj->a[4] = -15.0f * dth / Tf4;
    traj->a[5] =   6.0f * dth / Tf5;

    traj->theta_ref = traj->theta_0;
    traj->omega_ref = 0.0f;
    traj->alpha_ref = 0.0f;
    traj->t         = 0.0f;
    traj->active    = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_Init(Trajectory_t *traj,
                     float dt,
                     float omega_max,
                     float alpha_max)
{
    memset(traj, 0, sizeof(Trajectory_t));
    traj->dt        = dt;
    traj->omega_max = omega_max;
    traj->alpha_max = alpha_max;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_ComputeTf
 *
 *  Minimum Tf that satisfies both velocity and acceleration limits.
 * ═══════════════════════════════════════════════════════════════════════════ */
float Trajectory_ComputeTf(const Trajectory_t *traj, float dtheta)
{
    float adth = fabsf(dtheta);

    if (adth < 1e-6f) return TRAJ_TF_MIN;   /* trivial move */

    /* Velocity constraint: Tf >= (15/8) * |Δθ| / ω_max */
    float Tf_vel = (traj->omega_max > 0.0f)
                 ? (QUINTIC_VEL_COEFF * adth / traj->omega_max)
                 : 0.0f;

    /* Acceleration constraint: Tf >= sqrt( (10√3/3) * |Δθ| / α_max ) */
    float Tf_acc = (traj->alpha_max > 0.0f)
                 ? sqrtf(QUINTIC_ACC_COEFF * adth / traj->alpha_max)
                 : 0.0f;

    float Tf = (Tf_vel > Tf_acc) ? Tf_vel : Tf_acc;
    return (Tf > TRAJ_TF_MIN) ? Tf : TRAJ_TF_MIN;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_SetTargetConstrained  (primary API)
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_SetTargetConstrained(Trajectory_t *traj,
                                     float theta_0,
                                     float theta_f)
{
    traj->theta_0 = theta_0;
    traj->theta_f = theta_f;
    traj->T_f     = Trajectory_ComputeTf(traj, theta_f - theta_0);

    compute_coefficients(traj);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_SetTarget  (manual Tf — legacy / homing)
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_SetTarget(Trajectory_t *traj,
                          float theta_0,
                          float theta_f,
                          float T_f)
{
    if (T_f < TRAJ_TF_MIN) T_f = TRAJ_TF_MIN;

    traj->theta_0 = theta_0;
    traj->theta_f = theta_f;
    traj->T_f     = T_f;

    compute_coefficients(traj);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Update
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_Update(Trajectory_t *traj)
{
    if (!traj->active) return;

    float t = traj->t;

    /* Past end — hold final values and disarm */
    if (t >= traj->T_f)
    {
        traj->theta_ref = traj->theta_f;
        traj->omega_ref = 0.0f;
        traj->alpha_ref = 0.0f;
        traj->active    = 0;
        return;
    }

    float t2 = t * t;
    float t3 = t2 * t;
    float t4 = t3 * t;
    float t5 = t4 * t;

    /* Position:     θ(t) = a0 + a3·t³ + a4·t⁴ + a5·t⁵ */
    traj->theta_ref = traj->a[0]
                    + traj->a[3] * t3
                    + traj->a[4] * t4
                    + traj->a[5] * t5;

    /* Velocity:     θ̇(t) = 3a3·t² + 4a4·t³ + 5a5·t⁴ */
    traj->omega_ref = 3.0f * traj->a[3] * t2
                    + 4.0f * traj->a[4] * t3
                    + 5.0f * traj->a[5] * t4;

    /* Acceleration: θ̈(t) = 6a3·t + 12a4·t² + 20a5·t³ */
    traj->alpha_ref = 6.0f  * traj->a[3] * t
                    + 12.0f * traj->a[4] * t2
                    + 20.0f * traj->a[5] * t3;

    traj->t += traj->dt;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Stop / Trajectory_Reset
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_Stop(Trajectory_t *traj)
{
    traj->active    = 0;
    traj->omega_ref = 0.0f;
    traj->alpha_ref = 0.0f;
}

void Trajectory_Reset(Trajectory_t *traj)
{
    float dt        = traj->dt;
    float omega_max = traj->omega_max;
    float alpha_max = traj->alpha_max;
    memset(traj, 0, sizeof(Trajectory_t));
    traj->dt        = dt;
    traj->omega_max = omega_max;
    traj->alpha_max = alpha_max;
}
