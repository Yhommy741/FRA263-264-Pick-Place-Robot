/*
 * Trajectory.c
 *
 *  Created: May 2026
 *  Author : FRA263/264 Group 5
 *
 *  Quintic polynomial trajectory — see TrajectoryGen.h for full derivation.
 */

#include "TrajectoryGen.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_Init(Trajectory_t *traj, float dt)
{
    memset(traj, 0, sizeof(Trajectory_t));
    traj->dt = dt;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_SetTarget
 *
 *  Boundary conditions (zero vel & accel at both ends):
 *    θ(0)=θ_0, θ̇(0)=0, θ̈(0)=0
 *    θ(Tf)=θ_f, θ̇(Tf)=0, θ̈(Tf)=0
 *
 *  Closed-form solution:
 *    a0 =  θ_0
 *    a1 =  0
 *    a2 =  0
 *    a3 =  10·Δθ / Tf³
 *    a4 = -15·Δθ / Tf⁴
 *    a5 =   6·Δθ / Tf⁵
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_SetTarget(Trajectory_t *traj,
                           float theta_0, float theta_f, float T_f)
{
    float dtheta;
    float Tf2, Tf3, Tf4, Tf5;

    if (T_f <= 0.0f) return;   /* invalid duration — ignore */

    traj->theta_0 = theta_0;
    traj->theta_f = theta_f;
    traj->T_f     = T_f;
    traj->t       = 0.0f;

    dtheta = theta_f - theta_0;

    Tf2 = T_f  * T_f;
    Tf3 = Tf2  * T_f;
    Tf4 = Tf3  * T_f;
    Tf5 = Tf4  * T_f;

    /* Polynomial coefficients */
    traj->a[0] =  theta_0;
    traj->a[1] =  0.0f;
    traj->a[2] =  0.0f;
    traj->a[3] =  10.0f * dtheta / Tf3;
    traj->a[4] = -15.0f * dtheta / Tf4;
    traj->a[5] =   6.0f * dtheta / Tf5;

    /* Initialise outputs at start */
    traj->theta_ref = theta_0;
    traj->omega_ref = 0.0f;
    traj->alpha_ref = 0.0f;

    traj->active = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Update
 *
 *  Evaluates the polynomial and its derivatives at current time t,
 *  then advances t by dt.
 *
 *  θ(t)  = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
 *  θ̇(t)  =      a1 + 2a2·t + 3a3·t² + 4a4·t³ + 5a5·t⁴
 *  θ̈(t)  =           2a2   + 6a3·t  + 12a4·t² + 20a5·t³
 *
 *  Uses Horner's method for numerical efficiency.
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_Update(Trajectory_t *traj)
{
    float t, t2, t3, t4, t5;

    if (!traj->active) return;

    /* Clamp evaluation time */
    t = traj->t;
    if (t >= traj->T_f)
    {
        /* Hold final values and disarm */
        traj->theta_ref = traj->theta_f;
        traj->omega_ref = 0.0f;
        traj->alpha_ref = 0.0f;
        traj->active    = 0;
        return;
    }

    /* Powers of t */
    t2 = t * t;
    t3 = t2 * t;
    t4 = t3 * t;
    t5 = t4 * t;

    /* Position:  θ(t) = a0 + a1t + a2t² + a3t³ + a4t⁴ + a5t⁵ */
    traj->theta_ref = traj->a[0]
                    + traj->a[1] * t
                    + traj->a[2] * t2
                    + traj->a[3] * t3
                    + traj->a[4] * t4
                    + traj->a[5] * t5;

    /* Velocity:  θ̇(t) = a1 + 2a2t + 3a3t² + 4a4t³ + 5a5t⁴ */
    traj->omega_ref = traj->a[1]
                    + 2.0f * traj->a[2] * t
                    + 3.0f * traj->a[3] * t2
                    + 4.0f * traj->a[4] * t3
                    + 5.0f * traj->a[5] * t4;

    /* Acceleration:  θ̈(t) = 2a2 + 6a3t + 12a4t² + 20a5t³ */
    traj->alpha_ref = 2.0f  * traj->a[2]
                    + 6.0f  * traj->a[3] * t
                    + 12.0f * traj->a[4] * t2
                    + 20.0f * traj->a[5] * t3;

    /* Advance time */
    traj->t += traj->dt;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Stop
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_Stop(Trajectory_t *traj)
{
    traj->active    = 0;
    traj->omega_ref = 0.0f;
    traj->alpha_ref = 0.0f;
    /* theta_ref stays at last evaluated position */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Reset
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_Reset(Trajectory_t *traj)
{
    float dt = traj->dt;
    memset(traj, 0, sizeof(Trajectory_t));
    traj->dt = dt;
}
