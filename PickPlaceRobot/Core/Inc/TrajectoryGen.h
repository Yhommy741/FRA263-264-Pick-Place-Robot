/*
 * TrajectoryGen.h
 *
 *  Created : May 2026
 *  Author  : FRA263/264 Group 5
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  Quintic (5th-order) Trajectory Generator
 *  with automatic duration from velocity and acceleration constraints
 *
 *  Profile shape: zero velocity and acceleration at both endpoints.
 *    θ(0)  = θ_0,  θ̇(0)  = 0,  θ̈(0)  = 0
 *    θ(Tf) = θ_f,  θ̇(Tf) = 0,  θ̈(Tf) = 0
 *
 *  Polynomial:
 *    θ(t) = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
 *    a0 =  θ_0,  a1 = 0,  a2 = 0
 *    a3 =  10·Δθ / Tf³
 *    a4 = −15·Δθ / Tf⁴
 *    a5 =   6·Δθ / Tf⁵       where Δθ = θ_f − θ_0
 *
 * ── Automatic Tf from constraints ─────────────────────────────────────────
 *
 *  Peak velocity  (occurs at t = Tf/2):
 *    ω_peak = (15/8) · |Δθ| / Tf
 *    → Tf_vel = (15/8) · |Δθ| / ω_max
 *
 *  Peak acceleration  (occurs at t = Tf·(3 − √3)/6 ≈ 0.211·Tf):
 *    α_peak = (10√3/3) · |Δθ| / Tf²
 *    → Tf_acc = sqrt( (10√3/3) · |Δθ| / α_max )
 *
 *  Final duration:
 *    Tf = max(Tf_vel, Tf_acc)     ← both constraints satisfied
 *    Tf = max(Tf, Tf_min)         ← lower-bounded for numerical safety
 *
 * ── API ────────────────────────────────────────────────────────────────────
 *
 *  Old API (manual Tf):
 *    Trajectory_SetTarget(traj, theta_0, theta_f, T_f)   ← still available
 *
 *  New API (constraint-based):
 *    Trajectory_SetTargetConstrained(traj, theta_0, theta_f, omega_max, alpha_max)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#ifndef INC_TRAJECTORYGEN_H_
#define INC_TRAJECTORYGEN_H_

#include <stdint.h>
#include <string.h>
#include <math.h>

/* ── Minimum allowed duration [s] ────────────────────────────────────────── */
#define TRAJ_TF_MIN     0.05f

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory handle
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {

    /* Polynomial coefficients  θ(t) = Σ a[i]·tⁱ */
    float a[6];

    /* Profile parameters */
    float T_f;          /* Computed move duration   [s]     */
    float dt;           /* Slow-loop sample period  [s]     */

    /* Constraints (stored for reference) */
    float omega_max;    /* Peak velocity limit   [rad/s]    */
    float alpha_max;    /* Peak acceleration limit [rad/s²] */

    /* Internal time counter */
    float t;

    /* Reference outputs — read after Trajectory_Update() */
    float theta_ref;    /* [rad]     */
    float omega_ref;    /* [rad/s]   */
    float alpha_ref;    /* [rad/s²]  */

    /* Start / end */
    float theta_0;
    float theta_f;

    /* Status */
    uint8_t active;     /* 1 = running, 0 = at target */

} Trajectory_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise trajectory handle.
 *         Call once. Sets the slow-loop sample period and motion constraints.
 *
 * @param  traj       Handle
 * @param  dt         Slow-loop sample period [s]  (= Ts * ROBOT_POS_DIVIDER)
 * @param  omega_max  Maximum velocity   [rad/s]  — used by SetTargetConstrained
 * @param  alpha_max  Maximum acceleration [rad/s²] — used by SetTargetConstrained
 */
void Trajectory_Init(Trajectory_t *traj,
                     float dt,
                     float omega_max,
                     float alpha_max);

/**
 * @brief  Plan a move with AUTOMATIC duration from motion constraints.
 *
 *         Tf is computed so that neither ω_max nor α_max is exceeded:
 *           Tf = max( (15/8)·|Δθ|/ω_max,  sqrt((10√3/3)·|Δθ|/α_max) )
 *
 *         Use this as the primary API for commanded moves.
 *
 * @param  traj     Handle
 * @param  theta_0  Start position  [rad]
 * @param  theta_f  Target position [rad]
 */
void Trajectory_SetTargetConstrained(Trajectory_t *traj,
                                     float theta_0,
                                     float theta_f);

/**
 * @brief  Plan a move with MANUAL duration (legacy / homing use).
 *         Ignores ω_max / α_max — caller is responsible.
 *
 * @param  traj     Handle
 * @param  theta_0  Start position [rad]
 * @param  theta_f  Target position [rad]
 * @param  T_f      Move duration  [s]  (must be > TRAJ_TF_MIN)
 */
void Trajectory_SetTarget(Trajectory_t *traj,
                          float theta_0,
                          float theta_f,
                          float T_f);

/**
 * @brief  Advance trajectory by one slow-loop tick.
 *         Call BEFORE reading theta_ref / omega_ref.
 *         Sets active = 0 and holds final values when t ≥ T_f.
 */
void Trajectory_Update(Trajectory_t *traj);

/**
 * @brief  Freeze outputs at current values, set active = 0.
 */
void Trajectory_Stop(Trajectory_t *traj);

/**
 * @brief  Reset to zero state (active = 0, all outputs = 0).
 */
void Trajectory_Reset(Trajectory_t *traj);

/**
 * @brief  Compute the minimum Tf for a given displacement, respecting
 *         the stored ω_max and α_max.  Useful for preview / logging.
 *
 * @param  traj   Handle (reads omega_max, alpha_max)
 * @param  dtheta Displacement |θ_f − θ_0| [rad]
 * @return Computed Tf [s]
 */
float Trajectory_ComputeTf(const Trajectory_t *traj, float dtheta);

#endif /* INC_TRAJECTORYGEN_H_ */
