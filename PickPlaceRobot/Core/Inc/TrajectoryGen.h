/*
 * TrajectoryGen.h
 *
 *  Created: May 2026
 *  Author : FRA263/264 Group 5
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  5th-Order Polynomial (Quintic) Trajectory Generator
 *
 *  Generates a smooth position profile from θ_0 to θ_f in time T_f.
 *  Boundary conditions (zero vel and accel at start and end):
 *
 *    θ(0)   = θ_0       θ(T_f)   = θ_f
 *    θ̇(0)   = 0         θ̇(T_f)   = 0
 *    θ̈(0)   = 0         θ̈(T_f)   = 0
 *
 *  Polynomial form:
 *    θ(t) = a0 + a1·t + a2·t² + a3·t³ + a4·t⁴ + a5·t⁵
 *
 *  Solving the 6 boundary conditions gives:
 *    a0 =  θ_0
 *    a1 =  0
 *    a2 =  0
 *    a3 =  10·Δθ / T_f³
 *    a4 = -15·Δθ / T_f⁴
 *    a5 =   6·Δθ / T_f⁵
 *  where Δθ = θ_f − θ_0
 *
 *  Velocity:
 *    θ̇(t) = 3·a3·t² + 4·a4·t³ + 5·a5·t⁴
 *
 *  Acceleration:
 *    θ̈(t) = 6·a3·t + 12·a4·t² + 20·a5·t³
 *
 *  All outputs are held at final values after t > T_f.
 * ═══════════════════════════════════════════════════════════════════════════
 */

#ifndef INC_TRAJECTORYGEN_H_
#define INC_TRAJECTORYGEN_H_

#include <stdint.h>
#include <string.h>

/* ════════════════════════════════════════════════════════════════════════════
 *  Trajectory_t
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {

    /* ── Polynomial coefficients ─────────────────────────────────────── */
    float a[6];        /* a[0]..a[5]  for θ(t) = Σ a[i]·t^i         */

    /* ── Profile parameters ──────────────────────────────────────────── */
    float T_f;         /* Total move duration     [s]                  */
    float dt;          /* Sample period           [s]                  */

    /* ── Internal time counter ───────────────────────────────────────── */
    float t;           /* Elapsed time since move start [s]            */

    /* ── Reference outputs (updated every tick) ──────────────────────── */
    float theta_ref;   /* Position reference  [rad]                    */
    float omega_ref;   /* Velocity reference  [rad/s]                  */
    float alpha_ref;   /* Acceleration reference [rad/s²]              */

    /* ── Start / end snapshots ───────────────────────────────────────── */
    float theta_0;     /* Start position [rad]                         */
    float theta_f;     /* Target position [rad]                        */

    /* ── Status ──────────────────────────────────────────────────────── */
    uint8_t active;    /* 1 = profile running, 0 = at target           */

} Trajectory_t;

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise the trajectory handle.
 *         Must be called once before any other Trajectory_* call.
 *
 * @param  traj  Trajectory handle
 * @param  dt    Control sample period [s]
 */
void Trajectory_Init(Trajectory_t *traj, float dt);

/**
 * @brief  Plan and start a new quintic move.
 *
 *         Computes coefficients a[0]..a[5] from:
 *           start position  = theta_0  (current position)
 *           target position = theta_f
 *           move duration   = T_f  [s]
 *
 *         Immediately arms the profile (active = 1) and resets t = 0.
 *
 * @param  traj     Trajectory handle
 * @param  theta_0  Start position  [rad]
 * @param  theta_f  Target position [rad]
 * @param  T_f      Move duration   [s]  (must be > 0)
 */
void Trajectory_SetTarget(Trajectory_t *traj,
                           float theta_0, float theta_f, float T_f);

/**
 * @brief  Step the trajectory by one sample period.
 *
 *         Call every control tick BEFORE reading theta_ref / omega_ref.
 *         After t >= T_f, holds outputs at final values and sets active = 0.
 *
 * @param  traj  Trajectory handle
 */
void Trajectory_Update(Trajectory_t *traj);

/**
 * @brief  Stop the profile immediately and hold current outputs.
 *         Sets active = 0 and freezes theta_ref, omega_ref, alpha_ref.
 *
 * @param  traj  Trajectory handle
 */
void Trajectory_Stop(Trajectory_t *traj);

/**
 * @brief  Reset handle to zero state (idle, outputs = 0).
 *
 * @param  traj  Trajectory handle
 */
void Trajectory_Reset(Trajectory_t *traj);

#endif /* INC_TRAJECTORYGEN_H_ */
