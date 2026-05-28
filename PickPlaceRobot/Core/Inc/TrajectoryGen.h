/*
 * TrajectoryGen.h
 *
 *  Updated : May 2026
 *  Author  : FRA263/264 Group 5
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  7-Segment S-Curve (Jerk-Limited) Trajectory Generator
 *
 *  Generates a smooth, jerk-limited position profile with 7 phases:
 *
 *    Phase 1  [0,       t1]   Jerk = +jmax      (accel ramp-up)
 *    Phase 2  [t1,      t2]   Jerk =  0         (constant accel)
 *    Phase 3  [t2,      t3]   Jerk = -jmax      (accel ramp-down)
 *    Phase 4  [t3,      t4]   Jerk =  0         (constant velocity cruise)
 *    Phase 5  [t4,      t5]   Jerk = -jmax      (decel ramp-up)
 *    Phase 6  [t5,      t6]   Jerk =  0         (constant decel)
 *    Phase 7  [t6,      t7]   Jerk = +jmax      (decel ramp-down)
 *
 *  Boundary conditions: θ̇(0)=0, θ̈(0)=0, θ̇(T)=0, θ̈(T)=0
 *
 *  Handles all 6 feasibility patterns from Biagiotti & Melchiorri:
 *    Pattern 1: vmax ≤ va, s ≥ sa           (no cruise, no const-accel)
 *    Pattern 2/3/4: short distance variants  (jerk-limited only)
 *    Pattern 5: vmax ≥ va, s ≥ sv           (full 7-segment)
 *    Pattern 6: vmax ≥ va, sa ≤ s < sv      (no cruise phase)
 *
 *  All state is per-instance (no static globals) — multi-axis safe.
 *  All arithmetic in float — Cortex-M4 FPU optimised.
 *
 * ── API ────────────────────────────────────────────────────────────────────
 *  Same as previous quintic version — drop-in replacement.
 *  Trajectory_Init now takes jerk_max as additional parameter.
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

    /* Constraints */
    float omega_max;    /* Peak velocity limit      [rad/s]     */
    float alpha_max;    /* Peak acceleration limit  [rad/s²]    */
    float jerk_max;     /* Peak jerk limit          [rad/s³]    */

    /* Timing */
    float dt;           /* Slow-loop sample period  [s]         */
    float T_f;          /* Total move duration      [s]         */

    /* 7-segment time stamps */
    float t1, t2, t3, t4, t5, t6, t7;

    /* Direction of travel: +1 or -1 */
    float dir;

    /* Start / end */
    float theta_0;
    float theta_f;

    /* Per-phase boundary states (pos, vel, accel at start of each phase)
     * Indexed [0..6] → phases 1..7                                          */
    float p[7];   /* position  at start of phase i */
    float v[7];   /* velocity  at start of phase i */
    float a[7];   /* accel     at start of phase i */

    /* Internal time counter */
    float t;

    /* Reference outputs — read after Trajectory_Update() */
    float theta_ref;    /* [rad]     */
    float omega_ref;    /* [rad/s]   */
    float alpha_ref;    /* [rad/s²]  */

    /* Status */
    uint8_t active;     /* 1 = running, 0 = at target */

} Trajectory_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  API  (drop-in compatible with quintic version except Trajectory_Init)
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise trajectory handle.
 *         Call once. Sets the slow-loop sample period and motion constraints.
 *
 * @param  traj       Handle
 * @param  dt         Slow-loop sample period [s]  (= Ts * CTRL_LOOP_MULTI)
 * @param  omega_max  Maximum velocity    [rad/s]
 * @param  alpha_max  Maximum acceleration [rad/s²]
 * @param  jerk_max   Maximum jerk         [rad/s³]
 */
void Trajectory_Init(Trajectory_t *traj,
                     float dt,
                     float omega_max,
                     float alpha_max,
                     float jerk_max);

/**
 * @brief  Plan a move with AUTOMATIC duration from motion constraints.
 *         Tf is computed so that none of ω_max, α_max, j_max is exceeded.
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
 *         Ignores constraints — caller is responsible.
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
 *         the stored ω_max, α_max, j_max.
 *
 * @param  traj   Handle (reads constraints)
 * @param  dtheta Displacement |θ_f − θ_0| [rad]
 * @return Computed Tf [s]
 */
float Trajectory_ComputeTf(const Trajectory_t *traj, float dtheta);

#endif /* INC_TRAJECTORYGEN_H_ */
