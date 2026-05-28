/*
 * TrajectoryGen.c
 *
 *  Updated : May 2026
 *  Author  : FRA263/264 Group 5
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  7-Segment S-Curve (Jerk-Limited) Trajectory Generator
 *
 *  Reference: Biagiotti & Melchiorri, "Trajectory Planning for Automatic
 *             Machines and Robots", Chapter 3.
 *
 *  All state stored per-instance (p[], v[], a[] arrays) — no static globals.
 *  All arithmetic in float for Cortex-M4 FPU efficiency.
 *
 * ── Planning (Trajectory_SetTargetConstrained) ──────────────────────────
 *
 *  Given:  s = |Δθ|,  vmax, amax, jmax
 *
 *  Derived limits:
 *    va   = amax² / jmax         (velocity at which accel phase is triangular)
 *    sa   = 2·amax³ / jmax²      (min distance to reach amax)
 *    sv   = vmax · (ta + tj)     (min distance to reach vmax)
 *      where ta = vmax/amax, tj = amax/jmax  (pattern 5 nominal)
 *
 *  Feasibility patterns and their (tj, ta, tv):
 *    P1: vmax ≤ va,  s ≥ sa   → triangular accel, cruise
 *         tj = sqrt(vmax/jmax),  ta = tj,           tv = s/vmax
 *    P2/P3/P4: vmax < va, s < sa  → short, jerk-only ramps
 *         tj = (s/(2·jmax))^(1/3), ta = tj,  tv = 2·tj
 *    P5: vmax ≥ va, s ≥ sv    → full 7-segment
 *         tj = amax/jmax,  ta = vmax/amax,  tv = s/vmax
 *    P6: vmax ≥ va, sa ≤ s < sv  → no cruise (tv = ta+tj)
 *         tj = amax/jmax
 *         ta = 0.5*(sqrt((4·s·j²+a³)/(a·j²)) − a/j)
 *         tv = ta + tj
 *
 *  Phase timestamps:
 *    t1 = tj
 *    t2 = ta               (= tj for patterns 1/2/3/4)
 *    t3 = ta + tj
 *    t4 = tv
 *    t5 = tv + tj
 *    t6 = tv + ta
 *    t7 = tv + ta + tj     (= T_f)
 *
 * ── Evaluation (Trajectory_Update) ──────────────────────────────────────
 *
 *  Each phase uses exact kinematic equations from its own boundary state:
 *    pos(τ) = p_i + v_i·τ + ½·a_i·τ² + ⅙·j_i·τ³
 *    vel(τ) = v_i         + a_i·τ     + ½·j_i·τ²
 *    acc(τ) = a_i                     + j_i·τ
 *  where τ = t − t_start_of_phase,  j_i = ±jmax or 0
 *
 *  Boundary states are pre-computed at plan time (compute_phases()).
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include "TrajectoryGen.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private: compute_phases
 *
 *  Pre-computes the (p,v,a) state at the START of each of the 7 phases.
 *  The direction sign is already baked into the jerk terms at call sites;
 *  boundary states are always in the SIGNED (directional) domain.
 *
 *  Phase jerk values (signed, applied in compute_phases):
 *    Ph1: +jmax·dir    Ph2:  0    Ph3: -jmax·dir
 *    Ph4:  0           Ph5: -jmax·dir
 *    Ph6:  0           Ph7: +jmax·dir
 * ═══════════════════════════════════════════════════════════════════════════ */
static void compute_phases(Trajectory_t *traj)
{
    const float jmax = traj->jerk_max;
    const float dir  = traj->dir;

    /* Phase durations */
    float d1 = traj->t1;
    float d2 = traj->t2 - traj->t1;
    float d3 = traj->t3 - traj->t2;
    float d4 = traj->t4 - traj->t3;
    float d5 = traj->t5 - traj->t4;
    float d6 = traj->t6 - traj->t5;
    float d7 = traj->t7 - traj->t6;

    /* ── Phase 1 start: at rest ───────────────────────────────────────── */
    traj->p[0] = traj->theta_0;
    traj->v[0] = 0.0f;
    traj->a[0] = 0.0f;

    /* ── Phase 1 end = Phase 2 start (jerk = +jmax·dir) ──────────────── */
    float j1 = +jmax * dir;
    traj->p[1] = traj->p[0] + traj->v[0]*d1 + 0.5f*traj->a[0]*d1*d1 + (1.0f/6.0f)*j1*d1*d1*d1;
    traj->v[1] = traj->v[0] + traj->a[0]*d1 + 0.5f*j1*d1*d1;
    traj->a[1] = traj->a[0] + j1*d1;

    /* ── Phase 2 end = Phase 3 start (jerk = 0) ──────────────────────── */
    traj->p[2] = traj->p[1] + traj->v[1]*d2 + 0.5f*traj->a[1]*d2*d2;
    traj->v[2] = traj->v[1] + traj->a[1]*d2;
    traj->a[2] = traj->a[1];

    /* ── Phase 3 end = Phase 4 start (jerk = -jmax·dir) ─────────────── */
    float j3 = -jmax * dir;
    traj->p[3] = traj->p[2] + traj->v[2]*d3 + 0.5f*traj->a[2]*d3*d3 + (1.0f/6.0f)*j3*d3*d3*d3;
    traj->v[3] = traj->v[2] + traj->a[2]*d3 + 0.5f*j3*d3*d3;
    traj->a[3] = traj->a[2] + j3*d3;

    /* ── Phase 4 end = Phase 5 start (jerk = 0, cruise) ─────────────── */
    traj->p[4] = traj->p[3] + traj->v[3]*d4;
    traj->v[4] = traj->v[3];
    traj->a[4] = 0.0f;

    /* ── Phase 5 end = Phase 6 start (jerk = -jmax·dir) ─────────────── */
    float j5 = -jmax * dir;
    traj->p[5] = traj->p[4] + traj->v[4]*d5 + 0.5f*traj->a[4]*d5*d5 + (1.0f/6.0f)*j5*d5*d5*d5;
    traj->v[5] = traj->v[4] + traj->a[4]*d5 + 0.5f*j5*d5*d5;
    traj->a[5] = traj->a[4] + j5*d5;

    /* ── Phase 6 end = Phase 7 start (jerk = 0) ──────────────────────── */
    traj->p[6] = traj->p[5] + traj->v[5]*d6 + 0.5f*traj->a[5]*d6*d6;
    traj->v[6] = traj->v[5] + traj->a[5]*d6;
    traj->a[6] = traj->a[5];

    /* Phase 7 final check (jerk = +jmax·dir) should arrive at theta_f, v=0, a=0 */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private: plan_scurve
 *
 *  Computes t1..t7 and calls compute_phases().
 *  Handles all 6 Biagiotti-Melchiorri patterns.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void plan_scurve(Trajectory_t *traj, float theta_0, float theta_f)
{
    const float vmax = traj->omega_max;
    const float amax = traj->alpha_max;
    const float jmax = traj->jerk_max;

    traj->theta_0 = theta_0;
    traj->theta_f = theta_f;

    float delta = theta_f - theta_0;
    traj->dir = (delta >= 0.0f) ? 1.0f : -1.0f;
    float s   = fabsf(delta);

    float tj, ta, tv;

    if (s < 1e-6f)
    {
        /* Trivial move — stay put */
        traj->t1 = traj->t2 = traj->t3 = 0.0f;
        traj->t4 = traj->t5 = traj->t6 = traj->t7 = TRAJ_TF_MIN;
        traj->T_f = TRAJ_TF_MIN;
        traj->theta_ref = theta_0;
        traj->omega_ref = 0.0f;
        traj->alpha_ref = 0.0f;
        traj->t         = 0.0f;
        traj->active    = 0;
        return;
    }

    /* Derived scalar thresholds */
    float tj_nom = amax / jmax;             /* jerk ramp duration at full amax  */
    float sa = 2.0f * (amax*amax*amax) / (jmax*jmax);  /* min s to reach amax  */
    float sv = vmax * (vmax / amax + amax / jmax);      /* min s to reach vmax  */

    /* Pattern selection (Biagiotti & Melchiorri, simplified):
     *
     *   s >= sv  → Pattern 5: full 7-segment (cruise + const-accel + jerk ramps)
     *   s >= sa  → Pattern 6: no cruise, but has const-accel phase
     *   s <  sa  → Pattern 2/3/4: jerk-only triangular ramps (no const-accel, no cruise)
     *
     * The gate is s vs sa/sv — NOT vmax vs va.
     * For any s < sa the distance is too short to reach amax, so jerk-only ramps suffice.
     */
    if (s >= sv)
    {
        /* Pattern 5: full 7-segment — jerk ramps + const-accel + cruise */
        tj = tj_nom;
        ta = vmax / amax;
        tv = s / vmax;
    }
    else if (s >= sa)
    {
        /* Pattern 6: no cruise — jerk ramps + const-accel only
         *
         * Total distance: s = amax·ta·(ta + tj)
         * Solving for ta: amax·ta² + amax·tj·ta − s = 0
         *   ta = (−tj + sqrt(tj² + 4·s/amax)) / 2
         */
        tj = tj_nom;
        ta = 0.5f * (-tj + sqrtf(tj * tj + 4.0f * s / amax));
        if (ta < tj) ta = tj;   /* safety clamp — should not trigger for s >= sa */
        tv = ta + tj;           /* t4 = t3: zero cruise duration */
    }
    else
    {
        /* Pattern 2/3/4: jerk-only triangular ramps — no const-accel, no cruise
         *
         * Total distance: s = 2·jmax·tj³
         * Solving for tj: tj = (s / (2·jmax))^(1/3)
         * (ta = tj → zero const-accel,  tv = 2·tj → zero cruise)
         */
        tj = cbrtf(s / (2.0f * jmax));
        ta = tj;
        tv = 2.0f * tj;
    }

    /* Timestamps */
    traj->t1 = tj;
    traj->t2 = ta;
    traj->t3 = ta + tj;
    traj->t4 = tv;
    traj->t5 = tv + tj;
    traj->t6 = tv + ta;
    traj->t7 = tv + ta + tj;

    /* Floor total time */
    if (traj->t7 < TRAJ_TF_MIN) traj->t7 = TRAJ_TF_MIN;
    traj->T_f = traj->t7;

    /* Pre-compute per-phase boundary states */
    compute_phases(traj);

    /* Arm */
    traj->theta_ref = theta_0;
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
                     float alpha_max,
                     float jerk_max)
{
    memset(traj, 0, sizeof(Trajectory_t));
    traj->dt        = dt;
    traj->omega_max = omega_max;
    traj->alpha_max = alpha_max;
    traj->jerk_max  = jerk_max;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_ComputeTf
 *
 *  Returns approximate minimum Tf that respects all three constraints.
 *  Used externally for timeout planning.
 * ═══════════════════════════════════════════════════════════════════════════ */
float Trajectory_ComputeTf(const Trajectory_t *traj, float dtheta)
{
    float s    = fabsf(dtheta);
    float vmax = traj->omega_max;
    float amax = traj->alpha_max;
    float jmax = traj->jerk_max;

    if (s < 1e-6f) return TRAJ_TF_MIN;

    float tj = amax / jmax;
    float ta = vmax / amax;
    float tv = s / vmax;

    float T_f = tv + ta + tj;   /* pattern 5 nominal (upper bound) */
    return (T_f > TRAJ_TF_MIN) ? T_f : TRAJ_TF_MIN;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_SetTargetConstrained  (primary API)
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_SetTargetConstrained(Trajectory_t *traj,
                                     float theta_0,
                                     float theta_f)
{
    plan_scurve(traj, theta_0, theta_f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_SetTarget  (manual Tf — legacy / homing)
 *
 *  For homing we don't need the full S-curve planning — we just need a
 *  well-defined move.  Uses the S-curve planner but ignores T_f argument;
 *  the planner finds the fastest feasible profile.
 * ═══════════════════════════════════════════════════════════════════════════ */
void Trajectory_SetTarget(Trajectory_t *traj,
                          float theta_0,
                          float theta_f,
                          float T_f)
{
    (void)T_f;   /* S-curve self-determines duration */
    plan_scurve(traj, theta_0, theta_f);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trajectory_Update
 *
 *  Evaluates the S-curve at the current time t, then advances by dt.
 *  Each phase uses exact kinematics from its pre-computed boundary state.
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

    /* Determine which phase we're in and compute local time τ */
    float tau;
    float p_i, v_i, a_i, j_i;
    const float jmax = traj->jerk_max;
    const float dir  = traj->dir;

    if (t <= traj->t1)
    {
        tau = t;
        p_i = traj->p[0];  v_i = traj->v[0];  a_i = traj->a[0];
        j_i = +jmax * dir;
    }
    else if (t <= traj->t2)
    {
        tau = t - traj->t1;
        p_i = traj->p[1];  v_i = traj->v[1];  a_i = traj->a[1];
        j_i = 0.0f;
    }
    else if (t <= traj->t3)
    {
        tau = t - traj->t2;
        p_i = traj->p[2];  v_i = traj->v[2];  a_i = traj->a[2];
        j_i = -jmax * dir;
    }
    else if (t <= traj->t4)
    {
        tau = t - traj->t3;
        p_i = traj->p[3];  v_i = traj->v[3];  a_i = traj->a[3];
        j_i = 0.0f;
    }
    else if (t <= traj->t5)
    {
        tau = t - traj->t4;
        p_i = traj->p[4];  v_i = traj->v[4];  a_i = traj->a[4];
        j_i = -jmax * dir;
    }
    else if (t <= traj->t6)
    {
        tau = t - traj->t5;
        p_i = traj->p[5];  v_i = traj->v[5];  a_i = traj->a[5];
        j_i = 0.0f;
    }
    else
    {
        tau = t - traj->t6;
        p_i = traj->p[6];  v_i = traj->v[6];  a_i = traj->a[6];
        j_i = +jmax * dir;
    }

    float tau2 = tau * tau;
    float tau3 = tau2 * tau;

    traj->theta_ref = p_i + v_i*tau + 0.5f*a_i*tau2 + (1.0f/6.0f)*j_i*tau3;
    traj->omega_ref = v_i           + a_i*tau        + 0.5f*j_i*tau2;
    traj->alpha_ref = a_i                            + j_i*tau;

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
    float jerk_max  = traj->jerk_max;
    memset(traj, 0, sizeof(Trajectory_t));
    traj->dt        = dt;
    traj->omega_max = omega_max;
    traj->alpha_max = alpha_max;
    traj->jerk_max  = jerk_max;
}
