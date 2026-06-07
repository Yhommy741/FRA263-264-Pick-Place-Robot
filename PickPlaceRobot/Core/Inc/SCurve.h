/*
 * SCurve.h
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * 7-segment S-curve (jerk-limited) trajectory generator interface.
 * Phases: +J / const-A / -J / cruise / -J / const-D / +J.
 * Boundary conditions: θ̇(0)=0, θ̈(0)=0, θ̇(T)=0, θ̈(T)=0.
 */

#ifndef INC_SCURVE_H_
#define INC_SCURVE_H_

#include <stdint.h>
#include <string.h>
#include <math.h>

/* ── Minimum allowed duration [s] ────────────────────────────────────────── */
#define SCURVE_TF_MIN   0.05f
#define TRAJ_TF_MIN     SCURVE_TF_MIN   /* legacy alias */

/* ═══════════════════════════════════════════════════════════════════════════
 *  SCurve_t handle
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* Constraints */
    float omega_max;
    float alpha_max;
    float jerk_max;

    /* Timing */
    float dt;
    float T_f;

    /* 7-segment timestamps */
    float t1, t2, t3, t4, t5, t6, t7;

    /* Direction */
    float dir;

    /* Start / end */
    float theta_0;
    float theta_f;

    /* Per-phase boundary states [0..6] = phases 1..7 */
    float p[7];
    float v[7];
    float a[7];

    /* Internal time counter */
    float t;

    /* Outputs — read after SCurve_Update() */
    float theta_ref;
    float omega_ref;
    float alpha_ref;

    /* Status */
    uint8_t active;

} SCurve_t;

/* Legacy typedef alias — files still using Trajectory_t compile unchanged */
typedef SCurve_t Trajectory_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  API
 * ═══════════════════════════════════════════════════════════════════════════ */

void  SCurve_Init     (SCurve_t *sc, float dt,
                        float omega_max, float alpha_max, float jerk_max);
void  SCurve_SetTarget(SCurve_t *sc, float theta_0, float theta_f);
void  SCurve_Update   (SCurve_t *sc);
void  SCurve_Stop     (SCurve_t *sc);
void  SCurve_Reset    (SCurve_t *sc);
float SCurve_ComputeTf(const SCurve_t *sc, float dtheta);

/* ── Legacy macros — keep Trajectory_* callers compiling without changes ─── *
 * Using macros (not static inline) avoids per-TU instantiation warnings.     */
#define Trajectory_Init(sc,dt,om,al,jk)             SCurve_Init((sc),(dt),(om),(al),(jk))
#define Trajectory_SetTargetConstrained(sc,t0,tf)   SCurve_SetTarget((sc),(t0),(tf))
#define Trajectory_SetTarget(sc,t0,tf,Tf)           SCurve_SetTarget((sc),(t0),(tf))
#define Trajectory_Update(sc)                        SCurve_Update(sc)
#define Trajectory_Stop(sc)                          SCurve_Stop(sc)
#define Trajectory_Reset(sc)                         SCurve_Reset(sc)
#define Trajectory_ComputeTf(sc,dtheta)              SCurve_ComputeTf((sc),(dtheta))

#endif /* INC_SCURVE_H_ */
