/*
 * Trapezoid.h
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * 3-phase trapezoidal trajectory generator interface (position-triggered).
 * Trapezoidal profile when v²/a ≤ stroke; triangular otherwise.
 * Returns vel_ref each tick; decel entry keyed on remaining distance.
 */

#ifndef INC_TRAPEZOID_H_
#define INC_TRAPEZOID_H_

#include <stdint.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Trapezoid handle
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {

    /* Constraints */
    float omega_max;        /* Default peak velocity limit  [rad/s]   */
    float alpha_max;        /* Default peak accel limit     [rad/s²]  */

    /* Timing */
    float dt;               /* Slow-loop sample period      [s]       */
    float T_f;              /* Estimated total duration     [s]       */

    /* Profile parameters — computed at plan time */
    float v_cruise;         /* Actual peak velocity [rad/s]           */
    float a_used;           /* Actual acceleration  [rad/s²]          */
    float s_ramp;           /* Ramp distance: v²/(2a)  [rad]          */
    float stroke;           /* Total displacement magnitude [rad]     */
    float dir;              /* +1.0 (forward) or -1.0 (backward)     */

    /* Start / end */
    float theta_0;          /* Position at plan time [rad]            */
    float theta_f;          /* Target position       [rad]            */

    /* Reference outputs — updated each tick by Trapezoid_Update()   */
    float theta_ref;        /* [rad]    */
    float omega_ref;        /* [rad/s]  — used as velocity command    */
    float alpha_ref;        /* [rad/s²] — informational               */

    /* Status */
    uint8_t active;         /* 1 = running, 0 = finished              */
    uint8_t is_triangular;  /* 1 = triangular, 0 = trapezoidal        */
    uint8_t phase;          /* 0=accel, 1=cruise, 2=decel, 3=done     */

} Trapezoid_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  API
 * ═══════════════════════════════════════════════════════════════════════════ */

void  Trapezoid_Init               (Trapezoid_t *trap, float dt,
                                    float omega_max, float alpha_max);

void  Trapezoid_SetTarget          (Trapezoid_t *trap,
                                    float theta_0, float theta_f);

void  Trapezoid_SetTargetConstrained(Trapezoid_t *trap,
                                     float theta_0, float theta_f,
                                     float omega_max, float alpha_max);

/**
 * @brief  Advance one slow-loop tick given the ACTUAL current position.
 *
 *         Position-triggered phase transitions:
 *           ACCEL  → CRUISE when dist_travelled >= s_ramp
 *           CRUISE → DECEL  when dist_remaining <= s_ramp
 *           DECEL  → DONE   when dist_remaining <= 0
 *
 * @param  trap         Handle
 * @param  theta_actual Live robot position [rad] from Robot_GetPosition()
 */
void  Trapezoid_Update(Trapezoid_t *trap, float theta_actual);

void  Trapezoid_Stop (Trapezoid_t *trap);
void  Trapezoid_Reset(Trapezoid_t *trap);
float Trapezoid_ComputeTf(const Trapezoid_t *trap, float dtheta);

#endif /* INC_TRAPEZOID_H_ */
