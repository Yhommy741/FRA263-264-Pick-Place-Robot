/*
 * DCMotor.h
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * DC motor physical parameter struct.
 * Electrical model : V = Rm·I + Lm·(dI/dt) + Ke·ω
 * Mechanical model : J·(dω/dt) = Kt·I − b·ω − τ_d
 */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

#include <stdint.h>

/* ════════════════════════════════════════════════════════════════════════════
 *  DCMotor_t
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {

    float Rm;   /* Armature resistance      [Ω]         */
    float Lm;   /* Armature inductance      [H]         */
    float Ke;   /* Back-EMF constant        [V·s/rad]   */
    float Kt;   /* Torque constant          [N·m/A]     */
    float J;    /* Rotor inertia            [kg·m²]     */
    float b;    /* Viscous friction coeff   [N·m·s/rad] */

} DCMotor_t;

/* ════════════════════════════════════════════════════════════════════════════
 *  API
 * ════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Fill a DCMotor_t with all physical parameters.
 *
 * @param  motor  Pointer to DCMotor_t instance
 * @param  Rm     Armature resistance      [Ω]
 * @param  Lm     Armature inductance      [H]
 * @param  Ke     Back-EMF constant        [V·s/rad]
 * @param  Kt     Torque constant          [N·m/A]
 * @param  J      Rotor inertia            [kg·m²]
 * @param  b      Viscous friction coeff   [N·m·s/rad]
 */
void DCMotor_Init(DCMotor_t *motor,
                  float Rm, float Lm,
                  float Ke, float Kt,
                  float J,  float b);

#endif /* INC_DCMOTOR_H_ */
