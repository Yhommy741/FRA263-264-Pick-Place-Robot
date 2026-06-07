/*
 * KalmanFilterDCMotor.h
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * 4-state discrete Kalman filter interface — DC motor observer.
 * Persistent arm_matrix_instance_f32 wrappers; no re-init per tick.
 * Multi-step init: _Init → Set_ObserverPeriod → Set_ProcessNoise
 *                  → Set_MeasurementNoise → _Start
 */

#ifndef INC_KALMANFILTERDCMOTOR_H_
#define INC_KALMANFILTERDCMOTOR_H_

#include "arm_math.h"
#include "DCMotor.h"
#include <stdint.h>
#include <string.h>

/* ── Fixed dimensions ────────────────────────────────────────────────────── */
#define KF_DCM_N  4
#define KF_DCM_M  1   /* position only */



/* ════════════════════════════════════════════════════════════════════════════
 *  Handle
 *
 *  Each matrix has:
 *    float                   <name>_d[]  — flat data array
 *    arm_matrix_instance_f32 <name>      — persistent CMSIS wrapper
 *
 *  Wrappers are bound once in _Start() via arm_mat_init_f32().
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {

    /* ── Motor parameters (pointer to shared DCMotor_t) ─────────────── */
    DCMotor_t *motor;

    /* ── Convenience aliases (read directly from motor->*) ───────────── */
    /* Access via: kf->motor->Rm, kf->motor->Kt, etc.                    */

    /* Timing & noise variances */
    float dt;
    float var_tau_d;   /* disturbance variance  [N·m]^2 */
    float var_theta;   /* encoder variance      [rad]^2 */

    /* ── State  x (4x1) ─────────────────────────────────────────────── */
    float                   x_d[KF_DCM_N];
    arm_matrix_instance_f32 x;

    /* ── Error covariance  P (4x4) ──────────────────────────────────── */
    float                   P_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 P;

    /* ── Model matrices ─────────────────────────────────────────────── */
    float                   Fd_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 Fd;

    float                   Gd_d[KF_DCM_N];

    float                   Q_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 Q;

    float                   H_d[KF_DCM_M * KF_DCM_N];   /* 1x4 */
    arm_matrix_instance_f32 H;

    float                   R_d[KF_DCM_M * KF_DCM_M];   /* 1x1 */
    arm_matrix_instance_f32 R;

    /* ── Predict intermediates ──────────────────────────────────────── */
    float                   FdT_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 FdT;

    float                   FdP_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 FdP;

    float                   FdPFdT_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 FdPFdT;

    float                   x_pred_d[KF_DCM_N];
    arm_matrix_instance_f32 x_pred;

    float                   P_pred_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 P_pred;

    /* ── Update intermediates ───────────────────────────────────────── */
    float                   HT_d[KF_DCM_N * KF_DCM_M];   /* 4x1 */
    arm_matrix_instance_f32 HT;

    float                   HP_d[KF_DCM_M * KF_DCM_N];   /* 1x4 */
    arm_matrix_instance_f32 HP;

    float                   HPHt_d[KF_DCM_M * KF_DCM_M]; /* 1x1 */
    arm_matrix_instance_f32 HPHt;

    float                   S_d[KF_DCM_M * KF_DCM_M];    /* 1x1 */
    arm_matrix_instance_f32 S;

    float                   PHT_d[KF_DCM_N * KF_DCM_M];  /* 4x1 */
    arm_matrix_instance_f32 PHT;

    float                   K_d[KF_DCM_N * KF_DCM_M];    /* 4x1 */
    arm_matrix_instance_f32 K;

    float                   Hx_d[KF_DCM_M];
    arm_matrix_instance_f32 Hx;

    float                   y_d[KF_DCM_M];
    arm_matrix_instance_f32 y;

    float                   Ky_d[KF_DCM_N];
    arm_matrix_instance_f32 Ky;

    float                   x_post_d[KF_DCM_N];
    arm_matrix_instance_f32 x_post;

    float                   KH_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 KH;

    float                   IKH_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 IKH;

    float                   P_post_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 P_post;

    float                   I_d[KF_DCM_N * KF_DCM_N];
    arm_matrix_instance_f32 I;

    uint8_t started;

} KalmanFilterDCMotor_t;

/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

/** Step 1 — link motor parameters, bind CMSIS wrappers, build H */
void KalmanFilterDCMotor_Init(KalmanFilterDCMotor_t *kf,
                               DCMotor_t *motor);

/** Step 2 — set sample period, rebuild Fd / Gd / Q */
void KalmanFilterDCMotor_Set_ObserverPeriod(KalmanFilterDCMotor_t *kf,
                                             float dt);

/** Step 3 — set disturbance variance, rebuild Q */
void KalmanFilterDCMotor_Set_ProcessNoise(KalmanFilterDCMotor_t *kf,
                                           float var_tau_d);

/** Step 4 — set encoder variance, rebuild R */
void KalmanFilterDCMotor_Set_MeasurementNoise(KalmanFilterDCMotor_t *kf,
                                               float var_theta);

/** Step 5 — rebuild all matrices, set initial state, arm filter */
void KalmanFilterDCMotor_Start(KalmanFilterDCMotor_t *kf,
                                float theta0, float omega0,
                                float I0,     float taud0);

/** Call every tick from timer ISR */
void KalmanFilterDCMotor_Update(KalmanFilterDCMotor_t *kf,
                                 float z_theta, float u_Vin);

/* ── Getters ─────────────────────────────────────────────────────────────── */
float KalmanFilterDCMotor_Get_Position   (KalmanFilterDCMotor_t *kf); /* [rad]   */
float KalmanFilterDCMotor_Get_Velocity   (KalmanFilterDCMotor_t *kf); /* [rad/s] */
float KalmanFilterDCMotor_Get_Current    (KalmanFilterDCMotor_t *kf); /* [A]     */
float KalmanFilterDCMotor_Get_Disturbance(KalmanFilterDCMotor_t *kf); /* [N·m]   */

/* ── Runtime re-tuning ───────────────────────────────────────────────────── */
void KalmanFilterDCMotor_Set_ObserverPeriod_Runtime(KalmanFilterDCMotor_t *kf, float dt);
void KalmanFilterDCMotor_Set_ProcessNoise_Runtime  (KalmanFilterDCMotor_t *kf, float var_tau_d);

#endif /* INC_KALMANFILTERDCMOTOR_H_ */
