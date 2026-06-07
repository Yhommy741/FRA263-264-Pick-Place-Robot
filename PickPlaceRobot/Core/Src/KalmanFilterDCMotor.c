/*
 * KalmanFilterDCMotor.c
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * 4-state discrete Kalman filter for the DC motor (position-only observer).
 * State : [ theta  omega  I  tau_d ]
 * All matrix ops via CMSIS-DSP arm_mat_*; no scratch aliasing.
 */

#include "KalmanFilterDCMotor.h"
#include "DCMotor.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  init_wrappers — bind every CMSIS wrapper to its backing array ONCE
 * ═══════════════════════════════════════════════════════════════════════════ */
static void init_wrappers(KalmanFilterDCMotor_t *kf)
{
    arm_mat_init_f32(&kf->x,       KF_DCM_N, 1,          kf->x_d);
    arm_mat_init_f32(&kf->P,       KF_DCM_N, KF_DCM_N,   kf->P_d);

    arm_mat_init_f32(&kf->Fd,      KF_DCM_N, KF_DCM_N,   kf->Fd_d);
    arm_mat_init_f32(&kf->Q,       KF_DCM_N, KF_DCM_N,   kf->Q_d);
    arm_mat_init_f32(&kf->H,       KF_DCM_M, KF_DCM_N,   kf->H_d);
    arm_mat_init_f32(&kf->R,       KF_DCM_M, KF_DCM_M,   kf->R_d);

    arm_mat_init_f32(&kf->FdT,     KF_DCM_N, KF_DCM_N,   kf->FdT_d);
    arm_mat_init_f32(&kf->FdP,     KF_DCM_N, KF_DCM_N,   kf->FdP_d);
    arm_mat_init_f32(&kf->FdPFdT,  KF_DCM_N, KF_DCM_N,   kf->FdPFdT_d);
    arm_mat_init_f32(&kf->x_pred,  KF_DCM_N, 1,          kf->x_pred_d);
    arm_mat_init_f32(&kf->P_pred,  KF_DCM_N, KF_DCM_N,   kf->P_pred_d);

    arm_mat_init_f32(&kf->HT,      KF_DCM_N, KF_DCM_M,   kf->HT_d);
    arm_mat_init_f32(&kf->HP,      KF_DCM_M, KF_DCM_N,   kf->HP_d);
    arm_mat_init_f32(&kf->HPHt,    KF_DCM_M, KF_DCM_M,   kf->HPHt_d);
    arm_mat_init_f32(&kf->S,       KF_DCM_M, KF_DCM_M,   kf->S_d);
    arm_mat_init_f32(&kf->PHT,     KF_DCM_N, KF_DCM_M,   kf->PHT_d);
    arm_mat_init_f32(&kf->K,       KF_DCM_N, KF_DCM_M,   kf->K_d);
    arm_mat_init_f32(&kf->Hx,      KF_DCM_M, 1,          kf->Hx_d);
    arm_mat_init_f32(&kf->y,       KF_DCM_M, 1,          kf->y_d);
    arm_mat_init_f32(&kf->Ky,      KF_DCM_N, 1,          kf->Ky_d);
    arm_mat_init_f32(&kf->x_post,  KF_DCM_N, 1,          kf->x_post_d);
    arm_mat_init_f32(&kf->KH,      KF_DCM_N, KF_DCM_N,   kf->KH_d);
    arm_mat_init_f32(&kf->IKH,     KF_DCM_N, KF_DCM_N,   kf->IKH_d);
    arm_mat_init_f32(&kf->P_post,  KF_DCM_N, KF_DCM_N,   kf->P_post_d);
    arm_mat_init_f32(&kf->I,       KF_DCM_N, KF_DCM_N,   kf->I_d);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Matrix builders
 * ═══════════════════════════════════════════════════════════════════════════ */

static void build_Fd(KalmanFilterDCMotor_t *kf)
{
    float dt = kf->dt;
    float *F = kf->Fd_d;

    memset(F, 0, KF_DCM_N * KF_DCM_N * sizeof(float));

    F[0*KF_DCM_N + 0] = 1.0f;
    F[0*KF_DCM_N + 1] = dt;

    F[1*KF_DCM_N + 1] = 1.0f - (kf->motor->b  / kf->motor->J) * dt;
    F[1*KF_DCM_N + 2] =        (kf->motor->Kt / kf->motor->J) * dt;
    F[1*KF_DCM_N + 3] =        (1.0f          / kf->motor->J) * dt;

    F[2*KF_DCM_N + 1] = -(kf->motor->Ke / kf->motor->Lm) * dt;
    F[2*KF_DCM_N + 2] = 1.0f - (kf->motor->Rm / kf->motor->Lm) * dt;

    F[3*KF_DCM_N + 3] = 1.0f;
}

static void build_Gd(KalmanFilterDCMotor_t *kf)
{
    kf->Gd_d[0] = 0.0f;
    kf->Gd_d[1] = 0.0f;
    kf->Gd_d[2] = kf->dt / kf->motor->Lm;
    kf->Gd_d[3] = 0.0f;
}

static void build_Q(KalmanFilterDCMotor_t *kf)
{
    uint8_t i, j;
    float Dd[KF_DCM_N] = {0.0f, 0.0f, 0.0f, kf->dt};

    for (i = 0; i < KF_DCM_N; i++)
        for (j = 0; j < KF_DCM_N; j++)
            kf->Q_d[i * KF_DCM_N + j] = kf->var_tau_d * Dd[i] * Dd[j];
}

static void build_H(KalmanFilterDCMotor_t *kf)
{
    /* H = [ 1  0  0  0 ] — observe theta only */
    memset(kf->H_d, 0, KF_DCM_M * KF_DCM_N * sizeof(float));
    kf->H_d[0] = 1.0f;
}

static void build_R(KalmanFilterDCMotor_t *kf)
{
    /* R = [ var_theta ] — 1x1 scalar */
    kf->R_d[0] = kf->var_theta;
}

static void build_I(KalmanFilterDCMotor_t *kf)
{
    uint8_t i;
    memset(kf->I_d, 0, KF_DCM_N * KF_DCM_N * sizeof(float));
    for (i = 0; i < KF_DCM_N; i++)
        kf->I_d[i * KF_DCM_N + i] = 1.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  predict_and_update
 *
 *  S is 1x1 — Si_scalar = 1/S_d[0], no arm_mat_inverse_f32 needed.
 *  K  = PHT * Si_scalar  via arm_mat_scale_f32.
 *  Ky = K   * y_scalar   via arm_mat_scale_f32.
 * ═══════════════════════════════════════════════════════════════════════════ */
static void predict_and_update(KalmanFilterDCMotor_t *kf,
                                float u_Vin, float z_theta)
{
    uint8_t i;
    float   Si_scalar;

    /* ── PREDICT ──────────────────────────────────────────────────────── */

    arm_mat_mult_f32(&kf->Fd, &kf->x, &kf->x_pred);

    for (i = 0; i < KF_DCM_N; i++)
        kf->x_pred_d[i] += kf->Gd_d[i] * u_Vin;

    arm_mat_trans_f32(&kf->Fd,     &kf->FdT);
    arm_mat_mult_f32 (&kf->Fd,     &kf->P,      &kf->FdP);
    arm_mat_mult_f32 (&kf->FdP,    &kf->FdT,    &kf->FdPFdT);
    arm_mat_add_f32  (&kf->FdPFdT, &kf->Q,      &kf->P_pred);  /* dst != any src */

    memcpy(kf->x_d, kf->x_pred_d, KF_DCM_N            * sizeof(float));
    memcpy(kf->P_d, kf->P_pred_d, KF_DCM_N * KF_DCM_N * sizeof(float));

    /* ── UPDATE ───────────────────────────────────────────────────────── */

    arm_mat_trans_f32(&kf->H,      &kf->HT);
    arm_mat_mult_f32 (&kf->H,      &kf->x,      &kf->Hx);

    kf->y_d[0] = z_theta - kf->Hx_d[0];

    arm_mat_mult_f32(&kf->H,    &kf->P,   &kf->HP);
    arm_mat_mult_f32(&kf->HP,   &kf->HT,  &kf->HPHt);
    arm_mat_add_f32 (&kf->HPHt, &kf->R,   &kf->S);             /* dst != any src */

    Si_scalar = (kf->S_d[0] > 1e-12f) ? (1.0f / kf->S_d[0]) : 0.0f;

    arm_mat_mult_f32 (&kf->P,   &kf->HT,       &kf->PHT);
    arm_mat_scale_f32(&kf->PHT,  Si_scalar,     &kf->K);        /* K  = PHT / S  */
    arm_mat_scale_f32(&kf->K,    kf->y_d[0],   &kf->Ky);       /* Ky = K * y    */
    arm_mat_add_f32  (&kf->x,   &kf->Ky,       &kf->x_post);   /* dst != any src */

    arm_mat_mult_f32(&kf->K,   &kf->H,  &kf->KH);
    arm_mat_sub_f32 (&kf->I,   &kf->KH, &kf->IKH);             /* dst != any src */
    arm_mat_mult_f32(&kf->IKH, &kf->P,  &kf->P_post);

    memcpy(kf->x_d, kf->x_post_d, KF_DCM_N            * sizeof(float));
    memcpy(kf->P_d, kf->P_post_d, KF_DCM_N * KF_DCM_N * sizeof(float));
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void KalmanFilterDCMotor_Init(KalmanFilterDCMotor_t *kf,
                               DCMotor_t *motor)
{
    memset(kf, 0, sizeof(KalmanFilterDCMotor_t));

    /* Link shared motor parameter struct */
    kf->motor = motor;

    /* Bind all CMSIS wrappers to their backing arrays — done once here */
    init_wrappers(kf);

    /* H is constant — build now */
    build_H(kf);

    kf->started = 0;
}

void KalmanFilterDCMotor_Set_ObserverPeriod(KalmanFilterDCMotor_t *kf, float dt)
{
    kf->dt = dt;
    build_Fd(kf);
    build_Gd(kf);
    build_Q(kf);
}

void KalmanFilterDCMotor_Set_ProcessNoise(KalmanFilterDCMotor_t *kf, float var_tau_d)
{
    kf->var_tau_d = var_tau_d;
    build_Q(kf);
}

void KalmanFilterDCMotor_Set_MeasurementNoise(KalmanFilterDCMotor_t *kf, float var_theta)
{
    kf->var_theta = var_theta;
    build_R(kf);
}

void KalmanFilterDCMotor_Start(KalmanFilterDCMotor_t *kf,
                                float theta0, float omega0,
                                float I0, float taud0)
{
    uint8_t i;

    /* Rebuild all matrices (covers any skipped Set_* calls) */
    build_Fd(kf);
    build_Gd(kf);
    build_Q(kf);
    build_H(kf);
    build_R(kf);
    build_I(kf);

    kf->x_d[0] = theta0;
    kf->x_d[1] = omega0;
    kf->x_d[2] = I0;
    kf->x_d[3] = taud0;

    /* P = Identity */
    memset(kf->P_d, 0, KF_DCM_N * KF_DCM_N * sizeof(float));
    for (i = 0; i < KF_DCM_N; i++)
        kf->P_d[i * KF_DCM_N + i] = 1.0f;

    kf->started = 1;
}

void KalmanFilterDCMotor_Update(KalmanFilterDCMotor_t *kf,
                                 float z_theta, float u_Vin)
{
    predict_and_update(kf, u_Vin, z_theta);
}

/* ── Getters ────────────────────────────────────────────────────────────── */
float KalmanFilterDCMotor_Get_Position   (KalmanFilterDCMotor_t *kf) { return kf->x_d[0]; }
float KalmanFilterDCMotor_Get_Velocity   (KalmanFilterDCMotor_t *kf) { return kf->x_d[1]; }
float KalmanFilterDCMotor_Get_Current    (KalmanFilterDCMotor_t *kf) { return kf->x_d[2]; }
float KalmanFilterDCMotor_Get_Disturbance(KalmanFilterDCMotor_t *kf) { return kf->x_d[3]; }

/* ── Runtime re-tuning ──────────────────────────────────────────────────── */
void KalmanFilterDCMotor_Set_ObserverPeriod_Runtime(KalmanFilterDCMotor_t *kf, float dt)
{
    KalmanFilterDCMotor_Set_ObserverPeriod(kf, dt);
}
void KalmanFilterDCMotor_Set_ProcessNoise_Runtime(KalmanFilterDCMotor_t *kf, float var_tau_d)
{
    KalmanFilterDCMotor_Set_ProcessNoise(kf, var_tau_d);
}
