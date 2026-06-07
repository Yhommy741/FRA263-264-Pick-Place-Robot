/*
 * Controller.h
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * PID controller structs and API.
 * Cascaded velocity/position loop used by Robot.c.
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>
#include "DCMotor.h"

/* ════════════════════════════════════════════════════════════════════════════
 * Global Motor Constants
 * ════════════════════════════════════════════════════════════════════════════ */
#define TAU                0.001f   /* Motor Mechanical Time Constant \tau [s] */

/* ════════════════════════════════════════════════════════════════════════════
 * Control Mode Enum
 * ════════════════════════════════════════════════════════════════════════════ */
typedef enum {
    CTRL_MODE_VEL_PID        = 1,   /* Velocity PID only                     */
    CTRL_MODE_FF_REF         = 2,   /* Reference FF only                     */
    CTRL_MODE_FF_REF_DIST    = 3,   /* Ref FF + Disturbance FF               */
    CTRL_MODE_VEL_PID_FF_ALL = 4,   /* Vel PID + Ref FF + Dist FF            */
    CTRL_MODE_POS_PID        = 5,   /* Position PID only                     */
    CTRL_MODE_CASCADE_FF_ALL = 6    /* Cascade (Pos→Vel PID) + All FF        */
} ControlMode_t;

/* ════════════════════════════════════════════════════════════════════════════
 * PID_t  —  Velocity-form (incremental) PID
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float Kp, Ki, Kd;
    float Ts;                /* Sample period [s]                             */
    float out_min, out_max;  /* Output clamp limits                           */
    float e[3];              /* e[0]=e[k], e[1]=e[k-1], e[2]=e[k-2]          */
    float u_prev;            /* u[k-1]                                        */
} PID_t;

void  PID_Init  (PID_t *pid, float Kp, float Ki, float Kd, float Ts, float out_min, float out_max);
float PID_Update(PID_t *pid, float setpoint, float measured);
void  PID_Reset (PID_t *pid);

/* ════════════════════════════════════════════════════════════════════════════
 * FeedforwardController_t
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* Reference Feedforward Gain */
    float G_ff;         /* (Rm·b)/Kt + Ke   [V·s/rad]                          */

    /* Dynamic Disturbance Feedforward Gains (from Slide Equation) */
    float G_aff_static; /* Rm / Kt          [V/(N·m)]                          */
    float G_aff_dynamic;/* Lm / Kt          [V·s/(N·m)]                        */

    /* Memory states for numerical derivatives */
    float omega_prev;   /* omega_ref[k-1]                                      */
    float tau_d_prev;   /* tau_d[k-1]                                          */
} FeedforwardController_t;

void  FF_Init   (FeedforwardController_t *ff, const DCMotor_t *motor);
float FF_Compute(FeedforwardController_t *ff, float omega_ref, float tau_d, float Ts);

/* ════════════════════════════════════════════════════════════════════════════
 * Controller_t  —  Top-level control object
 * ════════════════════════════════════════════════════════════════════════════ */
typedef struct {
    ControlMode_t           ControlMode;
    PID_t                   pid_vel;
    PID_t                   pid_pos;
    FeedforwardController_t ff;

    uint32_t pos_loop_multi;   /* Outer PID runs every N inner ticks */
    uint32_t tick_counter;     /* Managed internally                 */

    float    max_omega;        /* Robot Max Velocity * Speed Ratio [rad/s]    */
    float    pos_deadband;     /* |pos error| below this → output 0, PIDs hold*/

    /* Diagnostics (read-only) */
    float u_pid;
    float u_ff;
    float u_total;
    float omega_cmd;
} Controller_t;

void Controller_Init(Controller_t    *ctrl,
                     const DCMotor_t *motor,
                     float Kp_vel, float Ki_vel, float Kd_vel,
                     float Kp_pos, float Ki_pos, float Kd_pos,
                     float Ts_vel,
                     uint32_t pos_loop_multi,
                     float max_omega,
                     float V_max,
                     float pos_deadband);

void Controller_Reset(Controller_t *ctrl);

float Controller_Update(Controller_t *ctrl,
                        float target_pos, float target_vel,
                        float pos,        float vel,
                        float disturbance);

static inline float Saturate(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

#endif /* INC_CONTROLLER_H_ */
