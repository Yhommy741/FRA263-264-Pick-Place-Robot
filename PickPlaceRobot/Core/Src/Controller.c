/*
 * Controller.c
 *
 * Created : May 2026
 * Author  : FRA263/264 Group 5
 */

#include "Controller.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * PID Implementation
 * ═══════════════════════════════════════════════════════════════════════════ */

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float Ts, float out_min, float out_max)
{
    pid->Kp      = Kp;
    pid->Ki      = Ki;
    pid->Kd      = Kd;
    pid->Ts      = Ts;
    pid->out_min = out_min;
    pid->out_max = out_max;
    PID_Reset(pid);
}

float PID_Update(PID_t *pid, float setpoint, float measured)
{
    pid->e[2] = pid->e[1];
    pid->e[1] = pid->e[0];
    pid->e[0] = setpoint - measured;

    float du =   pid->Kp * (pid->e[0] - pid->e[1])
               + pid->Ki * pid->Ts   * pid->e[0]
               + (pid->Kd / pid->Ts)  * (pid->e[0] - 2.0f * pid->e[1] + pid->e[2]);

    float u = Saturate(pid->u_prev + du, pid->out_min, pid->out_max);
    pid->u_prev = u;
    return u;
}

void PID_Reset(PID_t *pid)
{
    pid->e[0]   = 0.0f;
    pid->e[1]   = 0.0f;
    pid->e[2]   = 0.0f;
    pid->u_prev = 0.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Feedforward Implementation
 * ═══════════════════════════════════════════════════════════════════════════ */

void FF_Init(FeedforwardController_t *ff, const DCMotor_t *motor)
{
    ff->G_ff            = (motor->Rm * motor->b / motor->Kt) + motor->Ke;
    ff->G_aff_static    = motor->Rm / motor->Kt;
    ff->G_aff_dynamic   = motor->Lm / motor->Kt;
    ff->omega_prev      = 0.0f;
    ff->tau_d_prev      = 0.0f;
}

float FF_Compute(FeedforwardController_t *ff, float omega_ref, float tau_d, float Ts)
{
    float alpha_ref = 0.0f;
    float d_tau_d   = 0.0f;

    if (Ts > 0.0f) {
        alpha_ref = (omega_ref - ff->omega_prev) / Ts;
        d_tau_d   = (tau_d - ff->tau_d_prev) / Ts;
    }
    ff->omega_prev = omega_ref;
    ff->tau_d_prev = tau_d;

    // 1. Reference Feedforward Target using TAU macro
    float u_ff_ref  = ff->G_ff * (omega_ref + (TAU * alpha_ref));

    // 2. Dynamic Disturbance Compensation Voltage V[k] from slide equation
    float u_ff_dist = (ff->G_aff_dynamic * d_tau_d) + (ff->G_aff_static * tau_d);

    // Subtracting the disturbance voltage to isolate external load injections
    return u_ff_ref - u_ff_dist;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Controller Top Level Implementation
 * ═══════════════════════════════════════════════════════════════════════════ */

void Controller_Init(Controller_t    *ctrl,
                     const DCMotor_t *motor,
                     float Kp_vel, float Ki_vel, float Kd_vel,
                     float Kp_pos, float Ki_pos, float Kd_pos,
                     float Ts_vel,
                     uint32_t pos_loop_multi,
                     float max_omega,
                     float V_max)
{
    ctrl->ControlMode    = CTRL_MODE_VEL_PID;
    ctrl->pos_loop_multi = (pos_loop_multi > 0U) ? pos_loop_multi : 1U;
    ctrl->tick_counter   = 0U;
    ctrl->max_omega      = max_omega; // Clamps combined cascade targets

    // Velocity Loop output is bounded by physical Hardware Voltage Limit (V_max)
    PID_Init(&ctrl->pid_vel, Kp_vel, Ki_vel, Kd_vel, Ts_vel, -V_max, V_max);

    // Position Loop output maps to velocity command space bounded by max_omega
    float Ts_pos = Ts_vel * (float)ctrl->pos_loop_multi;
    PID_Init(&ctrl->pid_pos, Kp_pos, Ki_pos, Kd_pos, Ts_pos, -max_omega, max_omega);

    FF_Init(&ctrl->ff, motor);

    Controller_Reset(ctrl);
}

void Controller_Reset(Controller_t *ctrl)
{
    PID_Reset(&ctrl->pid_vel);
    PID_Reset(&ctrl->pid_pos);
    ctrl->ff.omega_prev = 0.0f;
    ctrl->ff.tau_d_prev = 0.0f;
    ctrl->tick_counter  = 0U;
    ctrl->u_pid         = 0.0f;
    ctrl->u_ff          = 0.0f;
    ctrl->u_total       = 0.0f;
    ctrl->omega_cmd     = 0.0f;
}

float Controller_Update(Controller_t *ctrl,
                        float target_pos, float target_vel,
                        float pos,        float vel,
                        float disturbance)
{
    float u     = 0.0f;
    float u_pid = 0.0f;
    float u_ff  = 0.0f;

    float Ts = ctrl->pid_vel.Ts;

    switch (ctrl->ControlMode)
    {
        case CTRL_MODE_VEL_PID:
            u_pid = PID_Update(&ctrl->pid_vel, target_vel, vel);
            u     = u_pid;
            break;

        case CTRL_MODE_FF_REF:
            u_ff  = FF_Compute(&ctrl->ff, target_vel, 0.0f, Ts);
            u     = u_ff;
            break;

        case CTRL_MODE_FF_REF_DIST:
            u_ff  = FF_Compute(&ctrl->ff, target_vel, disturbance, Ts);
            u     = u_ff;
            break;

        case CTRL_MODE_VEL_PID_FF_ALL:
            u_pid = PID_Update(&ctrl->pid_vel, target_vel, vel);
            u_ff  = FF_Compute(&ctrl->ff, target_vel, disturbance, Ts);
            u     = u_pid + u_ff;
            break;

        case CTRL_MODE_POS_PID:
            if (ctrl->tick_counter > ctrl->pos_loop_multi) {
                u_pid = PID_Update(&ctrl->pid_pos, target_pos, pos);
                ctrl->tick_counter = 1U;
            } else {
                u_pid = ctrl->pid_pos.u_prev;
                ctrl->tick_counter++;
            }
            u = u_pid;
            break;

        case CTRL_MODE_CASCADE_FF_ALL:
            /* 1. Update the outer Position loop at downsampled multi-rate */
            if (ctrl->tick_counter > ctrl->pos_loop_multi) {
                ctrl->omega_cmd = PID_Update(&ctrl->pid_pos, target_pos, pos);
                ctrl->tick_counter = 1U;
            } else {
                ctrl->tick_counter++;
            }

            /* 2. Combine tracking paths: Target Velocity + Position Loop Output Offset */
            float combined_vel_target = target_vel + ctrl->omega_cmd;

            /* 3. Clamp explicitly with Robot Max Velocity scaled constraint */
            combined_vel_target = Saturate(combined_vel_target, -ctrl->max_omega, ctrl->max_omega);

            /* 4. Drive Inner Loops with unified clamped target velocity */
            u_pid = PID_Update(&ctrl->pid_vel, combined_vel_target, vel);
            u_ff  = FF_Compute(&ctrl->ff, combined_vel_target, disturbance, Ts);
            u     = u_pid + u_ff;
            break;

        default:
            u = 0.0f;
            break;
    }

    // Final Voltage Ceiling Clamp
    u = Saturate(u, ctrl->pid_vel.out_min, ctrl->pid_vel.out_max);

    ctrl->u_pid   = u_pid;
    ctrl->u_ff    = u_ff;
    ctrl->u_total = u;

    return u;
}
