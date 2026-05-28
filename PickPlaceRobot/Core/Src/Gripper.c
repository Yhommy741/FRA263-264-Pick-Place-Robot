/*
 * Gripper.c
 *
 * Created on: May 24, 2026
 * Author: Yhommy's Notebook
 *
 * Non-blocking implementation — HAL_Delay() removed entirely.
 *
 * HOW IT WORKS:
 *   Each command sets the GPIO pin HIGH immediately and records
 *   HAL_GetTick() in pulse_start_ms.
 *
 *   Gripper_Update() must be called every main-loop iteration.
 *   It checks whether pulse_duration_ms has elapsed and resets
 *   the pin LOW when it has.  Only one pin is ever active at once;
 *   a new command cancels any in-progress pulse first.
 *
 *   The main loop therefore never stalls — Modbus_Protocol_Worker
 *   continues to be called on every iteration during gripper operation.
 */

#include "Gripper.h"
#include <stddef.h>

/* ── Private: start a non-blocking pulse ────────────────────────────────── */
static void Gripper_StartPulse(Gripper_t* gripper,
                                GPIO_TypeDef* port, uint16_t pin)
{
    if (gripper == NULL || port == NULL) return;

    /* Cancel any previous pulse first */
    if (gripper->pulse_active && gripper->active_port != NULL)
        HAL_GPIO_WritePin(gripper->active_port, gripper->active_pin, GPIO_PIN_RESET);

    /* Start new pulse */
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    gripper->active_port    = port;
    gripper->active_pin     = pin;
    gripper->pulse_start_ms = HAL_GetTick();
    gripper->pulse_active   = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_Init(Gripper_t* gripper,
                  GPIO_TypeDef* up_port_out,    uint16_t up_pin_out,
                  GPIO_TypeDef* down_port_out,  uint16_t down_pin_out,
                  GPIO_TypeDef* open_port_out,  uint16_t open_pin_out,
                  GPIO_TypeDef* close_port_out, uint16_t close_pin_out,
                  GPIO_TypeDef* up_port_in,     uint16_t up_pin_in,
                  GPIO_TypeDef* down_port_in,   uint16_t down_pin_in,
                  GPIO_TypeDef* claw_port_in,   uint16_t claw_pin_in)
{
    if (gripper == NULL) return;

    /* Outputs */
    gripper->up_port_out    = up_port_out;
    gripper->up_pin_out     = up_pin_out;
    gripper->down_port_out  = down_port_out;
    gripper->down_pin_out   = down_pin_out;
    gripper->open_port_out  = open_port_out;
    gripper->open_pin_out   = open_pin_out;
    gripper->close_port_out = close_port_out;
    gripper->close_pin_out  = close_pin_out;

    /* Inputs */
    gripper->up_port_in     = up_port_in;
    gripper->up_pin_in      = up_pin_in;
    gripper->down_port_in   = down_port_in;
    gripper->down_pin_in    = down_pin_in;
    gripper->claw_port_in   = claw_port_in;
    gripper->claw_pin_in    = claw_pin_in;

    /* Timing — default 100 ms pulse */
    gripper->pulse_duration_ms = 100;

    /* Pulse state */
    gripper->active_port    = NULL;
    gripper->active_pin     = 0;
    gripper->pulse_start_ms = 0;
    gripper->pulse_active   = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Gripper_Update  — call every main-loop iteration
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_Update(Gripper_t* gripper)
{
    if (gripper == NULL || !gripper->pulse_active) return;

    if (HAL_GetTick() - gripper->pulse_start_ms >= gripper->pulse_duration_ms)
    {
        HAL_GPIO_WritePin(gripper->active_port, gripper->active_pin, GPIO_PIN_RESET);
        gripper->active_port  = NULL;
        gripper->active_pin   = 0;
        gripper->pulse_active = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Command Functions — non-blocking pulse start
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_MoveUp(Gripper_t* gripper)
{
    if (gripper == NULL) return;
    Gripper_StartPulse(gripper, gripper->up_port_out, gripper->up_pin_out);
}

void Gripper_MoveDown(Gripper_t* gripper)
{
    if (gripper == NULL) return;
    Gripper_StartPulse(gripper, gripper->down_port_out, gripper->down_pin_out);
}

void Gripper_Open(Gripper_t* gripper)
{
    if (gripper == NULL) return;
    Gripper_StartPulse(gripper, gripper->open_port_out, gripper->open_pin_out);
}

void Gripper_Close(Gripper_t* gripper)
{
    if (gripper == NULL) return;
    Gripper_StartPulse(gripper, gripper->close_port_out, gripper->close_pin_out);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Sensor Getters
 * ═══════════════════════════════════════════════════════════════════════════ */
GripperState_t Gripper_Up_State(Gripper_t* gripper)
{
    if (gripper == NULL || gripper->up_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->up_port_in, gripper->up_pin_in) == GPIO_PIN_SET)
           ? GRP_STATE_HIGH : GRP_STATE_LOW;
}

GripperState_t Gripper_Down_State(Gripper_t* gripper)
{
    if (gripper == NULL || gripper->down_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->down_port_in, gripper->down_pin_in) == GPIO_PIN_SET)
           ? GRP_STATE_HIGH : GRP_STATE_LOW;
}

GripperState_t Gripper_Claw_State(Gripper_t* gripper)
{
    if (gripper == NULL || gripper->claw_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->claw_port_in, gripper->claw_pin_in) == GPIO_PIN_SET)
           ? GRP_STATE_HIGH : GRP_STATE_LOW;
}
