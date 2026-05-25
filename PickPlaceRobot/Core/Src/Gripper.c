/*
 * Gripper.c
 *
 * Created on: May 24, 2026
 * Author: Yhommy's Notebook
 */

#include "Gripper.h"
#include <stddef.h>

/**
 * @brief  Initializes the gripper structure with explicit port and pin configuration
 * @param  gripper: Pointer to the gripper instance
 */
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

    /* Assign Outputs */
    gripper->up_port_out    = up_port_out;
    gripper->up_pin_out     = up_pin_out;

    gripper->down_port_out  = down_port_out;
    gripper->down_pin_out   = down_pin_out;

    gripper->open_port_out  = open_port_out;
    gripper->open_pin_out   = open_pin_out;

    gripper->close_port_out = close_port_out;
    gripper->close_pin_out  = close_pin_out;

    /* Assign Inputs */
    gripper->up_port_in     = up_port_in;
    gripper->up_pin_in      = up_pin_in;

    gripper->down_port_in   = down_port_in;
    gripper->down_pin_in    = down_pin_in;

    gripper->claw_port_in   = claw_port_in;
    gripper->claw_pin_in    = claw_pin_in;

    /* Default Pulse Width (100ms) */
    gripper->pulse_duration_ms = 100;
}

/* ── Private Helper Function ─────────────────────────────────────────────── */
static void Gripper_PulsePin(GPIO_TypeDef* port, uint16_t pin, uint32_t duration) {
    if (port == NULL) return;

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    HAL_Delay(duration);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/* ── Actuator Commands ───────────────────────────────────────────────────── */

void Gripper_MoveUp(Gripper_t* gripper) {
    if (gripper == NULL) return;
    Gripper_PulsePin(gripper->up_port_out, gripper->up_pin_out, gripper->pulse_duration_ms);
}

void Gripper_MoveDown(Gripper_t* gripper) {
    if (gripper == NULL) return;
    Gripper_PulsePin(gripper->down_port_out, gripper->down_pin_out, gripper->pulse_duration_ms);
}

void Gripper_Open(Gripper_t* gripper) {
    if (gripper == NULL) return;
    Gripper_PulsePin(gripper->open_port_out, gripper->open_pin_out, gripper->pulse_duration_ms);
}

void Gripper_Close(Gripper_t* gripper) {
    if (gripper == NULL) return;
    Gripper_PulsePin(gripper->close_port_out, gripper->close_pin_out, gripper->pulse_duration_ms);
}

/* ── Sensor Getters ──────────────────────────────────────────────────────── */

GripperState_t Gripper_Up_State(Gripper_t* gripper) {
    if (gripper == NULL || gripper->up_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->up_port_in, gripper->up_pin_in) == GPIO_PIN_SET) ? GRP_STATE_HIGH : GRP_STATE_LOW;
}

GripperState_t Gripper_Down_State(Gripper_t* gripper) {
    if (gripper == NULL || gripper->down_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->down_port_in, gripper->down_pin_in) == GPIO_PIN_SET) ? GRP_STATE_HIGH : GRP_STATE_LOW;
}

GripperState_t Gripper_Claw_State(Gripper_t* gripper) {
    if (gripper == NULL || gripper->claw_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->claw_port_in, gripper->claw_pin_in) == GPIO_PIN_SET) ? GRP_STATE_HIGH : GRP_STATE_LOW;
}
