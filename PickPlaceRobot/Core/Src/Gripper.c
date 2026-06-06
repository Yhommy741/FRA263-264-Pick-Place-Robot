/*
 * Gripper.c
 *
 * Created on: May 24, 2026 | Revised: June 2026
 * Author: FRA263/264 Group 5
 *
 * Dual-mode non-blocking gripper driver.
 *
 * IO Mode
 * ────────
 *   Each command pulses a GPIO pin HIGH and records HAL_GetTick().
 *   Gripper_Update() resets the pin after pulse_duration_ms.
 *   Only one pin is ever active at once; a new command cancels the previous.
 *
 * CAN Mode
 * ─────────
 *   Each command calls CANBus_WriteRelays() with the appropriate bitmask.
 *   Gripper_Update() calls CANBus_WriteRelays(0x00) after pulse_duration_ms
 *   to de-energise the solenoid.
 *   Only one relay group is ever asserted at once; a new command overrides
 *   the pending mask immediately.
 *
 *   Sensor state is read from bus->opto_state, which is kept current by the
 *   CANBus driver whenever a Command Response or Real-Time Broadcast arrives.
 */

#include "Gripper.h"
#include <stddef.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Private helpers — IO mode
 * ═══════════════════════════════════════════════════════════════════════════ */

static void io_start_pulse(Gripper_t *g, GPIO_TypeDef *port, uint16_t pin)
{
    if (g == NULL || port == NULL) return;

    /* Cancel any previous pulse */
    if (g->pulse_active && g->active_port != NULL)
        HAL_GPIO_WritePin(g->active_port, g->active_pin, GPIO_PIN_RESET);

    /* Start new pulse */
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    g->active_port    = port;
    g->active_pin     = pin;
    g->pulse_start_ms = HAL_GetTick();
    g->pulse_active   = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Private helpers — CAN mode
 * ═══════════════════════════════════════════════════════════════════════════ */

static void can_start_pulse(Gripper_t *g, uint8_t relay_mask)
{
    if (g == NULL || g->bus == NULL) return;

    /* Override any in-flight relay command immediately */
    CANBus_WriteRelays(g->bus, relay_mask);

    g->can_relay_pending  = relay_mask;
    g->can_pulse_start_ms = HAL_GetTick();
    g->can_pulse_active   = 1;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Gripper_Init_IO
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_Init_IO(Gripper_t *gripper,
                     GPIO_TypeDef *up_port_out,    uint16_t up_pin_out,
                     GPIO_TypeDef *down_port_out,  uint16_t down_pin_out,
                     GPIO_TypeDef *open_port_out,  uint16_t open_pin_out,
                     GPIO_TypeDef *close_port_out, uint16_t close_pin_out,
                     GPIO_TypeDef *up_port_in,     uint16_t up_pin_in,
                     GPIO_TypeDef *down_port_in,   uint16_t down_pin_in,
                     GPIO_TypeDef *claw_port_in,   uint16_t claw_pin_in)
{
    if (gripper == NULL) return;
    memset(gripper, 0, sizeof(Gripper_t));

    gripper->mode = GRP_MODE_IO;

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
    gripper->up_port_in   = up_port_in;
    gripper->up_pin_in    = up_pin_in;
    gripper->down_port_in = down_port_in;
    gripper->down_pin_in  = down_pin_in;
    gripper->claw_port_in = claw_port_in;
    gripper->claw_pin_in  = claw_pin_in;

    /* Default pulse width */
    gripper->pulse_duration_ms = 100u;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Gripper_Init_CAN
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_Init_CAN(Gripper_t *gripper, CANBus_t *bus,
                      GPIO_TypeDef *up_port_in,   uint16_t up_pin_in,
                      GPIO_TypeDef *down_port_in, uint16_t down_pin_in,
                      GPIO_TypeDef *claw_port_in, uint16_t claw_pin_in)
{
    if (gripper == NULL || bus == NULL) return;
    memset(gripper, 0, sizeof(Gripper_t));

    gripper->mode = GRP_MODE_CANBUS;
    gripper->bus  = bus;

    /* Sensor inputs — always read via GPIO regardless of mode */
    gripper->up_port_in   = up_port_in;
    gripper->up_pin_in    = up_pin_in;
    gripper->down_port_in = down_port_in;
    gripper->down_pin_in  = down_pin_in;
    gripper->claw_port_in = claw_port_in;
    gripper->claw_pin_in  = claw_pin_in;

    /* Default pulse width */
    gripper->pulse_duration_ms = 100u;

    /* NOTE: Do NOT call CANBus_WriteRelays or CANBus_WriteConfig here.
     * The node is still in BOOT state and will ignore commands until it
     * receives the first Master Heartbeat. Initial relay commands are sent
     * from Robot_CANBus_Update once the node reaches Operational state. */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Gripper_Update  — call every main-loop iteration
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_Update(Gripper_t *gripper)
{
    if (gripper == NULL) return;

    if (gripper->mode == GRP_MODE_IO)
    {
        /* IO: reset pin after pulse duration */
        if (gripper->pulse_active)
        {
            if ((HAL_GetTick() - gripper->pulse_start_ms) >= gripper->pulse_duration_ms)
            {
                HAL_GPIO_WritePin(gripper->active_port,
                                  gripper->active_pin,
                                  GPIO_PIN_RESET);
                gripper->active_port  = NULL;
                gripper->active_pin   = 0;
                gripper->pulse_active = 0;
            }
        }
    }
    else /* GRP_MODE_CANBUS */
    {
        /* CAN: send all-off frame after pulse duration */
        if (gripper->can_pulse_active)
        {
            if ((HAL_GetTick() - gripper->can_pulse_start_ms) >= gripper->pulse_duration_ms)
            {
                CANBus_WriteRelays(gripper->bus, GRP_CAN_RELAY_ALL_OFF);
                gripper->can_relay_pending = 0;
                gripper->can_pulse_active  = 0;
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Command Functions
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_MoveUp(Gripper_t *gripper)
{
    if (gripper == NULL) return;
    if (gripper->mode == GRP_MODE_IO)
        io_start_pulse(gripper, gripper->up_port_out, gripper->up_pin_out);
    else
        can_start_pulse(gripper, GRP_CAN_RELAY_UP);
}

void Gripper_MoveDown(Gripper_t *gripper)
{
    if (gripper == NULL) return;
    if (gripper->mode == GRP_MODE_IO)
        io_start_pulse(gripper, gripper->down_port_out, gripper->down_pin_out);
    else
        can_start_pulse(gripper, GRP_CAN_RELAY_DOWN);
}

void Gripper_Open(Gripper_t *gripper)
{
    if (gripper == NULL) return;
    if (gripper->mode == GRP_MODE_IO)
        io_start_pulse(gripper, gripper->open_port_out, gripper->open_pin_out);
    else
        can_start_pulse(gripper, GRP_CAN_RELAY_OPEN);
}

void Gripper_Close(Gripper_t *gripper)
{
    if (gripper == NULL) return;
    if (gripper->mode == GRP_MODE_IO)
        io_start_pulse(gripper, gripper->close_port_out, gripper->close_pin_out);
    else
        can_start_pulse(gripper, GRP_CAN_RELAY_CLOSE);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Sensor Getter Functions
 * Both modes read directly from GPIO — CAN protocol does not provide sensor
 * feedback, so physical GPIO pins are always used regardless of gripper mode.
 * ═══════════════════════════════════════════════════════════════════════════ */
GripperState_t Gripper_Up_State(Gripper_t *gripper)
{
    if (gripper == NULL || gripper->up_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->up_port_in, gripper->up_pin_in) == GPIO_PIN_SET)
           ? GRP_STATE_HIGH : GRP_STATE_LOW;
}

GripperState_t Gripper_Down_State(Gripper_t *gripper)
{
    if (gripper == NULL || gripper->down_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->down_port_in, gripper->down_pin_in) == GPIO_PIN_SET)
           ? GRP_STATE_HIGH : GRP_STATE_LOW;
}

GripperState_t Gripper_Claw_State(Gripper_t *gripper)
{
    if (gripper == NULL || gripper->claw_port_in == NULL) return GRP_STATE_LOW;
    return (HAL_GPIO_ReadPin(gripper->claw_port_in, gripper->claw_pin_in) == GPIO_PIN_SET)
           ? GRP_STATE_HIGH : GRP_STATE_LOW;
}
