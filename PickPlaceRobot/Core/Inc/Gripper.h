/*
 * Gripper.h
 *
 * Created on: May 24, 2026
 * Author: Yhommy's Notebook
 *
 * Non-blocking gripper driver.
 *
 * Each command (MoveUp/Down/Open/Close) sets a GPIO pin HIGH and records
 * a timestamp.  Gripper_Update() must be called every main-loop iteration —
 * it resets the pin after pulse_duration_ms without blocking.
 *
 * HAL_Delay() is NOT used — Modbus / control loop never stalls.
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ── Gripper State Enumeration ───────────────────────────────────────────── */
typedef enum {
    GRP_STATE_LOW  = 0,
    GRP_STATE_HIGH = 1
} GripperState_t;

/* ── Gripper Structure ───────────────────────────────────────────────────── */
typedef struct {
    /* Actuator Output Pins */
    GPIO_TypeDef* up_port_out;
    uint16_t      up_pin_out;

    GPIO_TypeDef* down_port_out;
    uint16_t      down_pin_out;

    GPIO_TypeDef* open_port_out;
    uint16_t      open_pin_out;

    GPIO_TypeDef* close_port_out;
    uint16_t      close_pin_out;

    /* Sensor Input Pins */
    GPIO_TypeDef* up_port_in;
    uint16_t      up_pin_in;

    GPIO_TypeDef* down_port_in;
    uint16_t      down_pin_in;

    GPIO_TypeDef* claw_port_in;
    uint16_t      claw_pin_in;

    /* Pulse timing — non-blocking */
    uint32_t      pulse_duration_ms;  /* HIGH pulse width [ms], default 100   */

    /* Active pulse tracking (internal — set by command, cleared by Update) */
    GPIO_TypeDef* active_port;        /* port of the currently active pulse   */
    uint16_t      active_pin;         /* pin  of the currently active pulse   */
    uint32_t      pulse_start_ms;     /* HAL_GetTick() when pulse was started */
    uint8_t       pulse_active;       /* 1 = pulse in progress                */

} Gripper_t;

/* ── Function Prototypes ─────────────────────────────────────────────────── */

void Gripper_Init(Gripper_t* gripper,
                  GPIO_TypeDef* up_port_out,    uint16_t up_pin_out,
                  GPIO_TypeDef* down_port_out,  uint16_t down_pin_out,
                  GPIO_TypeDef* open_port_out,  uint16_t open_pin_out,
                  GPIO_TypeDef* close_port_out, uint16_t close_pin_out,
                  GPIO_TypeDef* up_port_in,     uint16_t up_pin_in,
                  GPIO_TypeDef* down_port_in,   uint16_t down_pin_in,
                  GPIO_TypeDef* claw_port_in,   uint16_t claw_pin_in);

/**
 * @brief  Call every main-loop iteration.
 *         Resets the active output pin after pulse_duration_ms.
 *         Non-blocking — returns immediately.
 */
void Gripper_Update(Gripper_t* gripper);

/* Command Functions — non-blocking pulse start */
void Gripper_MoveUp  (Gripper_t* gripper);
void Gripper_MoveDown(Gripper_t* gripper);
void Gripper_Open    (Gripper_t* gripper);
void Gripper_Close   (Gripper_t* gripper);

/* Sensor Getter Functions */
GripperState_t Gripper_Up_State  (Gripper_t* gripper);
GripperState_t Gripper_Down_State(Gripper_t* gripper);
GripperState_t Gripper_Claw_State(Gripper_t* gripper);

#endif /* INC_GRIPPER_H_ */
