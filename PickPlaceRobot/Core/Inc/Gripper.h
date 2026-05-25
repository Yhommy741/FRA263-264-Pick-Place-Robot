/*
 * Gripper.h
 *
 * Created on: May 24, 2026
 * Author: Yhommy's Notebook
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

#include "stm32g4xx_hal.h"

/* ── Gripper State Enumeration ───────────────────────────────────────────── */
typedef enum {
    GRP_STATE_LOW = 0,
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

    /* Timing configuration */
    uint32_t      pulse_duration_ms;
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

/* Command Functions (100ms Pulsed Outputs) */
void Gripper_MoveUp(Gripper_t* gripper);
void Gripper_MoveDown(Gripper_t* gripper);
void Gripper_Open(Gripper_t* gripper);
void Gripper_Close(Gripper_t* gripper);

/* Sensor Getter Functions */
GripperState_t Gripper_Up_State(Gripper_t* gripper);
GripperState_t Gripper_Down_State(Gripper_t* gripper);
GripperState_t Gripper_Claw_State(Gripper_t* gripper);

#endif /* INC_GRIPPER_H_ */
