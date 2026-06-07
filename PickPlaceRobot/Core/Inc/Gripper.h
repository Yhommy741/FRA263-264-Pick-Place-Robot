/*
 * Gripper.h
 *
 * Created on: May 24, 2026
 * Author: Yhommy
 *
 * Dual-mode non-blocking gripper driver interface.
 * GRP_MODE_IO     — direct GPIO output (original behaviour).
 * GRP_MODE_CANBUS — CAN bus output via protocol v1.0.1.
 * Sensor lines always use GPIO; CAN v1.0.1 has no sensor readback.
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

#include "stm32g4xx_hal.h"
#include "CANBus.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Gripper Mode
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    GRP_MODE_IO     = 0,   /* Direct GPIO control (original)            */
    GRP_MODE_CANBUS = 1    /* CAN bus relay control via protocol v1.0.1 */
} GripperMode_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * Gripper State (sensor read result)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    GRP_STATE_LOW  = 0,
    GRP_STATE_HIGH = 1
} GripperState_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * CAN Relay Bitmask Definitions
 * (matches Relay Bank 0 in protocol spec)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define GRP_CAN_RELAY_UP        (1u << 0u)   /* Relay 0 — Up   solenoid     */
#define GRP_CAN_RELAY_DOWN      (1u << 1u)   /* Relay 1 — Down solenoid     */
#define GRP_CAN_RELAY_CLOSE     (1u << 2u)   /* Relay 2 — Close solenoid    */
#define GRP_CAN_RELAY_OPEN      (1u << 3u)   /* Relay 3 — Open  solenoid    */
#define GRP_CAN_RELAY_ALL_OFF   0x00u

/* NOTE: Sensor feedback (Relay 4/5/6) is NOT readable via CAN protocol v1.0.1.
 * No frame delivers Relay 4-6 state to the master. Use GRP_MODE_IO for sensors. */

/* ═══════════════════════════════════════════════════════════════════════════
 * Gripper_t
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {

    /* ── Mode ─────────────────────────────────────────────────────────────── */
    GripperMode_t mode;

    /* ── IO Mode: Actuator Output Pins ────────────────────────────────────── */
    GPIO_TypeDef *up_port_out;
    uint16_t      up_pin_out;

    GPIO_TypeDef *down_port_out;
    uint16_t      down_pin_out;

    GPIO_TypeDef *open_port_out;
    uint16_t      open_pin_out;

    GPIO_TypeDef *close_port_out;
    uint16_t      close_pin_out;

    /* ── IO Mode: Sensor Input Pins ───────────────────────────────────────── */
    GPIO_TypeDef *up_port_in;
    uint16_t      up_pin_in;

    GPIO_TypeDef *down_port_in;
    uint16_t      down_pin_in;

    GPIO_TypeDef *claw_port_in;
    uint16_t      claw_pin_in;

    /* ── CAN Mode ─────────────────────────────────────────────────────────── */
    CANBus_t     *bus;          /* Pointer to an initialised CANBus_t         */

    /* ── Shared: pulse / command timing ──────────────────────────────────── */
    uint32_t      pulse_duration_ms;  /* Solenoid pulse width [ms], default 100 */

    /* IO mode — active pulse tracking (internal) */
    GPIO_TypeDef *active_port;        /* Port of the currently active pulse    */
    uint16_t      active_pin;         /* Pin  of the currently active pulse    */
    uint32_t      pulse_start_ms;     /* HAL_GetTick() when pulse was started  */
    uint8_t       pulse_active;       /* 1 = pulse in progress                 */

    /* CAN mode — pending relay bitmask tracking (internal) */
    uint8_t       can_relay_pending;  /* Relay bits to clear after pulse_ms    */
    uint8_t       can_pulse_active;   /* 1 = CAN relay pulse in progress       */
    uint32_t      can_pulse_start_ms; /* HAL_GetTick() when CAN pulse started  */

} Gripper_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * Initialisation
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise gripper in IO (direct GPIO) mode.
 *         Identical to the original Gripper_Init() signature.
 */
void Gripper_Init_IO(Gripper_t *gripper,
                     GPIO_TypeDef *up_port_out,    uint16_t up_pin_out,
                     GPIO_TypeDef *down_port_out,  uint16_t down_pin_out,
                     GPIO_TypeDef *open_port_out,  uint16_t open_pin_out,
                     GPIO_TypeDef *close_port_out, uint16_t close_pin_out,
                     GPIO_TypeDef *up_port_in,     uint16_t up_pin_in,
                     GPIO_TypeDef *down_port_in,   uint16_t down_pin_in,
                     GPIO_TypeDef *claw_port_in,   uint16_t claw_pin_in);

/**
 * @brief  Initialise gripper in CAN bus mode.
 *         Sensor pins are still read via GPIO — CAN protocol does not provide
 *         sensor feedback. Sensor pin arguments may be NULL if not wired.
 */
void Gripper_Init_CAN(Gripper_t *gripper, CANBus_t *bus,
                      GPIO_TypeDef *up_port_in,   uint16_t up_pin_in,
                      GPIO_TypeDef *down_port_in, uint16_t down_pin_in,
                      GPIO_TypeDef *claw_port_in, uint16_t claw_pin_in);

/* ═══════════════════════════════════════════════════════════════════════════
 * Main Loop Function
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Call every main-loop iteration (both modes).
 *         IO   mode: resets the active GPIO pin after pulse_duration_ms.
 *         CAN  mode: sends an all-relays-off frame after pulse_duration_ms.
 *         Non-blocking — returns immediately.
 */
void Gripper_Update(Gripper_t *gripper);

/* ═══════════════════════════════════════════════════════════════════════════
 * Command Functions  (non-blocking, both modes)
 * ═══════════════════════════════════════════════════════════════════════════ */
void Gripper_MoveUp  (Gripper_t *gripper);
void Gripper_MoveDown(Gripper_t *gripper);
void Gripper_Open    (Gripper_t *gripper);
void Gripper_Close   (Gripper_t *gripper);

/* ═══════════════════════════════════════════════════════════════════════════
 * Sensor Getter Functions  (both modes)
 * Always reads directly from GPIO pins — CAN protocol v1.0.1 has no sensor
 * readback frame, so GPIO is used regardless of gripper mode.
 * ═══════════════════════════════════════════════════════════════════════════ */
GripperState_t Gripper_Up_State  (Gripper_t *gripper);
GripperState_t Gripper_Down_State(Gripper_t *gripper);
GripperState_t Gripper_Claw_State(Gripper_t *gripper);

#ifdef __cplusplus
}
#endif

#endif /* INC_GRIPPER_H_ */
