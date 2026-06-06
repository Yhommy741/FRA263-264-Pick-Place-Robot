/*
 * CANBus.h
 *
 * Created : June 2026
 * Author  : FRA263/264 Group 5
 *
 * Low-level CAN bus driver for the custom protocol (v1.0.1).
 *
 * Implements:
 *   - 11-bit ID encoding  : MAKE_CAN_ID(funcCode, nodeID)
 *   - RX filter setup     : accept responses from a specific Node ID
 *   - TX helpers          : Command Request, Config Request, Heartbeat
 *   - RX polling          : CANBus_Poll() — call every main-loop iteration
 *   - NMT watchdog        : Master must call CANBus_SendHeartbeat() every ≤500 ms
 *   - Callback hooks      : register handlers for Command Response, Config
 *                           Response, Real-Time Data, EMCY, and Node Heartbeat
 *
 * Protocol ID structure (11 bits):
 *   Bits [10:8] = Function Code (3 bits)
 *   Bits  [7:0] = Node ID       (8 bits)
 *
 * Function Code Map:
 *   0x0  EMCY              Node → Master   (highest priority)
 *   0x1  Real-Time Data    Node → Master
 *   0x2  Command Request   Master → Node
 *   0x3  Command Response  Node → Master
 *   0x4  Config Request    Master → Node
 *   0x5  Config Response   Node → Master
 *   0x6  Master Heartbeat  Broadcast 0x600
 *   0x7  Node Heartbeat    Node → Master   (lowest priority)
 */

#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 * Protocol Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ID encoding ---------------------------------------------------------------- */
#define CANBUS_MAKE_ID(funcCode, nodeID) \
    (uint32_t)((((funcCode) & 0x07u) << 8u) | ((nodeID) & 0xFFu))

/* Function codes ------------------------------------------------------------- */
#define CANBUS_FC_EMCY              0x0u
#define CANBUS_FC_RT_DATA           0x1u
#define CANBUS_FC_CMD_REQ           0x2u
#define CANBUS_FC_CMD_RESP          0x3u
#define CANBUS_FC_CFG_REQ           0x4u
#define CANBUS_FC_CFG_RESP          0x5u
#define CANBUS_FC_MASTER_HB         0x6u
#define CANBUS_FC_NODE_HB           0x7u

/* Broadcast / reserved node IDs --------------------------------------------- */
#define CANBUS_NODE_BROADCAST       0x00u

/* Heartbeat Master-State byte ------------------------------------------------ */
#define CANBUS_MASTER_STATE_OPER    0x05u   /* Node allowed to drive outputs   */
#define CANBUS_MASTER_STATE_STOP    0x00u   /* Node must force failsafe        */

/* Node State byte (Node Heartbeat Byte 0) ------------------------------------ */
#define CANBUS_NODE_STATE_BOOT      0x00u
#define CANBUS_NODE_STATE_OPER      0x05u
#define CANBUS_NODE_STATE_FAILSAFE  0xFFu

/* Instructions (Byte 0 of payload) ------------------------------------------ */
#define CANBUS_INS_WRITE_REQ        0x10u   /* Command Request  – Write Relays  */
#define CANBUS_INS_READ_REQ         0x20u   /* Command Request  – Read Optos    */
#define CANBUS_INS_WRITE_ACK        0x11u   /* Command Response – Write Ack     */
#define CANBUS_INS_READ_RESP        0x21u   /* Command Response – Read Response */
#define CANBUS_INS_CFG_WRITE        0x40u   /* Config Request   – Write Config  */

/* Target bank bytes ---------------------------------------------------------- */
#define CANBUS_TARGET_RELAY_BANK0   0x00u
#define CANBUS_TARGET_OPTO_BANK0    0x10u

/* Config Parameter IDs ------------------------------------------------------- */
#define CANBUS_CFG_NMT_STATE        0x00u
#define CANBUS_CFG_BCAST_PERIOD     0x01u
#define CANBUS_CFG_FAILSAFE_MASK    0x02u

/* EMCY Error Codes ----------------------------------------------------------- */
#define CANBUS_ERR_VOLTAGE          0x10u
#define CANBUS_ERR_OVERCURRENT      0x20u
#define CANBUS_ERR_BUS_PASSIVE      0x80u

/* Heartbeat interval --------------------------------------------------------- */
#define CANBUS_HEARTBEAT_PERIOD_MS  500u

/* RT broadcast period requested from node at init [ms] ----------------------- */
#define CANBUS_RT_BROADCAST_PERIOD_MS  50u

/* Relay readback poll period [ms] — periodic write to force Write Ack --------- *
 * This is the only way to refresh sensor feedback (Relay 4/5/6 in Write Ack).  */
#define CANBUS_RELAY_POLL_PERIOD_MS  100u


/* ═══════════════════════════════════════════════════════════════════════════
 * Callback type definitions
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Called when a Command Response (0x3xx) is received.
 * @param  instruction  CANBUS_INS_WRITE_ACK or CANBUS_INS_READ_RESP
 * @param  target       CANBUS_TARGET_RELAY_BANK0 or CANBUS_TARGET_OPTO_BANK0
 * @param  data_byte    Relay actual-state or Opto bitmask
 */
typedef void (*CANBus_CmdRespCallback_t)(uint8_t instruction,
                                         uint8_t target,
                                         uint8_t data_byte);

/**
 * @brief  Called when a Real-Time Broadcast (0x1xx) is received.
 * @param  opto_mask    Opto bitmask (Bit0=Opto0 … Bit3=Opto3)
 */
typedef void (*CANBus_RTDataCallback_t)(uint8_t opto_mask);

/**
 * @brief  Called when a Node Heartbeat (0x7xx) is received.
 * @param  node_state   CANBUS_NODE_STATE_*
 */
typedef void (*CANBus_NodeHBCallback_t)(uint8_t node_state);

/**
 * @brief  Called when an EMCY message (0x0xx) is received.
 * @param  error_code   CANBUS_ERR_*
 * @param  info         Reserved diagnostic byte
 */
typedef void (*CANBus_EMCYCallback_t)(uint8_t error_code, uint8_t info);


/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_t  — driver handle
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    FDCAN_HandleTypeDef *hfdcan;        /* HAL FDCAN handle (e.g. &hfdcan1)    */
    uint8_t              node_id;       /* Target node ID  (e.g. 0x10)         */

    /* Non-blocking heartbeat timing */
    uint32_t             hb_last_tick;         /* HAL_GetTick() of last TX heartbeat  */

    /* Relay readback poll timing */
    uint32_t             relay_poll_last_tick;  /* HAL_GetTick() of last relay poll TX */

    /* Cached relay states */
    uint8_t              relay_out;    /* Last requested relay output mask (bits 0-3) */
    uint8_t              relay_state;  /* Last reported actual state from Write Ack   *
                                        * bits 0-3 = relay outputs                    *
                                        * bits 4-6 = sensor feedback (Relay 4/5/6)   */

    /* Cached node state (updated from Node Heartbeat RX) */
    uint8_t              node_state;    /* Last received CANBUS_NODE_STATE_*   */

    /* User-registered callbacks (NULL = disabled) */
    CANBus_CmdRespCallback_t on_cmd_resp;
    CANBus_RTDataCallback_t  on_rt_data;
    CANBus_NodeHBCallback_t  on_node_hb;
    CANBus_EMCYCallback_t    on_emcy;

} CANBus_t;


/* ═══════════════════════════════════════════════════════════════════════════
 * API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Initialise the CANBus driver, configure RX filters, and start FDCAN.
 *
 * Accepts frames from:
 *   0x110  Real-Time Data
 *   0x310  Command Response
 *   0x510  Config Response
 *   0x710  Node Heartbeat
 *   0x010  EMCY
 *
 * (All with node_id 0x10 as per spec §5.2)
 *
 * @param  bus      Pointer to an uninitialised CANBus_t instance.
 * @param  hfdcan   Pointer to the HAL FDCAN handle (already MX-initialised).
 * @param  node_id  Target node ID (e.g. 0x10).
 * @retval HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef CANBus_Init(CANBus_t *bus,
                               FDCAN_HandleTypeDef *hfdcan,
                               uint8_t node_id);

/**
 * @brief  Register application callbacks.
 *         Any pointer may be NULL to disable that callback.
 */
void CANBus_RegisterCallbacks(CANBus_t *bus,
                               CANBus_CmdRespCallback_t on_cmd_resp,
                               CANBus_RTDataCallback_t  on_rt_data,
                               CANBus_NodeHBCallback_t  on_node_hb,
                               CANBus_EMCYCallback_t    on_emcy);

/**
 * @brief  Call every main-loop iteration.
 *         Drains the RX FIFO0, dispatches callbacks, and manages the
 *         non-blocking heartbeat TX (every CANBUS_HEARTBEAT_PERIOD_MS).
 */
void CANBus_Update(CANBus_t *bus);

/* ── TX helpers ─────────────────────────────────────────────────────────── */

/**
 * @brief  Send a Master Heartbeat (0x600).
 * @param  master_state  CANBUS_MASTER_STATE_OPER or CANBUS_MASTER_STATE_STOP
 */
HAL_StatusTypeDef CANBus_SendHeartbeat(CANBus_t *bus, uint8_t master_state);

/**
 * @brief  Send a Write Relay Command Request (0x210).
 *         Sets relays according to state_mask (Bit0=Relay0 … Bit3=Relay3).
 * @param  state_mask  Desired relay bitmask
 */
HAL_StatusTypeDef CANBus_WriteRelays(CANBus_t *bus, uint8_t state_mask);

/**
 * @brief  Send a Read Opto Command Request (0x210).
 *         Node will reply with a Command Response (0x310).
 */
HAL_StatusTypeDef CANBus_ReadOptos(CANBus_t *bus);

/**
 * @brief  Send a Config Request (0x410).
 * @param  param_id  CANBUS_CFG_*
 * @param  value     16-bit config value (little-endian in payload)
 */
HAL_StatusTypeDef CANBus_WriteConfig(CANBus_t *bus,
                                      uint8_t  param_id,
                                      uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* INC_CANBUS_H_ */
