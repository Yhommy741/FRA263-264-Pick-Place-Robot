/*
 * CANBus.c
 *
 * Created : June 2026
 * Author  : FRA263/264 Group 5
 *
 * Implementation of the CAN protocol driver (v1.0.1).
 *
 * Design notes
 * ─────────────
 * • Non-blocking: no HAL_Delay() anywhere.
 * • CANBus_Update() must be called every main-loop iteration.
 * • Heartbeat TX is handled internally by CANBus_Update(); the caller only
 *   needs to ensure CANBus_Update() runs at least once per 500 ms.
 * • All TX is done via HAL_FDCAN_AddMessageToTxFifoQ (polling mode);
 *   the FDCAN TX FIFO has 3 slots so back-pressure is handled automatically.
 * • RX filter uses TWO range filters:
 *     Filter 0 : 0x010 – 0x010  (EMCY)
 *     Filter 1 : 0x110 – 0x710  (RT Data, Cmd Resp, Cfg Resp, Node HB)
 *   This requires StdFiltersNbr = 2 in MX_FDCAN1_Init().
 *   If only 1 filter slot is available use a mask filter instead — see
 *   CANBus_Init_SingleFilter() alternative commented below.
 */

#include "CANBus.h"
#include <stddef.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Private helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Build a pre-filled TX header for standard-frame, 8-byte DLC.
 *         Caller adjusts .Identifier and .DataLength before passing to HAL.
 */
static void priv_fill_txheader(FDCAN_TxHeaderTypeDef *h,
                                uint32_t id, uint32_t dlc)
{
    h->Identifier          = id;
    h->IdType              = FDCAN_STANDARD_ID;
    h->TxFrameType         = FDCAN_DATA_FRAME;
    h->DataLength          = dlc;
    h->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h->BitRateSwitch       = FDCAN_BRS_OFF;
    h->FDFormat            = FDCAN_CLASSIC_CAN;
    h->TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    h->MessageMarker       = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_Init
 * ═══════════════════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef CANBus_Init(CANBus_t *bus,
                               FDCAN_HandleTypeDef *hfdcan,
                               uint8_t node_id)
{
    if (bus == NULL || hfdcan == NULL) return HAL_ERROR;

    /* ── Populate handle ──────────────────────────────────────────────────── */
    memset(bus, 0, sizeof(CANBus_t));
    bus->hfdcan     = hfdcan;
    bus->node_id    = node_id;
    bus->node_state = CANBUS_NODE_STATE_BOOT;

    /* ── Global filter: reject all non-matching frames ───────────────────── */
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                  FDCAN_REJECT,         /* non-matching std  */
                                  FDCAN_REJECT,         /* non-matching ext  */
                                  FDCAN_FILTER_REMOTE,  /* remote frames     */
                                  FDCAN_FILTER_REMOTE);

    /*
     * Filter 0 — Range: 0x010 .. 0x010  (EMCY from node_id)
     *   EMCY CAN-ID = CANBUS_MAKE_ID(0x0, node_id)
     */
    uint32_t id_emcy  = CANBUS_MAKE_ID(CANBUS_FC_EMCY,     node_id);
    uint32_t id_rt    = CANBUS_MAKE_ID(CANBUS_FC_RT_DATA,  node_id);
    uint32_t id_cresp = CANBUS_MAKE_ID(CANBUS_FC_CMD_RESP, node_id);
    uint32_t id_nohb  = CANBUS_MAKE_ID(CANBUS_FC_NODE_HB,  node_id);

    /* Filter 0: EMCY single ID */
    FDCAN_FilterTypeDef f0 = {
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterIndex  = 0,
        .FilterID1    = id_emcy,
        .FilterID2    = id_emcy,      /* exact match via RANGE with same lo/hi */
        .IdType       = FDCAN_STANDARD_ID,
        .FilterType   = FDCAN_FILTER_RANGE
    };
    if (HAL_FDCAN_ConfigFilter(hfdcan, &f0) != HAL_OK) return HAL_ERROR;

    /*
     * Filter 1 — Range: id_rt .. id_nohb
     *   Covers 0x110, 0x310, 0x510, 0x710  (all node responses)
     *   Note: 0x210 and 0x410 are Master→Node; they are in this range too but
     *   the STM32 master never transmits to itself, so filtering them in is
     *   harmless — they simply never appear on RX FIFO.
     */
    FDCAN_FilterTypeDef f1 = {
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterIndex  = 1,
        .FilterID1    = id_rt,        /* 0x110 — lowest expected RX ID        */
        .FilterID2    = id_nohb,      /* 0x710 — highest expected RX ID       */
        .IdType       = FDCAN_STANDARD_ID,
        .FilterType   = FDCAN_FILTER_RANGE
    };
    if (HAL_FDCAN_ConfigFilter(hfdcan, &f1) != HAL_OK) return HAL_ERROR;

    /* ── Start FDCAN ─────────────────────────────────────────────────────── */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) return HAL_ERROR;

    bus->hb_last_tick = HAL_GetTick();
    return HAL_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_RegisterCallbacks
 * ═══════════════════════════════════════════════════════════════════════════ */
void CANBus_RegisterCallbacks(CANBus_t *bus,
                               CANBus_CmdRespCallback_t on_cmd_resp,
                               CANBus_RTDataCallback_t  on_rt_data,
                               CANBus_NodeHBCallback_t  on_node_hb,
                               CANBus_EMCYCallback_t    on_emcy)
{
    if (bus == NULL) return;
    bus->on_cmd_resp = on_cmd_resp;
    bus->on_rt_data  = on_rt_data;
    bus->on_node_hb  = on_node_hb;
    bus->on_emcy     = on_emcy;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Private: process a single received frame
 * ═══════════════════════════════════════════════════════════════════════════ */
static void priv_dispatch(CANBus_t *bus,
                           FDCAN_RxHeaderTypeDef *h,
                           const uint8_t *d)
{
    uint32_t id       = h->Identifier;
    uint8_t  func     = (uint8_t)((id >> 8u) & 0x07u);
    /* uint8_t  src_node = (uint8_t)(id & 0xFFu); */  /* for future use */

    switch (func)
    {
    /* ── EMCY (0x0) ──────────────────────────────────────────────────────── */
    case CANBUS_FC_EMCY:
        if (h->DataLength >= FDCAN_DLC_BYTES_2)
        {
            if (bus->on_emcy) bus->on_emcy(d[0], d[1]);
        }
        break;

    /* ── Real-Time Broadcast (0x1) ───────────────────────────────────────── */
    case CANBUS_FC_RT_DATA:
        if (h->DataLength >= FDCAN_DLC_BYTES_1)
        {
            /* RT broadcast carries opto data (bits 0-3 only).
             * Sensor feedback is in relay_state — not here. */
            if (bus->on_rt_data) bus->on_rt_data(d[0]);
        }
        break;

    /* ── Command Response (0x3) ──────────────────────────────────────────── */
    case CANBUS_FC_CMD_RESP:
        if (h->DataLength >= FDCAN_DLC_BYTES_3)
        {
            uint8_t ins    = d[0];
            uint8_t target = d[1];
            uint8_t data   = d[2];

            if (ins == CANBUS_INS_WRITE_ACK &&
                target == CANBUS_TARGET_RELAY_BANK0)
            {
                /* Actual State byte contains relay outputs (bits 0-3) AND
                 * sensor feedback (bits 4-6 = Relay 4/5/6 per spec). */
                bus->relay_state = data;
            }

            if (bus->on_cmd_resp) bus->on_cmd_resp(ins, target, data);
        }
        break;

    /* ── Node Heartbeat (0x7) ────────────────────────────────────────────── */
    case CANBUS_FC_NODE_HB:
        if (h->DataLength >= FDCAN_DLC_BYTES_1)
        {
            bus->node_state = d[0];
            if (bus->on_node_hb) bus->on_node_hb(d[0]);
        }
        break;

    /* Config Response (0x5) — acknowledge only, no state to cache */
    case CANBUS_FC_CFG_RESP:
    default:
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_Update  — call from main loop ONLY, never from ISR
 * ═══════════════════════════════════════════════════════════════════════════ */
void CANBus_Update(CANBus_t *bus)
{
    if (bus == NULL) return;

    /* ── Drain RX FIFO0 ──────────────────────────────────────────────────── */
    while (HAL_FDCAN_GetRxFifoFillLevel(bus->hfdcan, FDCAN_RX_FIFO0) > 0u)
    {
        FDCAN_RxHeaderTypeDef rxh;
        uint8_t               rxd[8] = {0};

        if (HAL_FDCAN_GetRxMessage(bus->hfdcan,
                                    FDCAN_RX_FIFO0,
                                    &rxh, rxd) == HAL_OK)
        {
            priv_dispatch(bus, &rxh, rxd);
        }
    }

    /* ── Non-blocking heartbeat TX ───────────────────────────────────────── */
    if ((HAL_GetTick() - bus->hb_last_tick) >= CANBUS_HEARTBEAT_PERIOD_MS)
    {
        CANBus_SendHeartbeat(bus, CANBUS_MASTER_STATE_OPER);
    }

    /* ── Periodic relay readback poll ────────────────────────────────────── *
     * Sensor feedback (Relay 4/5/6) only arrives in the Write Ack response   *
     * (0x310 Byte 2 Actual State). Send a no-op write of the current relay   *
     * output state every CANBUS_RELAY_POLL_PERIOD_MS to force the node to    *
     * reply with a Write Ack, refreshing relay_state bits 4-6 (sensors).     *
     * Writing the same state the node already holds is safe — no side effect. */
    if ((HAL_GetTick() - bus->relay_poll_last_tick) >= CANBUS_RELAY_POLL_PERIOD_MS)
    {
        CANBus_WriteRelays(bus, bus->relay_out);
        bus->relay_poll_last_tick = HAL_GetTick();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_SendHeartbeat
 * ═══════════════════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef CANBus_SendHeartbeat(CANBus_t *bus, uint8_t master_state)
{
    if (bus == NULL) return HAL_ERROR;

    FDCAN_TxHeaderTypeDef txh;
    uint8_t txd[1] = { master_state };

    priv_fill_txheader(&txh,
                        CANBUS_MAKE_ID(CANBUS_FC_MASTER_HB, CANBUS_NODE_BROADCAST),
                        FDCAN_DLC_BYTES_1);

    HAL_StatusTypeDef ret =
        HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &txh, txd);

    if (ret == HAL_OK)
        bus->hb_last_tick = HAL_GetTick();

    return ret;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_WriteRelays
 * ═══════════════════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef CANBus_WriteRelays(CANBus_t *bus, uint8_t state_mask)
{
    if (bus == NULL) return HAL_ERROR;

    bus->relay_out = state_mask;   /* track intended output for poll readback */

    FDCAN_TxHeaderTypeDef txh;
    uint8_t txd[3] = {
        CANBUS_INS_WRITE_REQ,
        CANBUS_TARGET_RELAY_BANK0,
        state_mask
    };

    priv_fill_txheader(&txh,
                        CANBUS_MAKE_ID(CANBUS_FC_CMD_REQ, bus->node_id),
                        FDCAN_DLC_BYTES_3);

    return HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &txh, txd);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_ReadOptos
 * ═══════════════════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef CANBus_ReadOptos(CANBus_t *bus)
{
    if (bus == NULL) return HAL_ERROR;

    FDCAN_TxHeaderTypeDef txh;
    uint8_t txd[2] = {
        CANBUS_INS_READ_REQ,
        CANBUS_TARGET_OPTO_BANK0
    };

    priv_fill_txheader(&txh,
                        CANBUS_MAKE_ID(CANBUS_FC_CMD_REQ, bus->node_id),
                        FDCAN_DLC_BYTES_2);

    return HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &txh, txd);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CANBus_WriteConfig
 * ═══════════════════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef CANBus_WriteConfig(CANBus_t *bus,
                                      uint8_t  param_id,
                                      uint16_t value)
{
    if (bus == NULL) return HAL_ERROR;

    FDCAN_TxHeaderTypeDef txh;
    uint8_t txd[4] = {
        CANBUS_INS_CFG_WRITE,
        param_id,
        (uint8_t)(value & 0xFFu),        /* LSB */
        (uint8_t)((value >> 8u) & 0xFFu) /* MSB */
    };

    priv_fill_txheader(&txh,
                        CANBUS_MAKE_ID(CANBUS_FC_CFG_REQ, bus->node_id),
                        FDCAN_DLC_BYTES_4);

    return HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &txh, txd);
}
