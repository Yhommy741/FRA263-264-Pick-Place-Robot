/*
 * SerialFrame.c
 *
 * Created on: Apr 23, 2026
 * Author: Yhommy
 *
 * Binary serial framing library over UART (DMA-backed).
 * Builds and parses fixed-layout frames: [ HEADER | fields | TERM ].
 * Up to 32 TX and 32 RX fields; 8 supported data types.
 */

#include "SerialFrame.h"
#include <string.h>

static const uint8_t TYPE_SIZE[8] = {
    1,  /* SF_UINT8  */
    1,  /* SF_INT8   */
    2,  /* SF_UINT16 */
    2,  /* SF_INT16  */
    4,  /* SF_UINT32 */
    4,  /* SF_INT32  */
    4,  /* SF_FLOAT  */
    8   /* SF_DOUBLE */
};

/* =============================================================================
 *  SerialFrame_Init
 * =============================================================================
 */
void SerialFrame_Init(SerialFrame_t *frame,
                      UART_HandleTypeDef *huart,
                      uint8_t header,
                      uint8_t terminator)
{
    frame->huart      = huart;
    frame->header     = header;
    frame->terminator = terminator;

    frame->tx_count      = 0;
    frame->rx_count      = 0;
    frame->tx_frame_size = 2;   /* header (byte 0) + terminator (byte 1) */
    frame->rx_frame_size = 2;

    memset(frame->tx_buf, 0, SF_MAX_FRAME_BYTES);
    memset(frame->rx_buf, 0, SF_MAX_FRAME_BYTES);
}

/* =============================================================================
 *  SerialFrame_Add_TX
 *
 *  FIX: byte_offset = (tx_frame_size - 1) places each field where the
 *       terminator currently sits, then tx_frame_size grows by sz so the
 *       terminator shifts right.  This is correct.
 *
 *  Frame build example for two SF_FLOAT fields (4 bytes each):
 *    After Init:          tx_frame_size=2,  layout=[H][T]
 *    After Add field0:    tx_frame_size=6,  layout=[H][f0_0][f0_1][f0_2][f0_3][T]
 *    After Add field1:    tx_frame_size=10, layout=[H][f0...][f1_0][f1_1][f1_2][f1_3][T]
 * =============================================================================
 */
int SerialFrame_Add_TX(SerialFrame_t *frame,
                       const char    *name,
                       void          *data_ptr,
                       SF_Type_t      type)
{
    if (frame->tx_count >= SF_MAX_FIELDS) return -1;

    uint8_t sz = TYPE_SIZE[type];
    if ((frame->tx_frame_size + sz) > SF_MAX_FRAME_BYTES) return -1;

    SF_Field_t *f  = &frame->tx_fields[frame->tx_count];
    f->name        = name;
    f->type        = type;
    f->size        = sz;
    f->data_ptr    = data_ptr;
    /* FIX: offset = tx_frame_size - 1 (terminator slot, shifts right each time) */
    f->byte_offset = frame->tx_frame_size - 1;

    frame->tx_frame_size += sz;
    frame->tx_count++;
    return 0;
}

/* =============================================================================
 *  SerialFrame_Add_RX
 *  Same offset logic as Add_TX.
 * =============================================================================
 */
int SerialFrame_Add_RX(SerialFrame_t *frame,
                       const char    *name,
                       void          *data_ptr,
                       SF_Type_t      type)
{
    if (frame->rx_count >= SF_MAX_FIELDS) return -1;

    uint8_t sz = TYPE_SIZE[type];
    if ((frame->rx_frame_size + sz) > SF_MAX_FRAME_BYTES) return -1;

    SF_Field_t *f  = &frame->rx_fields[frame->rx_count];
    f->name        = name;
    f->type        = type;
    f->size        = sz;
    f->data_ptr    = data_ptr;
    /* FIX: same offset logic — fields pack sequentially after header */
    f->byte_offset = frame->rx_frame_size - 1;

    frame->rx_frame_size += sz;
    frame->rx_count++;
    return 0;
}

/* =============================================================================
 *  SerialFrame_Transmit
 *
 *  FIX: Guard against calling HAL_UART_Transmit_DMA while a previous
 *       transfer is still in progress (returns HAL_BUSY and drops the frame).
 *       Check huart->gState before transmitting.
 * =============================================================================
 */
void SerialFrame_Transmit(SerialFrame_t *frame)
{
    /* FIX: Skip if previous DMA TX is still running — prevents HAL_BUSY drop */
    if (frame->huart->gState != HAL_UART_STATE_READY) return;

    /* Write header */
    frame->tx_buf[0] = frame->header;

    /* Pack each TX field at its pre-computed byte offset */
    for (uint8_t i = 0; i < frame->tx_count; i++) {
        SF_Field_t *f = &frame->tx_fields[i];
        memcpy(&frame->tx_buf[f->byte_offset], f->data_ptr, f->size);
    }

    /* Write terminator at the last byte */
    frame->tx_buf[frame->tx_frame_size - 1] = frame->terminator;

    /* Non-blocking DMA transmit */
    HAL_UART_Transmit_DMA(frame->huart, frame->tx_buf, frame->tx_frame_size);
}

/* =============================================================================
 *  SerialFrame_Receive
 *  Called from HAL_UART_RxCpltCallback — validates and unpacks the frame,
 *  then re-arms the DMA receiver.
 *  Also call once at startup to arm the first DMA receive.
 * =============================================================================
 */
void SerialFrame_Receive(SerialFrame_t *frame, UART_HandleTypeDef *huart)
{
    if (huart != frame->huart) return;

    /* Validate header and terminator before unpacking */
    if (frame->rx_buf[0]                        == frame->header &&
        frame->rx_buf[frame->rx_frame_size - 1] == frame->terminator)
    {
        for (uint8_t i = 0; i < frame->rx_count; i++) {
            SF_Field_t *f = &frame->rx_fields[i];
            memcpy(f->data_ptr, &frame->rx_buf[f->byte_offset], f->size);
        }
    }

    /* Re-arm DMA receiver for the next frame */
    HAL_UART_Receive_DMA(frame->huart, frame->rx_buf, frame->rx_frame_size);
}
