/*
 * SerialFrame.h
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  SerialFrame — Binary Serial Framing over UART (DMA-backed)
 * =============================================================================
 *  Builds and parses fixed-layout binary frames:
 *
 *    [ HEADER | field0 | field1 | ... | fieldN | TERMINATOR ]
 *
 *  Features
 *  --------
 *    - Up to 32 TX fields and 32 RX fields per frame.
 *    - Supports 8 data types: uint8, int8, uint16, int16,
 *                             uint32, int32, float, double.
 *    - Non-blocking transmit and receive via DMA.
 *    - Compatible with Teleplot and other serial plotters.
 *
 *  Init Sequence
 *  -------------
 *    SerialFrame_Init(&frame, &hlpuart1, 0x25, 'N');
 *
 *    SerialFrame_Add_TX(&frame, "Position", &pos,      SF_FLOAT);
 *    SerialFrame_Add_TX(&frame, "Mode",     &mode,     SF_INT16);
 *
 *    SerialFrame_Add_RX(&frame, "Setpoint", &setpoint, SF_FLOAT);
 *
 *    SerialFrame_Receive(&frame, &hlpuart1);  // arm first DMA receive once
 *
 *  Transmit
 *  -------------
 *    SerialFrame_Transmit(&frame);
 *
 *  Receive
 *  -------------
 *    SerialFrame_Receive(&frame, huart);
 * =============================================================================
 */

#ifndef INC_SERIALFRAME_H_
#define INC_SERIALFRAME_H_

#include "main.h"
#include <stdint.h>

/* =============================================================================
 *  Configuration
 * =============================================================================
 */
#define SF_MAX_FRAME_BYTES  255U   /* Maximum total frame size in bytes  */
#define SF_MAX_FIELDS        32U   /* Maximum number of fields per frame */

/* =============================================================================
 *  SF_Type_t  —  Supported Field Data Types
 * =============================================================================
 */
typedef enum {
    SF_UINT8   = 0,   /* 1 byte,  unsigned */
    SF_INT8    = 1,   /* 1 byte,  signed   */
    SF_UINT16  = 2,   /* 2 bytes, unsigned */
    SF_INT16   = 3,   /* 2 bytes, signed   */
    SF_UINT32  = 4,   /* 4 bytes, unsigned */
    SF_INT32   = 5,   /* 4 bytes, signed   */
    SF_FLOAT   = 6,   /* 4 bytes, IEEE 754 */
    SF_DOUBLE  = 7    /* 8 bytes, IEEE 754 */
} SF_Type_t;

/* =============================================================================
 *  SF_Field_t  —  Internal Field Descriptor
 *  Do not modify directly.
 * =============================================================================
 */
typedef struct {
    const char  *name;        /* Human-readable label (for debugging)   */
    SF_Type_t    type;        /* Data type enum                         */
    uint8_t      size;        /* Byte size of this field                */
    void        *data_ptr;    /* Pointer to the variable to read/write  */
    uint8_t      byte_offset; /* Byte position in the frame buffer      */
} SF_Field_t;

/* =============================================================================
 *  SerialFrame_t  —  Frame Handle
 * =============================================================================
 */
typedef struct {
    UART_HandleTypeDef *huart;       /* UART peripheral to use              */
    uint8_t             header;      /* Frame start byte  (e.g. 0x25 = '%') */
    uint8_t             terminator;  /* Frame end byte    (e.g. 'N' = 0x4E) */

    SF_Field_t tx_fields[SF_MAX_FIELDS];
    SF_Field_t rx_fields[SF_MAX_FIELDS];

    uint8_t tx_count;          /* Number of registered TX fields        */
    uint8_t rx_count;          /* Number of registered RX fields        */

    uint8_t tx_frame_size;     /* Total TX frame size in bytes          */
    uint8_t rx_frame_size;     /* Total RX frame size in bytes          */

    uint8_t tx_buf[SF_MAX_FRAME_BYTES];
    uint8_t rx_buf[SF_MAX_FRAME_BYTES];
} SerialFrame_t;

/* =============================================================================
 *  API
 * =============================================================================
 */

/* -----------------------------------------------------------------------------
 *  SerialFrame_Init
 *  Initialise a frame handle and clear all buffers.
 *
 *  Parameters
 *  ----------
 *  frame       Pointer to SerialFrame_t instance.
 *  huart       UART handle to use for TX/RX.
 *  header      Frame start byte (must be unique — must not appear in data).
 *  terminator  Frame end byte.
 * ----------------------------------------------------------------------------- */
void SerialFrame_Init(SerialFrame_t *frame,
                      UART_HandleTypeDef *huart,
                      uint8_t header,
                      uint8_t terminator);

/* -----------------------------------------------------------------------------
 *  SerialFrame_Add_TX
 *  Register one variable for transmission.
 *  Call once per TX variable at startup, in the order fields should appear.
 *
 *  Parameters
 *  ----------
 *  frame     Pointer to SerialFrame_t instance.
 *  name      Label string (for debugging — not sent over the wire).
 *  data_ptr  Pointer to the variable to transmit.
 *  type      Data type (SF_FLOAT, SF_INT16, etc.).
 *
 *  Returns   0 on success, -1 if frame is full or frame size exceeded.
 * ----------------------------------------------------------------------------- */
int SerialFrame_Add_TX(SerialFrame_t *frame,
                       const char    *name,
                       void          *data_ptr,
                       SF_Type_t      type);

/* -----------------------------------------------------------------------------
 *  SerialFrame_Add_RX
 *  Register one variable to be filled from incoming frames.
 *  Call once per RX variable at startup, in the order fields appear.
 *
 *  Parameters
 *  ----------
 *  frame     Pointer to SerialFrame_t instance.
 *  name      Label string (for debugging).
 *  data_ptr  Pointer to the variable to write received data into.
 *  type      Data type.
 *
 *  Returns   0 on success, -1 if frame is full or frame size exceeded.
 * ----------------------------------------------------------------------------- */
int SerialFrame_Add_RX(SerialFrame_t *frame,
                       const char    *name,
                       void          *data_ptr,
                       SF_Type_t      type);

/* -----------------------------------------------------------------------------
 *  SerialFrame_Transmit
 *  Build and send one TX frame over DMA.
 *  Non-blocking — returns immediately without waiting for completion.
 *  Call from your periodic timer ISR or task.
 * ----------------------------------------------------------------------------- */
void SerialFrame_Transmit(SerialFrame_t *frame);

/* -----------------------------------------------------------------------------
 *  SerialFrame_Receive
 *  Parse the last received frame and re-arm the DMA receiver.
 *  Call from HAL_UART_RxCpltCallback(), passing the huart argument.
 *  Also call once at startup to arm the very first DMA receive.
 *
 *  Parameters
 *  ----------
 *  frame  Pointer to SerialFrame_t instance.
 *  huart  The huart pointer from the callback — used to match the peripheral.
 * ----------------------------------------------------------------------------- */
void SerialFrame_Receive(SerialFrame_t *frame, UART_HandleTypeDef *huart);

#endif /* INC_SERIALFRAME_H_ */
