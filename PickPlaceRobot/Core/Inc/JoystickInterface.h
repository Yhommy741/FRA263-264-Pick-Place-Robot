#ifndef JOYSTICK_INTERFACE_H
#define JOYSTICK_INTERFACE_H

#include "main.h"
#include "usart.h"

#define JOY_FRAME_LEN  6
#define JOY_SYNC_BYTE  0xAA

/* ── Command IDs (matches StateMachine.cpp protocol spec) ───────────────── */
#define JOY_CMD_NONE         0x00
#define JOY_CMD_MOVE         0x01
#define JOY_CMD_STOP         0x02
#define JOY_CMD_SET_HOME     0x04
#define JOY_CMD_HOME         0x05
#define JOY_CMD_JOG_VEL_CCW  0x06
#define JOY_CMD_JOG_VEL_CW   0x07
#define JOY_CMD_JOG_STEP_CCW 0x08
#define JOY_CMD_JOG_STEP_CW  0x09
#define JOY_CMD_GRP_UP       0x0A
#define JOY_CMD_GRP_DOWN     0x0B
#define JOY_CMD_GRP_CLOSE    0x0C
#define JOY_CMD_GRP_OPEN     0x0D

/* ── Joystick Instance Structure ────────────────────────────────────────── */
typedef struct {
    UART_HandleTypeDef *huart;               // Pointer to the assigned UART handle
    GPIO_TypeDef       *rs485_de_port;       // GPIO Port for RS485 Driver Enable (e.g., GPIOA)
    uint16_t            rs485_de_pin;        // GPIO Pin for RS485 Driver Enable (e.g., GPIO_PIN_1)
    volatile uint8_t    rx_buf[12];             // Oversized for sync scanning
    volatile uint8_t    isr_flag;            // Raw frame ready notification flag
    uint8_t             parsed_cmd;          // Last successfully parsed command ID
    float               parsed_data;         // Last successfully parsed float data
    /* Debug — watch in Live Expressions */
    uint8_t             dbg_last_raw_cmd;
    float               dbg_last_raw_data;
    uint32_t            dbg_frame_count;
} JoystickInterface_t;

/* ── API ────────────────────────────────────────────────────────────────── */
void    JoystickInterface_Init(JoystickInterface_t *joy, UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin);
void    JoystickInterface_RxCpltCallback(JoystickInterface_t *joy, UART_HandleTypeDef *huart);
uint8_t JoystickInterface_Update(JoystickInterface_t *joy);

/* Getters */
uint8_t JoystickInterface_Get_Command(JoystickInterface_t *joy);
float   JoystickInterface_Get_Data(JoystickInterface_t *joy);

/* Mode Changers for Half-Duplex Bus Control */
void    JoystickInterface_Set_RxMode(JoystickInterface_t *joy);
void    JoystickInterface_Set_TxMode(JoystickInterface_t *joy);

#endif /* JOYSTICK_INTERFACE_H */
