#ifndef JOYSTICK_INTERFACE_H
#define JOYSTICK_INTERFACE_H

#include "main.h"
#include "usart.h"

#define JOY_FRAME_LEN  5

/* ── Joystick Instance Structure ────────────────────────────────────────── */
typedef struct {
    UART_HandleTypeDef *huart;               // Pointer to the assigned UART handle
    GPIO_TypeDef       *rs485_de_port;       // GPIO Port for RS485 Driver Enable (e.g., GPIOA)
    uint16_t            rs485_de_pin;        // GPIO Pin for RS485 Driver Enable (e.g., GPIO_PIN_1)
    volatile uint8_t    rx_buf[JOY_FRAME_LEN]; // Dedicated volatile raw RX buffer
    volatile uint8_t    isr_flag;            // Raw frame ready notification flag
    uint8_t             parsed_cmd;          // Last successfully parsed command ID
    float               parsed_data;         // Last successfully parsed float data
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
