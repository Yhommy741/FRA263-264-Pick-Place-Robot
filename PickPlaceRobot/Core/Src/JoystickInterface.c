/*
 * JoystickInterface.c
 *
 * Created on: May 2026
 * Author: Yhommy
 *
 * Joystick RS-485 UART driver implementation.
 * Parses 6-byte binary frames (0xAA sync, DMA RX) from USART3.
 * Outputs JoystickCmd_t consumed by TaskManager.
 */

#include "JoystickInterface.h"
#include <string.h>

/* ── Private RS485 Direction Toggles ────────────────────────────────────── */
void JoystickInterface_Set_RxMode(JoystickInterface_t *joy)
{
    if (joy->rs485_de_port != NULL) {
        // Pull DE/RE low to place the transceiver into Listening/Receiver mode
        HAL_GPIO_WritePin(joy->rs485_de_port, joy->rs485_de_pin, GPIO_PIN_RESET);
    }
}

void JoystickInterface_Set_TxMode(JoystickInterface_t *joy)
{
    if (joy->rs485_de_port != NULL) {
        // Drive DE/RE high to place the transceiver into Transmit mode
        HAL_GPIO_WritePin(joy->rs485_de_port, joy->rs485_de_pin, GPIO_PIN_SET);
    }
}

/* ── Public API Implementation ──────────────────────────────────────────── */

/**
 * @brief  Initializes the joystick structure instance for an RS485 bus interface.
 * @param  joy: Pointer to your JoystickInterface_t instance.
 * @param  huart: Pointer to the STM32 HAL UART handle.
 * @param  de_port: GPIO Port assigned to the RS485 DE pin (Pass NULL if using Hardware Flow Control).
 * @param  de_pin: GPIO Pin assigned to the RS485 DE pin (Pass 0 if using Hardware Flow Control).
 */
void JoystickInterface_Init(JoystickInterface_t *joy, UART_HandleTypeDef *huart, GPIO_TypeDef *de_port, uint16_t de_pin)
{
    if (joy == NULL || huart == NULL) return;

    joy->huart         = huart;
    joy->rs485_de_port = de_port;
    joy->rs485_de_pin  = de_pin;
    joy->isr_flag      = 0;
    joy->parsed_cmd    = 0;
    joy->parsed_data   = 0.0f;
    memset((uint8_t *)joy->rx_buf, 0, JOY_FRAME_LEN);

    /* Enforce listening mode on the physical RS485 Transceiver hardware layer */
    JoystickInterface_Set_RxMode(joy);

    /* Start non-blocking interrupt reception */
    HAL_UART_Receive_IT(joy->huart, (uint8_t *)joy->rx_buf, JOY_FRAME_LEN);
}

void JoystickInterface_RxCpltCallback(JoystickInterface_t *joy, UART_HandleTypeDef *huart)
{
    if (joy == NULL || huart == NULL) return;
    if (huart != joy->huart) return;

    joy->isr_flag = 1;
    /* Re-arm handled in JoystickInterface_Update after sync validation */
}

uint8_t JoystickInterface_Update(JoystickInterface_t *joy)
{
    if (joy == NULL) return 0;

    if (joy->isr_flag)
    {
        __disable_irq();
        joy->isr_flag = 0;

        /* Find sync byte 0xAA at position 0 — ensures frame alignment */
        uint8_t temp_cmd   = 0;
        float   temp_data  = 0.0f;
        uint8_t valid      = 0;

        if (joy->rx_buf[0] == JOY_SYNC_BYTE)
        {
            temp_cmd = joy->rx_buf[1];
            memcpy(&temp_data, (const uint8_t *)&joy->rx_buf[2], sizeof(float));
            valid = 1;
        }
        __enable_irq();

        /* Re-arm for next frame */
        HAL_UART_Receive_IT(joy->huart, (uint8_t *)joy->rx_buf, JOY_FRAME_LEN);

        if (valid)
        {
            joy->dbg_last_raw_cmd  = temp_cmd;
            joy->dbg_last_raw_data = temp_data;
            joy->dbg_frame_count++;
            joy->parsed_cmd  = temp_cmd;
            joy->parsed_data = temp_data;
            return 1;
        }
    }

    return 0;
}

uint8_t JoystickInterface_Get_Command(JoystickInterface_t *joy)
{
    if (joy == NULL) return 0;
    return joy->parsed_cmd;
}

float   JoystickInterface_Get_Data(JoystickInterface_t *joy)
{
    if (joy == NULL) return 0.0f;
    return joy->parsed_data;
}
