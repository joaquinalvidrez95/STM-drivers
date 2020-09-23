/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Sep 16, 2020
 *      Author: joaquin.alvidrez
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "utils.h"

typedef enum
{
    USART_BUS_1,
    USART_BUS_2,
    USART_BUS_3,
    UART_BUS_4,
    UART_BUS_5,
    USART_BUS_6,
    NUM_USART_BUSES,
} usart_bus_t;

typedef enum
{
    USART_MODE_TX,
    USART_MODE_RX,
    USART_MODE_TX_RX,
} usart_mode_t;

typedef enum
{
    USART_PARITY_NONE,
    USART_PARITY_EVEN,
    USART_PARITY_ODD,
} usart_parity_t;

typedef enum
{
    USART_BAUD_1200 = 1200,
    USART_BAUD_2400 = 2400,
    USART_BAUD_9600 = 9600,
    USART_BAUD_19200 = 19200,
    USART_BAUD_38400 = 38400,
    USART_BAUD_57600 = 57600,
    USART_BAUD_115200 = 115200,
    USART_BAUD_230400 = 230400,
    USART_BAUD_460800 = 460800,
    USART_BAUD_921600 = 921600,
    USART_BAUD_2M = 2000000,
    USART_BAUD_3M = 3000000,
} usart_baud_t;

typedef enum
{
    USART_NUM_STOP_BITS_1 = 0,
    USART_NUM_STOP_BITS_0_5 = 1,
    USART_NUM_STOP_BITS_2 = 2,
    USART_NUM_STOP_BITS_1_5 = 3,
} usart_num_stop_bits_t;

typedef enum
{
    USART_WORD_LENGTH_8_BITS,
    USART_WORD_LENGTH_9_BITS,
} usart_word_length_t;

typedef struct
{
    usart_bus_t bus;
    usart_mode_t mode;
    usart_baud_t baud;
    usart_num_stop_bits_t num_stop_bits;
    usart_word_length_t word_length;
    usart_parity_t parity;
    bool b_cts_hardware_flow_control_enabled;
    bool b_rts_hardware_flow_control_enabled;
} usart_cfg_t;

typedef struct
{
    uint8_t *const p_buffer;
    const size_t size;
} usart_msg_t;

typedef struct
{
    usart_msg_t *p_msg;
} usart_irq_mgr_t;

typedef struct
{
    const usart_cfg_t cfg;
    usart_irq_mgr_t irq_mgr;
} usart_handle_t;

void usart_init(const usart_cfg_t *p_cfg);
void usart_set_peripheral_enabled(usart_bus_t bus, bool b_enabled);

void usart_transmit(usart_handle_t *p_handle, usart_msg_t *p_msg, utils_mechanism_t mechanism);
void usart_receive(usart_handle_t *p_handle, usart_msg_t *p_msg, utils_mechanism_t mechanism);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
