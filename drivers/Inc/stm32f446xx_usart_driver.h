/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Sep 16, 2020
 *      Author: joaquin.alvidrez
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include <stdint.h>

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
    USART_MODE,
} usart_mode_t;

typedef enum
{
    USART_PARITY,
} usart_parity_t;

typedef struct
{
    usart_bus_t bus;
    usart_mode_t mode;
    uint32_t baud;
    uint8_t num_stop_bits;
    uint8_t word_length;
    usart_parity_t parity;
    uint8_t hardware_flow_control;
} usart_cfg_t;

typedef struct
{
    const usart_cfg_t cfg;
} usart_handler_t;

void usart_init(const usart_cfg_t *p_cfg);

#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
