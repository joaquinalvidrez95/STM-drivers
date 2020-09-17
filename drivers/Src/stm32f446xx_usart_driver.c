/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Sep 16, 2020
 *      Author: joaquin.alvidrez
 */

#include "stm32f446xx_usart_driver.h"

#include <stdint.h>
#include <stdbool.h>

#include "stm32f446xx.h"

typedef struct
{
    volatile uint16_t SR;
    uint16_t _reserved_0;
    volatile uint32_t DR;
    uint16_t _reserved_1;
    volatile uint32_t BRR;
    uint16_t _reserved_2;
    volatile uint32_t CR1;
    uint16_t _reserved_3;
    volatile uint32_t CR2;
    uint16_t _reserved_4;
    volatile uint32_t CR3;
    uint16_t _reserved_5;
    volatile uint32_t GTPR;
    uint16_t _reserved_6;
} reg_t;

static reg_t *const gp_registers[NUM_USART_BUSES] = {
    [USART_BUS_1] = (reg_t *)USART1_BASEADDR,
    [USART_BUS_2] = (reg_t *)USART2_BASEADDR,
    [USART_BUS_3] = (reg_t *)USART3_BASEADDR,
    [UART_BUS_4] = (reg_t *)UART4_BASEADDR,
    [UART_BUS_5] = (reg_t *)UART5_BASEADDR,
    [USART_BUS_6] = (reg_t *)USART6_BASEADDR,
};

void usart_init(const usart_cfg_t *p_cfg)
{
    
}