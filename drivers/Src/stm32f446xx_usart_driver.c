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
#include "stm32f446xx_rcc_driver.h"
#include "utils.h"

#define CR1_UE (13u)
#define CR1_M (12u)
#define CR1_PCE (10u)
#define CR1_PS (9u)
#define CR1_TE (3u)
#define CR1_RE (2u)

#define CR2_STOP (12u)

#define CR3_CTSE (9u)
#define CR3_RTSE (8u)

typedef struct
{
    volatile uint16_t SR;
    uint16_t _reserved_0;
    volatile uint16_t DR;
    uint16_t _reserved_1;
    volatile uint16_t BRR;
    uint16_t _reserved_2;
    volatile uint16_t CR1;
    uint16_t _reserved_3;
    volatile uint16_t CR2;
    uint16_t _reserved_4;
    volatile uint16_t CR3;
    uint16_t _reserved_5;
    volatile uint16_t GTPR;
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

static void set_parity(const usart_cfg_t *p_cfg);
static void set_mode(const usart_cfg_t *p_cfg);

void usart_init(const usart_cfg_t *p_cfg)
{
    rcc_set_usart_peripheral_clock_enabled(p_cfg->bus, true);
    utils_set_bit_by_position_u16(&gp_registers[p_cfg->bus]->CR1, CR1_M, p_cfg->word_length == USART_WORD_LENGTH_9_BITS);
    set_parity(p_cfg);
    set_mode(p_cfg);

    /* Clears STOP bits to avoid previous configurations */
    utils_set_bits_u16(&gp_registers[p_cfg->bus]->CR2, (uint16_t)(USART_NUM_STOP_BITS_1_5 << CR2_STOP), false);
    utils_set_bits_u16(&gp_registers[p_cfg->bus]->CR2, (uint16_t)(p_cfg->num_stop_bits << CR2_STOP), true);

    utils_set_bit_by_position_u16(&gp_registers[p_cfg->bus]->CR3, CR3_CTSE, p_cfg->b_cts_hardware_flow_control_enabled);
    utils_set_bit_by_position_u16(&gp_registers[p_cfg->bus]->CR3, CR3_RTSE, p_cfg->b_rts_hardware_flow_control_enabled);
}

void usart_set_peripheral_enabled(usart_bus_t bus, bool b_enabled)
{
    utils_set_bit_by_position_u16(&gp_registers[bus]->CR1, CR1_UE, true);
}

static void set_parity(const usart_cfg_t *p_cfg)
{
    switch (p_cfg->parity)
    {
    case USART_PARITY_NONE:
        break;
    case USART_PARITY_EVEN:
    case USART_PARITY_ODD:
        utils_set_bit_by_position_u16(&gp_registers[p_cfg->bus]->CR1, CR1_PCE, true);
        utils_set_bit_by_position_u16(&gp_registers[p_cfg->bus]->CR1, CR1_PS, p_cfg->parity == USART_PARITY_ODD);
        break;
    default:
        break;
    }
}

static void set_mode(const usart_cfg_t *p_cfg)
{
    bool b_tx_enabled = true;
    bool b_rx_enabled = true;

    switch (p_cfg->mode)
    {
    case USART_MODE_TX:
        b_rx_enabled = false;
        break;

    case USART_MODE_RX:
        b_tx_enabled = false;
        break;

    case USART_MODE_TX_RX:
    default:
        break;
    }
    utils_set_bit_by_position_u16(&gp_registers[p_cfg->bus]->CR1, CR1_TE, b_tx_enabled);
    utils_set_bit_by_position_u16(&gp_registers[p_cfg->bus]->CR1, CR1_RE, b_rx_enabled);
}