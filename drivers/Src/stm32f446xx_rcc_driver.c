/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Aug 12, 2020
 *      Author: joaquin
 */

#include "stm32f446xx_rcc_driver.h"

#include <stdint.h>

#include "stm32f446xx.h"
#include "utils.h"

#define MIN_AHB_PRESCALER (8u)
#define MIN_APB1_PRESCALER (4u)

#define CFGR_PPRE2 (13u)
#define CFGR_PPRE1 (10u)

/* RCC system clock switch status */
typedef enum
{
    SWS_HSI = 0u,
    SWS_HSE = 1u,
    SWS_PLL = 2u,
    SWS_PLL_R = 3u,
} sws_t;

typedef struct
{
    uint8_t bit_position;
    rcc_apb_t apb_idx;
} helper_t;

static const helper_t g_usart_helpers[NUM_USART_BUSES] = {
    [USART_BUS_1] = {.bit_position = 4u, .apb_idx = RCC_APB_2},
    [USART_BUS_2] = {.bit_position = 17u, .apb_idx = RCC_APB_1},
    [USART_BUS_3] = {.bit_position = 18u, .apb_idx = RCC_APB_1},
    [UART_BUS_4] = {.bit_position = 19u, .apb_idx = RCC_APB_1},
    [UART_BUS_5] = {.bit_position = 20u, .apb_idx = RCC_APB_1},
    [USART_BUS_6] = {.bit_position = 5u, .apb_idx = RCC_APB_2},
};

static inline uint8_t get_apb_prescaler(rcc_apb_t apb);
static inline uint16_t get_ahb_prescaler(void);
static inline uint32_t get_system_clock(void);

uint32_t rcc_get_pclk(rcc_apb_t apb)
{
    return (get_system_clock() / (uint32_t)get_ahb_prescaler()) / (uint32_t)get_apb_prescaler(apb);
}

void rcc_set_i2c_peripheral_clock_enabled(i2c_bus_t bus, bool b_enabled)
{
    const uint8_t bit_positions[I2C_BUS_TOTAL] = {
        [I2C_BUS_1] = 21u,
        [I2C_BUS_2] = 22u,
        [I2C_BUS_3] = 23u,
    };
    utils_set_bit_by_position_u32(&RCC->APBENR[RCC_APB_1], bit_positions[bus], b_enabled);
}

void rcc_set_usart_peripheral_clock_enabled(usart_bus_t bus, bool b_enabled)
{
    utils_set_bit_by_position_u32(&RCC->APBENR[g_usart_helpers[bus].apb_idx], g_usart_helpers[bus].bit_position, b_enabled);
}

void rcc_reset_usart(usart_bus_t bus)
{
    utils_set_bit_by_position_u32(&RCC->APB_RSTR[g_usart_helpers[bus].apb_idx], g_usart_helpers[bus].bit_position, true);
    utils_set_bit_by_position_u32(&RCC->APB_RSTR[g_usart_helpers[bus].apb_idx], g_usart_helpers[bus].bit_position, false);
}

static inline uint32_t get_system_clock(void)
{
    const uint8_t clock_source = ((RCC->CFGR >> 2u) & 0x3u);
    uint32_t system_clock = 0u;
    switch (clock_source)
    {
    case SWS_HSI:
        system_clock = 16000000u;
        break;
    case SWS_HSE:
        system_clock = 8000000u;
        break;
    case SWS_PLL:
        break;
    case SWS_PLL_R:
        break;
    default:
        break;
    }

    return system_clock;
}

static inline uint16_t get_ahb_prescaler(void)
{
    const uint16_t AHB_PRESCALERS[MIN_AHB_PRESCALER] = {2u, 4u, 8u, 16u, 64u, 128u, 256u, 512u};
    const uint8_t hpre = (RCC->CFGR >> 4u) & 0xFu;
    return hpre < MIN_AHB_PRESCALER ? 1u : AHB_PRESCALERS[hpre - MIN_AHB_PRESCALER];
}

static inline uint8_t get_apb_prescaler(rcc_apb_t apb)
{
    const uint8_t APB1_PRESCALERS[MIN_APB1_PRESCALER] = {2u, 4u, 8u, 16u};
    const uint32_t bit_position = RCC_APB_1 == apb ? CFGR_PPRE1 : CFGR_PPRE2;
    const uint8_t ppre = (RCC->CFGR >> bit_position) & 0x7u;

    return ppre < MIN_APB1_PRESCALER ? 1u : APB1_PRESCALERS[ppre - MIN_APB1_PRESCALER];
}