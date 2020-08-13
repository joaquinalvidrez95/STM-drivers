/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Aug 12, 2020
 *      Author: joaquin
 */

#include "stm32f446xx_rcc_driver.h"

#include <stdint.h>

#include "stm32f446xx.h"

#define MIN_AHB_PRESCALER (8u)
#define MIN_APB1_PRESCALER (4u)

/* RCC system clock switch status */
typedef enum
{
    RCC_SWS_HSI = 0u,
    RCC_SWS_HSE = 1u,
    RCC_SWS_PLL = 2u,
    RCC_SWS_PLL_R = 3u,
} rcc_sws_t;

static inline uint8_t get_apb_low_speed_prescaler(void);
static inline uint16_t get_ahb_prescaler(void);
static inline uint32_t get_system_clock(void);

uint32_t rcc_get_pclk1(void)
{
    return get_system_clock() / (uint32_t)get_ahb_prescaler() / (uint32_t)get_apb_low_speed_prescaler();
}

static inline uint32_t get_system_clock(void)
{
    const uint8_t clock_source = ((RCC->CFGR >> 2u) & 0x3u);
    uint32_t system_clock = 0u;
    switch (clock_source)
    {
    case RCC_SWS_HSI:
        system_clock = 16000000u;
        break;
    case RCC_SWS_HSE:
        system_clock = 8000000u;
        break;
    case RCC_SWS_PLL:
        break;
    case RCC_SWS_PLL_R:
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

static inline uint8_t get_apb_low_speed_prescaler(void)
{
    const uint8_t APB1_PRESCALERS[MIN_APB1_PRESCALER] = {2u, 4u, 8u, 16u};
    const uint8_t ppre1 = (RCC->CFGR >> 10u) & 0x7u;

    return ppre1 < MIN_APB1_PRESCALER ? 1u : APB1_PRESCALERS[ppre1 - MIN_APB1_PRESCALER];
}