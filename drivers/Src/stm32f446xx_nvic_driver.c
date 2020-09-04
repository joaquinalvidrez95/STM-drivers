/*
 * stm32f446xx_nvic_driver.c
 *
 *  Created on: Aug 29, 2020
 *      Author: joaquin
 */

#include "stm32f446xx_nvic_driver.h"

#include <stdbool.h>
#include "stm32f446xx.h"

#define NUM_PRIO_BITS_IMPLEMENTED (4u)
#define NUM_BITS_PER_REGISTER (32u)

void nvic_set_irq_enabled(nvic_irq_num_t irq_num, bool b_enabled)
{
    const uint32_t index = (uint32_t)irq_num / NUM_BITS_PER_REGISTER;
    const uint32_t bit_position = (uint32_t)irq_num % NUM_BITS_PER_REGISTER;
    volatile uint32_t *const reg = b_enabled ? NVIC_ISER0 : NVIC_ICER0;
    reg[index] |= 1u << bit_position;
}

void nvic_set_irq_priority(nvic_irq_num_t irq_num, nvic_irq_prio_t prio)
{
    const uint8_t index = irq_num / 4u;
    const uint8_t section = irq_num % 4u;
    const uint8_t shift_amount = (8u * section) + (8u - NUM_PRIO_BITS_IMPLEMENTED);
    NVIC_PR_BASE_ADDR[index] |= (uint32_t)prio << shift_amount;
}
