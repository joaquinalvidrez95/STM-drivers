#include "stm32f446xx.h"

uint8_t Driver_gpio_address_to_code(gpio_reg_t *p_reg)
{
    return (p_reg == GPIOA) ? 0u : (p_reg == GPIOB) ? 1u : (p_reg == GPIOC) ? 2u : (p_reg == GPIOD) ? 3u : (p_reg == GPIOE) ? 4u : (p_reg == GPIOF) ? 5u : (p_reg == GPIOG) ? 6u : (p_reg == GPIOH) ? 7u : 0u;
}
