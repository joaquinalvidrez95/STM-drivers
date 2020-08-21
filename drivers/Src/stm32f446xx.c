#include "stm32f446xx.h"

uint8_t Driver_gpio_address_to_code(gpio_reg_t *reg)
{
    return (reg == GPIOA) ? 0u : (reg == GPIOB) ? 1u : (reg == GPIOC) ? 2u : (reg == GPIOD) ? 3u : (reg == GPIOE) ? 4u : (reg == GPIOF) ? 5u : (reg == GPIOG) ? 6u : (reg == GPIOH) ? 7u : 0u;
}
