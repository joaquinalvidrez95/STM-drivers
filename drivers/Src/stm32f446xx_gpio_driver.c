/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: May 19, 2020
 *      Author: joaquin
 */

#include "stm32f446xx_gpio_driver.h"
#include <stddef.h>

/**
 * @brief 
 * 
 * @param handle 
 */
void Gpio_init(Gpio_handle_t *handle)
{
    Gpio_peripheral_clock_control(handle->reg, true);

    /* TODO: Check if memset */
    /* Configures mode. */
    if (handle->pin_config.mode <= GPIO_MODE_ANALOG)
    {
        handle->reg->MODER &= ~(0x3u << (2 * handle->pin_config.number));
        handle->reg->MODER |= handle->pin_config.mode << (2 * handle->pin_config.number);
    }
    else
    {
        switch (handle->pin_config.mode)
        {
        case GPIO_MODE_INTERRUPT_FT:
            EXTI->FTSR |= 1u << handle->pin_config.number;
            EXTI->RTSR &= ~(1u << handle->pin_config.number);
            break;

        case GPIO_MODE_INTERRUPT_RT:
            EXTI->RTSR |= 1u << handle->pin_config.number;
            EXTI->FTSR &= ~(1u << handle->pin_config.number);
            break;

        case GPIO_MODE_INTERRUPT_RFT:
            EXTI->RTSR |= 1u << handle->pin_config.number;
            EXTI->FTSR |= 1u << handle->pin_config.number;
            break;

        default:
            break;
        }
        /* Configures the GPIO port selection in SYSCFG_EXTICR */

        SYSCFG_PCLK_EN();
        size_t index = handle->pin_config.number / 4u;
        uint8_t section = handle->pin_config.number % 4u;
        SYSCFG->EXTICR[index] = (uint32_t)Driver_gpio_address_to_code(handle->reg) << (section * 4u);
        EXTI->IMR |= 1u << handle->pin_config.number;
    }

    /* Configures speed. */
    handle->reg->OSPEEDER &= ~(0x3u << (2 * handle->pin_config.number));
    handle->reg->OSPEEDER |= handle->pin_config.speed << (2 * handle->pin_config.number);

    /* Configures pupd. */
    handle->reg->PUPDR &= ~(0x3u << (2 * handle->pin_config.number));
    handle->reg->PUPDR |= handle->pin_config.pull_mode << (2 * handle->pin_config.number);

    /* Configures opt type. */
    handle->reg->OTYPER &= ~(0x1u << handle->pin_config.number);
    handle->reg->OTYPER |= handle->pin_config.out_type << handle->pin_config.out_type;

    /* Configures alternate function */
    if (handle->pin_config.mode == GPIO_MODE_ALT_FN)
    {
        const uint32_t temp1 = handle->pin_config.number / 8u;
        const uint32_t tmp2 = handle->pin_config.number % 8u;
        handle->reg->AFR[temp1] &= ~(0xFu << (4 * tmp2));
        handle->reg->AFR[temp1] |= handle->pin_config.alt_fun_mode << (4 * tmp2);
    }
}

/**
 * @brief 
 * 
 * @param gpiox 
 */
void Gpio_deinit(Gpio_reg_t *reg)
{
    Gpio_reset_t port = Gpio_reset_port_a;

    if (reg == GPIOA)
    {
    }
    else if (reg == GPIOB)
    {
        port = Gpio_reset_port_b;
    }
    else if (reg == GPIOC)
    {
        port = Gpio_reset_port_c;
    }
    else if (reg == GPIOD)
    {
        port = Gpio_reset_port_d;
    }
    else if (reg == GPIOE)
    {
        port = Gpio_reset_port_e;
    }
    else if (reg == GPIOF)
    {
        port = Gpio_reset_port_f;
    }
    else if (reg == GPIOG)
    {
        port = Gpio_reset_port_g;
    }
    else if (reg == GPIOH)
    {
        port = Gpio_reset_port_h;
    }
    GPIOX_REG_RESET(port);
}

/**
 * @brief 
 * 
 * @param gpiox 
 * @param enable 
 */
void Gpio_peripheral_clock_control(Gpio_reg_t *reg, bool enable)
{
    if (enable == En_status_enable)
    {
        if (reg == GPIOA)
        {
            GPIOA_PCLCK_EN();
        }
        else if (reg == GPIOB)
        {
            GPIOB_PCLCK_EN();
        }
        else if (reg == GPIOC)
        {
            GPIOC_PCLCK_EN();
        }
        else if (reg == GPIOD)
        {
            GPIOD_PCLCK_EN();
        }
        else if (reg == GPIOE)
        {
            GPIOE_PCLCK_EN();
        }
        else if (reg == GPIOF)
        {
            GPIOF_PCLCK_EN();
        }
        else if (reg == GPIOG)
        {
            GPIOG_PCLCK_EN();
        }
        else if (reg == GPIOH)
        {
            GPIOH_PCLCK_EN();
        }
    }
    else
    {
        if (reg == GPIOA)
        {
            GPIOA_PCLCK_DI();
        }
        else if (reg == GPIOB)
        {
            GPIOB_PCLCK_DI();
        }
        else if (reg == GPIOC)
        {
            GPIOC_PCLCK_DI();
        }
        else if (reg == GPIOD)
        {
            GPIOD_PCLCK_DI();
        }
        else if (reg == GPIOE)
        {
            GPIOE_PCLCK_DI();
        }
        else if (reg == GPIOF)
        {
            GPIOF_PCLCK_DI();
        }
        else if (reg == GPIOG)
        {
            GPIOG_PCLCK_DI();
        }
        else if (reg == GPIOH)
        {
            GPIOH_PCLCK_DI();
        }
    }
}

/**
 * @brief 
 * 
 * @param reg 
 * @param pin 
 * @return Gpio_button_state_e 
 */
Gpio_button_state_e Gpio_read_from_input_pin(Gpio_handle_t *handle)
{
    return (Gpio_button_state_e)((handle->reg->IDR >> handle->pin_config.number) & 0x00000001u);
}

/**
 * @brief 
 * 
 * @param reg 
 * @return uint16_t 
 */
uint16_t Gpio_read_from_input_port(Gpio_reg_t *reg)
{
    return (uint16_t)(reg->IDR);
}

/**
 * @brief 
 * 
 * @param handle 
 * @param value 
 */
void Gpio_write_to_pin(Gpio_handle_t *handle, Gpio_pin_status_t value)
{
    if (value == Gpio_pin_status_set)
    {
        handle->reg->ODR |= 1 << handle->pin_config.number;
    }
    else
    {
        handle->reg->ODR &= ~(1 << handle->pin_config.number);
    }
}

/**
 * @brief 
 * 
 * @param gpiox 
 * @param value 
 */
void Gpio_write_to_output_port(Gpio_reg_t *reg, uint16_t value)
{
    reg->ODR = value;
}

/**
 * @brief 
 * 
 * @param gpiox 
 * @param pin 
 */
void Gpio_toggle_pin(Gpio_handle_t *handle)
{
    handle->reg->ODR ^= 1u << handle->pin_config.number;
}

/**
 * @brief 
 * 
 * @param irq_number 
 * @param priority 
 * @param enable 
 */
void Gpio_config_irq(Irq_number_t irq_number, bool enable)
{
    if (enable == true)
    {
        if (irq_number <= 31u)
        {
            *NVIC_ISER0 |= 1u << irq_number;
        }
        else if ((irq_number > 31u) && (irq_number < 64u))
        {
            *NVIC_ISER1 |= 1u << (irq_number % 32u);
        }
        else if ((irq_number >= 64u) && (irq_number < 96u))
        {
            *NVIC_ISER3 |= 1u << (irq_number % 64u);
        }
    }
    else
    {
        if (irq_number <= 31u)
        {
            *NVIC_ICER0 |= 1u << irq_number;
        }
        else if ((irq_number > 31u) && (irq_number < 64u))
        {
            *NVIC_ICER1 |= 1u << (irq_number % 32u);
        }
        else if ((irq_number >= 64u) && (irq_number < 96u))
        {
            *NVIC_ICER3 |= 1u << (irq_number % 64u);
        }
    }
}

void Gpio_config_irq_priority(Irq_number_t irq_number, Nvic_irq_priority_t priority)
{
    const uint8_t index = irq_number / 4u;
    const uint8_t section = irq_number % 4u;
    const uint8_t shift_amount = (8u * section) + (8u - NO_PR_BITS_IMPLEMENTED);
    NVIC_PR_BASE_ADDR[index] |= (uint32_t)priority << shift_amount;
}

/**
 * @brief 
 * 
 * @param pin 
 */
void Gpio_irq_handling(Gpio_pin_e pin)
{
    if (EXTI->PR & (1 << pin))
    {
        /* Clears */
        EXTI->PR |= 1 << pin;
    }
}
