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
 * @param p_handle 
 */
void gpio_init(const gpio_handle_t *p_handle)
{
    gpio_enable_peripheral_clock(p_handle->p_reg, true);

    /* TODO: Check if memset */
    /* Configures mode. */
    if (p_handle->cfg.mode <= GPIO_MODE_ANALOG)
    {
        p_handle->p_reg->MODER &= ~(0x3u << (2u * p_handle->cfg.number));
        p_handle->p_reg->MODER |= p_handle->cfg.mode << (2u * p_handle->cfg.number);
    }
    else
    {
        switch (p_handle->cfg.mode)
        {
        case GPIO_MODE_INTERRUPT_FT:
            EXTI->FTSR |= 1u << p_handle->cfg.number;
            EXTI->RTSR &= ~(1u << p_handle->cfg.number);
            break;

        case GPIO_MODE_INTERRUPT_RT:
            EXTI->RTSR |= 1u << p_handle->cfg.number;
            EXTI->FTSR &= ~(1u << p_handle->cfg.number);
            break;

        case GPIO_MODE_INTERRUPT_RFT:
            EXTI->RTSR |= 1u << p_handle->cfg.number;
            EXTI->FTSR |= 1u << p_handle->cfg.number;
            break;

        default:
            break;
        }
        /* Configures the GPIO port selection in SYSCFG_EXTICR */

        SYSCFG_PCLK_EN();
        size_t index = p_handle->cfg.number / 4u;
        uint8_t section = p_handle->cfg.number % 4u;
        SYSCFG->EXTICR[index] = (uint32_t)Driver_gpio_address_to_code(p_handle->p_reg) << (section * 4u);
        EXTI->IMR |= 1u << p_handle->cfg.number;
    }

    /* Configures speed. */
    p_handle->p_reg->OSPEEDER &= ~(0x3u << (2 * p_handle->cfg.number));
    p_handle->p_reg->OSPEEDER |= p_handle->cfg.speed << (2 * p_handle->cfg.number);

    /* Configures pupd. */
    p_handle->p_reg->PUPDR &= ~(0x3u << (2 * p_handle->cfg.number));
    p_handle->p_reg->PUPDR |= p_handle->cfg.pull_mode << (2 * p_handle->cfg.number);

    /* Configures opt type. */
    p_handle->p_reg->OTYPER &= ~(0x1u << p_handle->cfg.number);
    p_handle->p_reg->OTYPER |= (uint32_t)p_handle->cfg.out_type << (uint32_t)p_handle->cfg.number;

    /* Configures alternate function */
    if (p_handle->cfg.mode == GPIO_MODE_ALT_FN)
    {
        const uint32_t temp1 = p_handle->cfg.number / 8u;
        const uint32_t tmp2 = p_handle->cfg.number % 8u;
        p_handle->p_reg->AFR[temp1] &= ~(0xFu << (4 * tmp2));
        p_handle->p_reg->AFR[temp1] |= p_handle->cfg.alt_fun_mode << (4 * tmp2);
    }
}

/**
 * @brief 
 * 
 * @param gpiox 
 */
void gpio_deinit(gpio_reg_t *p_reg)
{
    gpio_reset_t port = Gpio_reset_port_a;

    if (p_reg == GPIOA)
    {
    }
    else if (p_reg == GPIOB)
    {
        port = Gpio_reset_port_b;
    }
    else if (p_reg == GPIOC)
    {
        port = Gpio_reset_port_c;
    }
    else if (p_reg == GPIOD)
    {
        port = Gpio_reset_port_d;
    }
    else if (p_reg == GPIOE)
    {
        port = Gpio_reset_port_e;
    }
    else if (p_reg == GPIOF)
    {
        port = Gpio_reset_port_f;
    }
    else if (p_reg == GPIOG)
    {
        port = Gpio_reset_port_g;
    }
    else if (p_reg == GPIOH)
    {
        port = Gpio_reset_port_h;
    }
    GPIOX_REG_RESET(port);
}

/**
 * @brief 
 * 
 * @param gpiox 
 * @param b_enable 
 */
void gpio_enable_peripheral_clock(gpio_reg_t *p_reg, bool b_enable)
{
    if (b_enable)
    {
        if (p_reg == GPIOA)
        {
            GPIOA_PCLCK_EN();
        }
        else if (p_reg == GPIOB)
        {
            GPIOB_PCLCK_EN();
        }
        else if (p_reg == GPIOC)
        {
            GPIOC_PCLCK_EN();
        }
        else if (p_reg == GPIOD)
        {
            GPIOD_PCLCK_EN();
        }
        else if (p_reg == GPIOE)
        {
            GPIOE_PCLCK_EN();
        }
        else if (p_reg == GPIOF)
        {
            GPIOF_PCLCK_EN();
        }
        else if (p_reg == GPIOG)
        {
            GPIOG_PCLCK_EN();
        }
        else if (p_reg == GPIOH)
        {
            GPIOH_PCLCK_EN();
        }
    }
    else
    {
        if (p_reg == GPIOA)
        {
            GPIOA_PCLCK_DI();
        }
        else if (p_reg == GPIOB)
        {
            GPIOB_PCLCK_DI();
        }
        else if (p_reg == GPIOC)
        {
            GPIOC_PCLCK_DI();
        }
        else if (p_reg == GPIOD)
        {
            GPIOD_PCLCK_DI();
        }
        else if (p_reg == GPIOE)
        {
            GPIOE_PCLCK_DI();
        }
        else if (p_reg == GPIOF)
        {
            GPIOF_PCLCK_DI();
        }
        else if (p_reg == GPIOG)
        {
            GPIOG_PCLCK_DI();
        }
        else if (p_reg == GPIOH)
        {
            GPIOH_PCLCK_DI();
        }
    }
}

/**
 * @brief 
 * 
 * @param p_reg 
 * @param pin 
 * @return gpio_button_state_t 
 */
gpio_button_state_t gpio_read_pin(const gpio_handle_t *p_handle)
{
    return (gpio_button_state_t)((p_handle->p_reg->IDR >> p_handle->cfg.number) & 1u);
}

/**
 * @brief 
 * 
 * @param p_reg 
 * @return uint16_t 
 */
uint16_t gpio_read_port(gpio_reg_t *p_reg)
{
    return (uint16_t)(p_reg->IDR);
}

/**
 * @brief 
 * 
 * @param p_handle 
 * @param value 
 */
void gpio_write_pin(gpio_handle_t *p_handle, gpio_pin_status_t value)
{
    if (value == GPIO_PIN_STATUS_SET)
    {
        p_handle->p_reg->ODR |= 1u << p_handle->cfg.number;
    }
    else
    {
        p_handle->p_reg->ODR &= ~(1u << p_handle->cfg.number);
    }
}

/**
 * @brief 
 * 
 * @param gpiox 
 * @param value 
 */
void gpio_write_port(gpio_reg_t *p_reg, uint16_t value)
{
    p_reg->ODR = value;
}

/**
 * @brief 
 * 
 * @param gpiox 
 * @param pin 
 */
void gpio_toggle_pin(gpio_handle_t *p_handle)
{
    p_handle->p_reg->ODR ^= 1u << p_handle->cfg.number;
}

/**
 * @brief 
 * 
 * @param irq_number 
 * @param priority 
 * @param b_enable 
 */
void gpio_config_irq(irq_number_t irq_number, bool b_enable)
{
    if (b_enable == true)
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

void gpio_config_irq_priority(irq_number_t irq_number, nvic_irq_priority_t priority)
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
void gpio_irq_handling(gpio_pin_t pin)
{
    if (EXTI->PR & (1 << pin))
    {
        /* Clears */
        EXTI->PR |= 1 << pin;
    }
}
