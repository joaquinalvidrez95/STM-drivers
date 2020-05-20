/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: May 19, 2020
 *      Author: joaquin
 */

#include "stm32f446xx_gpio_driver.h"

/**
 * @brief 
 * 
 * @param handle 
 */
void Gpio_init(Gpio_handle_t *handle)
{
    /* TODO: Check if memset */
    /* Configures mode. */
    if (handle->pin_config.mode <= Gpio_mode_analog)
    {
        handle->reg->MODER &= ~(0x3u << (2 * handle->pin_config.number));
        handle->reg->MODER |= handle->pin_config.mode << (2 * handle->pin_config.number);
    }
    else
    {
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
    if (handle->pin_config.mode == Gpio_mode_alt_fn)
    {
        uint32_t temp1 = handle->pin_config.number / 8u;
        uint32_t tmp2 = handle->pin_config.number % 8u;
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
void Gpio_peripheral_clock_control(Gpio_reg_t *reg, uint8_t enable)
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
 * @return Gpio_button_state_t 
 */
Gpio_button_state_t Gpio_read_from_input_pin(Gpio_reg_t *reg, uint8_t pin)
{
    return (Gpio_button_state_t)((reg->IDR >> pin) & 0x00000001u);
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
 * @param gpiox 
 * @param pin 
 * @param value 
 */
void Gpio_write_to_pin(Gpio_reg_t *reg, uint8_t pin, uint8_t value)
{
    if (value == Gpio_pin_status_set)
    {
        reg->ODR |= 1 << pin;
    }
    else
    {
        reg->ODR &= ~(1 << pin);
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
void Gpio_toggle_pin(Gpio_reg_t *reg, uint8_t pin)
{
    reg->ODR ^= 1 << pin;
}

/**
 * @brief 
 * 
 * @param irq_number 
 * @param priority 
 * @param enable 
 */
void Gpio_config_irq(uint8_t irq_number, uint8_t priority, uint8_t enable)
{
}

/**
 * @brief 
 * 
 * @param pin 
 */
void Gpio_irq_handling(uint8_t pin)
{
}
