/*
 * nucleo.c
 *
 *  Created on: Aug 22, 2020
 *      Author: joaquin
 */

#include "nucleo.h"

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"
#include "stm32f446xx_i2c_driver.h"

#define ADDRESS_I2C (0x61u)

const static gpio_handle_t g_button = {
    .p_reg = GPIOC,
    .cfg = {
        .number = GPIO_PIN_13,
        .mode = GPIO_MODE_IN,
        .speed = GPIO_SPEED_FAST,
        .pull_mode = GPIO_PULL_MODE_NONE,
    },
};

void nucleo_init_button(void)
{
    gpio_init(&g_button);
}

void nucleo_init_i2c(i2c_handle_t *p_handle)
{
    gpio_handle_t pin = {
        .p_reg = GPIOB,
        .cfg = {
            .mode = GPIO_MODE_ALT_FN,
            .out_type = GPIO_OUT_TYPE_OPEN_DRAIN,
            .pull_mode = GPIO_PULL_MODE_UP,
            .alt_fun_mode = GPIO_ALTERNATE_FUNCTION_4,
            .number = GPIO_PIN_8,
            .speed = GPIO_SPEED_FAST,
        },
    };

    /* SCL */
    gpio_init(&pin);

    /* SDA */
    pin.cfg.number = GPIO_PIN_9;
    gpio_init(&pin);

    p_handle->p_reg = I2C1;
    p_handle->cfg.ack_control = I2C_ACK_CONTROL_ENABLE;
    p_handle->cfg.device_address = ADDRESS_I2C;
    p_handle->cfg.fm_duty_cycle = I2C_DUTY_2;
    p_handle->cfg.scl_speed = I2C_SCL_SPEED_STANDARD_MODE;

    i2c_init(p_handle);
    i2c_enable_peripheral(p_handle->p_reg, true);
    i2c_set_ack(p_handle->p_reg, I2C_ACK_CONTROL_ENABLE);
}

bool nucleo_is_button_pressed(void)
{
    return GPIO_BUTTON_STATE_LOW == gpio_read_pin(&g_button);
}