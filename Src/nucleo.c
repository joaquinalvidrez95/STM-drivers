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
#include "utils.h"

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

static i2c_cfg_t g_i2c_cfg = {
    .bus = I2C_BUS_1,
    .ack_control = I2C_ACK_CONTROL_ENABLED,
    .device_address = ADDRESS_I2C,
    .fm_duty_cycle = I2C_DUTY_2,
    .scl_speed = I2C_SCL_SPEED_STANDARD_MODE,
};

void nucleo_init_button(void)
{
    gpio_init(&g_button);
}

void nucleo_init_i2c(void)
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

    i2c_init(&g_i2c_cfg);
    i2c_enable_peripheral(g_i2c_cfg.bus, true);
    i2c_set_ack(g_i2c_cfg.bus, I2C_ACK_CONTROL_ENABLED);
}

bool nucleo_is_button_pressed(void)
{
    return GPIO_BUTTON_STATE_LOW == gpio_read_pin(&g_button);
}

void wait_till_button_pressed(void)
{
    while (!nucleo_is_button_pressed())
    {
    }
    utils_delay();
}