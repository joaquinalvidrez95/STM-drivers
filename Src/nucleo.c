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
#include "stm32f446xx_usart_driver.h"

#define ADDRESS_I2C (0x69u)

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

void nucleo_init_i2c(i2c_interrupt_callback_t callback)
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

    g_i2c_cfg.interrupt_cb = callback;
    i2c_init(&g_i2c_cfg);
    i2c_set_peripheral_enabled(g_i2c_cfg.bus, true);
    i2c_set_ack(g_i2c_cfg.bus, I2C_ACK_CONTROL_ENABLED);
}

void nucleo_init_usart(usart_cfg_t *p_cfg)
{
    gpio_handle_t pin = {
        .p_reg = GPIOA,
        .cfg = {
            .mode = GPIO_MODE_ALT_FN,
            .out_type = GPIO_OUT_TYPE_PUSH_PULL,
            .pull_mode = GPIO_PULL_MODE_UP,
            .alt_fun_mode = GPIO_ALTERNATE_FUNCTION_7,
            /* D1 */
            .number = GPIO_PIN_2,
            .speed = GPIO_SPEED_FAST,
        },
    };

    /* Tx */
    gpio_init(&pin);

    /* Rx */
    /* D0 */
    pin.cfg.number = GPIO_PIN_3;
    gpio_init(&pin);

    p_cfg->baud = USART_BAUD_115200;
    p_cfg->bus = USART_BUS_2;
    p_cfg->mode = USART_MODE_TX;
    p_cfg->b_cts_hardware_flow_control_enabled = false;
    p_cfg->b_rts_hardware_flow_control_enabled = false;
    p_cfg->num_stop_bits = USART_NUM_STOP_BITS_1;
    p_cfg->word_length = USART_WORD_LENGTH_8_BITS;
    p_cfg->parity = USART_PARITY_NONE;

    usart_init(p_cfg);
    usart_set_peripheral_enabled(p_cfg->bus, true);
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