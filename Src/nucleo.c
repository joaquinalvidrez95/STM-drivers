/*
 * nucleo.c
 *
 *  Created on: Aug 22, 2020
 *      Author: joaquin
 */

#include "nucleo.h"

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"

const static gpio_handle_t g_button = {
    .p_reg = GPIOC,
    .cfg.number = GPIO_PIN_13,
    .cfg.mode = GPIO_MODE_IN,
    .cfg.speed = GPIO_SPEED_FAST,
    .cfg.pull_mode = GPIO_PULL_MODE_NONE,
};

void nucleo_init(void)
{
    gpio_init(&g_button);
}

bool nucleo_is_button_pressed(void)
{
    return GPIO_BUTTON_STATE_HIGH == gpio_read_pin(&g_button);
}