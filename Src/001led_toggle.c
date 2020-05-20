/**
 * @file 001led_toggle.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-19
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "stm32f446xx.h"


void delay()
{
    for (uint32_t i = 0u; i < 500000u; i++);

}

int main()
{
    Gpio_handle_t led;
    Rcc_reg_t *p = RCC;
    led.reg = GPIOA;
    led.pin_config.number = Gpio_pin_5;
    led.pin_config.mode = Gpio_mode_out;
    led.pin_config.speed = Gpio_speed_fast;
    led.pin_config.out_type = Gpio_out_type_push_pull;
    led.pin_config.pull_mode = Gpio_pull_mode_none;

    Gpio_peripheral_clock_control(led.reg, En_status_enable);
    Gpio_init(&led);

    while (1)
    {
        Gpio_toggle_pin(led.reg, led.pin_config.number);
        delay();
    }

    return 0;
}
