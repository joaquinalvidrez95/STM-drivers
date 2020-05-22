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
#include "stm32f446xx_gpio_driver.h"

#define MAIN_001_LED_TOGGLE 0
#define MAIN_002_LED_BUTTON 1
#define MAIN_005_BUTTTON_INTERRUPT 2

#define MAIN MAIN_005_BUTTTON_INTERRUPT
void delay()
{
    for (uint32_t i = 0u; i < 500000u; i++)
    {
    }
}
#if MAIN == MAIN_001_LED_TOGGLE

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
        Gpio_toggle_pin(&led);
        delay();
    }

    return 0;
}

#elif MAIN == MAIN_002_LED_BUTTON
int main()
{
    Gpio_handle_t led;
    led.reg = GPIOA;
    led.pin_config.number = Gpio_pin_5;
    led.pin_config.mode = Gpio_mode_out;
    led.pin_config.speed = Gpio_speed_fast;
    led.pin_config.out_type = Gpio_out_type_push_pull;
    led.pin_config.pull_mode = Gpio_pull_mode_none;

    Gpio_peripheral_clock_control(led.reg, true);
    Gpio_init(&led);

    Gpio_handle_t button;
    button.reg = GPIOC;
    button.pin_config.number = Gpio_pin_13;
    button.pin_config.mode = Gpio_mode_in;
    button.pin_config.speed = Gpio_speed_fast;
    button.pin_config.pull_mode = Gpio_pull_mode_none;

    Gpio_peripheral_clock_control(button.reg, true);
    Gpio_init(&button);

    while (1)
    {
        if (Gpio_button_state_low == Gpio_read_from_input_pin(&button))
        {
            delay();
            Gpio_toggle_pin(&led);
        }
    }

    return 0;
}
#elif MAIN == MAIN_005_BUTTTON_INTERRUPT
static Gpio_handle_t button = {0u};
static Gpio_handle_t led = {0u};
int main()
{
    led.reg = GPIOA;
    led.pin_config.number = Gpio_pin_5;
    led.pin_config.mode = Gpio_mode_out;
    led.pin_config.speed = Gpio_speed_fast;
    led.pin_config.out_type = Gpio_out_type_push_pull;
    led.pin_config.pull_mode = Gpio_pull_mode_none;

    Gpio_peripheral_clock_control(led.reg, true);
    Gpio_init(&led);

    button.reg = GPIOC;
    button.pin_config.number = Gpio_pin_13;
    button.pin_config.mode = Gpio_mode_interrupt_ft;
    button.pin_config.speed = Gpio_speed_fast;
    button.pin_config.pull_mode = Gpio_pull_mode_none;

    Gpio_peripheral_clock_control(button.reg, true);
    Gpio_init(&button);

    /* Configures IRQ */
    Gpio_config_irq_priority(irq_number_exti15_10, nvic_irq_priority_15);
    Gpio_config_irq(irq_number_exti15_10, true);

    while (1)
        ;

    return 0;
}

void EXTI15_10_IRQHandler()
{
    delay();
    Gpio_irq_handling(button.pin_config.number);
    Gpio_toggle_pin(&led);
}
#endif
