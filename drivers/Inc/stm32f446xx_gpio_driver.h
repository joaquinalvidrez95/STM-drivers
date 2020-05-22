/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: May 19, 2020
 *      Author: joaquin
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"
#include <stdbool.h>

typedef enum
{
    Gpio_mode_in = 0u,
    Gpio_mode_out = 1u,
    Gpio_mode_alt_fn = 2u,
    Gpio_mode_analog = 3u,

    /* Interrupts */
    Gpio_mode_interrupt_ft = 4u,
    Gpio_mode_interrupt_rt = 5u,
    Gpio_mode_interrupt_rft = 6u,
} Gpio_mode_t;

typedef enum
{
    Gpio_out_type_push_pull = 0u,
    Gpio_out_type_open_drain = 1u,
} Gpio_out_type_t;

typedef enum
{
    Gpio_speed_low = 0u,
    Gpio_speed_med = 1u,
    Gpio_speed_fast = 2u,
    Gpio_speed_high = 3u,
} Gpio_speed_t;

typedef enum
{
    Gpio_pull_mode_none = 0u,
    Gpio_pull_mode_up = 1u,
    Gpio_pull_mode_down = 2u,
} Gpio_pull_mode_t;

typedef enum
{
    Gpio_pin_0 = 0u,
    Gpio_pin_1,
    Gpio_pin_2,
    Gpio_pin_3,
    Gpio_pin_4,
    Gpio_pin_5,
    Gpio_pin_6,
    Gpio_pin_7,
    Gpio_pin_8,
    Gpio_pin_9,
    Gpio_pin_10,
    Gpio_pin_11,
    Gpio_pin_12,
    Gpio_pin_13,
    Gpio_pin_14,
    Gpio_pin_15,
    Gpio_pin_16,
} Gpio_pin_t;

typedef struct
{
    Gpio_pin_t number;
    Gpio_mode_t mode;
    Gpio_speed_t speed;
    Gpio_pull_mode_t pull_mode;
    Gpio_out_type_t out_type;
    uint8_t alt_fun_mode;
} Gpio_pin_config_t;

typedef struct
{
    Gpio_reg_t *reg;
    Gpio_pin_config_t pin_config;
} Gpio_handle_t;

typedef enum
{
    Gpio_button_state_low = 0u,
    Gpio_button_state_high = 1u,
} Gpio_button_state_t;

void Gpio_init(Gpio_handle_t *handle);
void Gpio_deinit(Gpio_reg_t *reg);

void Gpio_peripheral_clock_control(Gpio_reg_t *reg, bool enable);

Gpio_button_state_t Gpio_read_from_input_pin(Gpio_handle_t *handle);
uint16_t Gpio_read_from_input_port(Gpio_reg_t *reg);
void Gpio_write_to_pin(Gpio_handle_t *handle, Gpio_pin_status_t value);
void Gpio_write_to_output_port(Gpio_reg_t *reg, uint16_t value);
void Gpio_toggle_pin(Gpio_handle_t *handle);

void Gpio_config_irq(Irq_number_t irq_number, bool enable);
void Gpio_config_irq_priority(Irq_number_t irq_number, Nvic_irq_priority_t priority);
void Gpio_irq_handling(Gpio_pin_t pin);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
