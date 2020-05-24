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
    GPIO_MODE_IN = 0u,
    GPIO_MODE_OUT = 1u,
    GPIO_MODE_ALT_FN = 2u,
    GPIO_MODE_ANALOG = 3u,

    /* Interrupts */
    GPIO_MODE_INTERRUPT_FT = 4u,
    GPIO_MODE_INTERRUPT_RT = 5u,
    GPIO_MODE_INTERRUPT_RFT = 6u,
} Gpio_mode_e;

typedef enum
{
    GPIO_OUT_TYPE_PUSH_PULL = 0u,
    GPIO_OUT_TYPE_OPEN_DRAIN = 1u,
} Gpio_out_type_e;

typedef enum
{
    GPIO_SPEED_LOW = 0u,
    GPIO_SPEED_MED = 1u,
    GPIO_SPEED_FAST = 2u,
    GPIO_SPEED_HIGH = 3u,
} Gpio_speed_e;

typedef enum
{
    GPIO_PULL_MODE_NONE = 0u,
    GPIO_PULL_MODE_UP = 1u,
    GPIO_PULL_MODE_DOWN = 2u,
} Gpio_pull_mode_e;

typedef enum
{
    GPIO_PIN_0 = 0u,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15,
    GPIO_PIN_16,
} Gpio_pin_e;

typedef enum
{
    GPIO_BUTTON_STATE_LOW = 0u,
    GPIO_BUTTON_STATE_HIGH = 1u,    
} Gpio_button_state_e;

typedef enum
{
    GPIO_ALTERNATE_FUNCTION_0 = 0u,
    GPIO_ALTERNATE_FUNCTION_1,
    GPIO_ALTERNATE_FUNCTION_2,
    GPIO_ALTERNATE_FUNCTION_3,
    GPIO_ALTERNATE_FUNCTION_4,
    GPIO_ALTERNATE_FUNCTION_5,
    GPIO_ALTERNATE_FUNCTION_6,
    GPIO_ALTERNATE_FUNCTION_7,
    GPIO_ALTERNATE_FUNCTION_8,
    GPIO_ALTERNATE_FUNCTION_9,
    GPIO_ALTERNATE_FUNCTION_10,
    GPIO_ALTERNATE_FUNCTION_11,
    GPIO_ALTERNATE_FUNCTION_12,
    GPIO_ALTERNATE_FUNCTION_13,
    GPIO_ALTERNATE_FUNCTION_14,
    GPIO_ALTERNATE_FUNCTION_15,
} Gpio_alternate_function_e;

typedef struct
{
    Gpio_pin_e number;
    Gpio_mode_e mode;
    Gpio_speed_e speed;
    Gpio_pull_mode_e pull_mode;
    Gpio_out_type_e out_type;
    uint8_t alt_fun_mode;
} Gpio_pin_config_t;

typedef struct
{
    Gpio_reg_t *reg;
    Gpio_pin_config_t pin_config;
} Gpio_handle_t;

void Gpio_init(Gpio_handle_t *handle);
void Gpio_deinit(Gpio_reg_t *reg);

void Gpio_peripheral_clock_control(Gpio_reg_t *reg, bool enable);

Gpio_button_state_e Gpio_read_from_input_pin(Gpio_handle_t *handle);
uint16_t Gpio_read_from_input_port(Gpio_reg_t *reg);
void Gpio_write_to_pin(Gpio_handle_t *handle, Gpio_pin_status_t value);
void Gpio_write_to_output_port(Gpio_reg_t *reg, uint16_t value);
void Gpio_toggle_pin(Gpio_handle_t *handle);

void Gpio_config_irq(Irq_number_t irq_number, bool enable);
void Gpio_config_irq_priority(Irq_number_t irq_number, Nvic_irq_priority_t priority);
void Gpio_irq_handling(Gpio_pin_e pin);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
