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
} gpio_mode_t;

typedef enum
{
    GPIO_OUT_TYPE_PUSH_PULL = 0u,
    GPIO_OUT_TYPE_OPEN_DRAIN = 1u,
} gpio_out_type_t;

typedef enum
{
    GPIO_SPEED_LOW = 0u,
    GPIO_SPEED_MED = 1u,
    GPIO_SPEED_FAST = 2u,
    GPIO_SPEED_HIGH = 3u,
} gpio_speed_t;

typedef enum
{
    GPIO_PULL_MODE_NONE = 0u,
    GPIO_PULL_MODE_UP = 1u,
    GPIO_PULL_MODE_DOWN = 2u,
} gpio_pull_mode_t;

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
} gpio_pin_t;

typedef enum
{
    GPIO_BUTTON_STATE_LOW = 0u,
    GPIO_BUTTON_STATE_HIGH = 1u,
} gpio_button_state_t;

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
} gpio_alternate_function_t;

typedef struct
{
    gpio_pin_t number;
    gpio_mode_t mode;
    gpio_speed_t speed;
    gpio_pull_mode_t pull_mode;
    gpio_out_type_t out_type;
    uint8_t alt_fun_mode;
} gpio_cfg_t;

typedef struct
{
    gpio_reg_t *const p_reg;
    const gpio_cfg_t cfg;
} gpio_handle_t;

typedef enum
{
    GPIO_PIN_STATUS_RESET = 0u,
    GPIO_PIN_STATUS_SET = 1u,
} gpio_pin_status_t;

void gpio_init(const gpio_handle_t *p_handle);
void gpio_deinit(gpio_reg_t *p_reg);

void gpio_enable_peripheral_clock(gpio_reg_t *p_reg, bool enable);

gpio_button_state_t gpio_read_pin(const gpio_handle_t *p_handle);
uint16_t gpio_read_port(gpio_reg_t *p_reg);
void gpio_write_to_pin(gpio_handle_t *p_handle, gpio_pin_status_t value);
void gpio_write_to_output_port(gpio_reg_t *p_reg, uint16_t value);
void gpio_toggle_pin(gpio_handle_t *p_handle);

void gpio_config_irq(irq_number_t irq_number, bool enable);
void gpio_config_irq_priority(irq_number_t irq_number, nvic_irq_priority_t priority);
void gpio_irq_handling(gpio_pin_t pin);

#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
