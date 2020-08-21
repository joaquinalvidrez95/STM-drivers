#ifndef I2C_H
#define I2C_H

#include "stm32f446xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef enum
{
    I2C_SCL_SPEED_STANDARD_MODE = 100000u,
    I2C_SCL_SPEED_FAST_MODE_4K = 400000u,
    I2C_SCL_SPEED_FAST_MODE_2K = 200000u,
} i2c_scl_speed_t;

typedef enum
{
    I2C_ACK_CONTROL_DISABLE = 0u,
    I2C_ACK_CONTROL_ENABLE = 1u,
} i2c_ack_control_t;

typedef enum
{
    I2C_DUTY_2 = 0u,
    I2C_DUTY_16_9 = 1u,
} i2c_fm_duty_t;

typedef struct
{
    i2c_scl_speed_t scl_speed;
    uint8_t device_address;
    i2c_ack_control_t ack_control;
    i2c_fm_duty_t fm_duty_cycle;
} i2c_cfg_t;

typedef struct
{
    i2c_reg_t *p_reg;
    i2c_cfg_t cfg;
} i2c_handle_t;

/**
 * @brief 
 * 
 */
typedef struct
{
    const uint8_t *buffer;
    size_t size;
    uint8_t slave_address;
} i2c_message_t;

void i2c_enable_peripheral_clock(i2c_reg_t *p_reg, bool enable);
void i2c_enable_peripheral(i2c_reg_t *p_reg, bool enable);

void i2c_init(i2c_handle_t *p_handle);
void i2c_deinit(i2c_reg_t *p_reg);

void I2c_enable_ssi(i2c_reg_t *p_reg, bool enable);
void I2c_enable_ssoe(i2c_reg_t *p_reg, bool enable);

void i2c_send_as_master(i2c_handle_t *p_handle, const i2c_message_t *p_message);
void I2c_receive(i2c_handle_t *p_handle);

void I2c_send_interrupt(i2c_handle_t *p_handle);
void I2c_receive_interrupt(i2c_handle_t *p_handle);

void I2c_config_irq(Irq_number_t irq_number, bool enable);
void I2c_config_irq_priority(Irq_number_t irq_number, Nvic_irq_priority_t priority);
void I2c_handle_irq(i2c_handle_t *p_handle);

// void I2c_on_app_event(i2c_handle_t *p_handle, I2c_event_e event);
#endif /* I2C_H */
