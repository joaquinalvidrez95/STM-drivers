#ifndef I2C_H
#define I2C_H

#include "stm32f446xx.h"
#include "stm32f446xx_nvic_driver.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "utils.h"

typedef enum
{
    I2C_BUS_1,
    I2C_BUS_2,
    I2C_BUS_3,
    I2C_BUS_TOTAL
} i2c_bus_t;

typedef enum
{
    I2C_SCL_SPEED_STANDARD_MODE = 100000u,
    I2C_SCL_SPEED_FAST_MODE_4K = 400000u,
    I2C_SCL_SPEED_FAST_MODE_2K = 200000u,
} i2c_scl_speed_t;

typedef enum
{
    I2C_ACK_CONTROL_DISABLED = 0u,
    I2C_ACK_CONTROL_ENABLED = 1u,
} i2c_ack_control_t;

typedef enum
{
    I2C_DUTY_2 = 0u,
    I2C_DUTY_16_9 = 1u,
} i2c_fm_duty_t;

typedef enum
{
    I2C_STATE_READY,
    I2C_STATE_BUSY_IN_RX,
    I2C_STATE_BUSY_IN_TX,
} i2c_state_t;

typedef enum
{
    I2C_INTERRUPT_EV_TX_DONE,
    I2C_INTERRUPT_EV_RX_DONE,
    I2C_INTERRUPT_EV_STOP,
    I2C_INTERRUPT_EV_SLAVE_TXE,
    I2C_INTERRUPT_EV_SLAVE_RXNE,
    I2C_INTERRUPT_ERR_BERR,
    I2C_INTERRUPT_ERR_ARLO,
    I2C_INTERRUPT_ERR_AF,
    I2C_INTERRUPT_ERR_OVR,
    I2C_INTERRUPT_ERR_TIMEOUT,
} i2c_interrupt_t;

typedef enum
{
    I2C_IRQ_EV,
    I2C_IRQ_ERR,
    I2C_IRQ_TOTAL,
} i2c_irq_t;

typedef enum
{
    I2C_REPEATED_START_DISABLED,
    I2C_REPEATED_START_ENABLED,
} i2c_repeated_start_t;

typedef void (*i2c_interrupt_callback_t)(i2c_interrupt_t interrupt);

typedef struct
{
    i2c_bus_t bus;
    i2c_scl_speed_t scl_speed;
    uint8_t device_address;
    i2c_ack_control_t ack_control;
    i2c_fm_duty_t fm_duty_cycle;
    i2c_interrupt_callback_t interrupt_cb;
} i2c_cfg_t;

/**
 * @brief 
 * 
 */
typedef struct
{
    uint8_t *p_buffer;
    size_t size;
    uint8_t slave_address;
    i2c_repeated_start_t repeated_start;
} i2c_msg_t;

void i2c_init(const i2c_cfg_t *p_cfg);
void i2c_deinit(i2c_bus_t bus);

void i2c_set_peripheral_enabled(i2c_bus_t bus, bool b_enabled);
void i2c_set_ack(i2c_bus_t bus, i2c_ack_control_t ack);

void i2c_transmit_as_master(i2c_bus_t bus, i2c_msg_t *p_msg, utils_mechanism_t mechanism);
void i2c_receive_as_master(i2c_bus_t bus, i2c_msg_t *p_msg, utils_mechanism_t mechanism);
void i2c_transmit_as_slave(i2c_bus_t bus, uint8_t data);
uint8_t i2c_receive_as_slave(i2c_bus_t bus);

bool i2c_is_interrupt_rx_tx_done(i2c_bus_t bus);
void i2c_set_interrupts_enabled(i2c_bus_t bus, bool b_enabled);
void i2c_set_irq_enabled(i2c_bus_t bus, i2c_irq_t irq, bool b_enabled);
void i2c_handle_ev_irq(i2c_bus_t bus);
void i2c_handle_err_irq(i2c_bus_t bus);

#endif /* I2C_H */
