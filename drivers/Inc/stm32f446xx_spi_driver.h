/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: May 21, 2020
 *      Author: joaquin
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f446xx.h"

typedef enum
{
    SPI_DEVICE_MODE_SLAVE = 0u,
    SPI_DEVICE_MODE_MASTER = 1u,
} Spi_device_mode_e;

typedef enum
{
    SPI_BUS_CONFIG_FULL_DUPLEX = 1,
    SPI_BUS_CONFIG_HALF_DUPLEX,
    SPI_BUS_CONFIG_SIMPLE_RX_ONLY,
} Spi_bus_config_e;

typedef enum
{
    SPI_CLOCK_SPEED_DIV_2 = 0u,
    SPI_CLOCK_SPEED_DIV_4 = 1u,
    SPI_CLOCK_SPEED_DIV_8 = 2u,
    SPI_CLOCK_SPEED_DIV_16 = 3u,
    SPI_CLOCK_SPEED_DIV_32 = 4u,
    SPI_CLOCK_SPEED_DIV_64 = 5u,
    SPI_CLOCK_SPEED_DIV_128 = 6u,
    SPI_CLOCK_SPEED_DIV_256 = 7u,
} Spi_clock_speed_e;

typedef enum
{
    SPI_DFF_8_BITS = 0u,
    SPI_DFF_16_BITS = 1u,
} Spi_dff_e;

typedef enum
{
    SPI_CPOL_LOW = 0u,
    SPI_CPOL_HIGH = 1u,
} Spi_cpol_e;

typedef enum
{
    SPI_CPHA_LOW = 0u,
    SPI_CPHA_HIGH = 1u,
} Spi_cpha_e;

/* Software slave management */
typedef enum
{
    SPI_SSM_DISABLED = 0u,
    SPI_SSM_ENABLED = 1u,
} Spi_ssm_e;

typedef enum
{
    SPI_STATE_READY,
    SPI_STATE_BUSY,
} Spi_state_e;

typedef enum
{
    SPI_EVENT_TX_DONE,
    SPI_EVENT_RX_DONE,
    SPI_EVENT_OVR_ERR,
    SPI_EVENT_CRC_ERR,
} Spi_event_e;

typedef struct
{
    Spi_device_mode_e device_mode;
    Spi_bus_config_e bus_config;
    Spi_clock_speed_e sclk_speed;
    Spi_dff_e DFF;
    Spi_cpol_e CPOL;
    Spi_cpha_e CPHA;
    Spi_ssm_e SSM;
} Spi_config_t;

typedef struct
{
    /* TODO: Check if it can be const */
    uint8_t *data;     /* !< To store the app. Tx buffer address > */
    size_t size;       /* !< To store Tx len > */
    Spi_state_e state; /* !< To store Tx state > */
} Spi_buffer_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct
{
    Spi_reg_t *reg; /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
    Spi_config_t config;
    Spi_buffer_t tx;
    Spi_buffer_t rx;
} Spi_handle_t;

void Spi_init(Spi_handle_t *handle);
void Spi_deinit(Spi_reg_t *reg);

void Spi_peripheral_clock_control(Spi_reg_t *reg, bool enable);
void Spi_enable_peripheral(Spi_reg_t *reg, bool enable);
void Spi_enable_ssi(Spi_reg_t *reg, bool enable);
void Spi_enable_ssoe(Spi_reg_t *reg, bool enable);

void Spi_send(Spi_handle_t *handle);
void Spi_receive(Spi_handle_t *handle);

void Spi_send_interrupt(Spi_handle_t *handle);
void Spi_receive_interrupt(Spi_handle_t *handle);

void Spi_config_irq(Irq_number_t irq_number, bool enable);
void Spi_config_irq_priority(Irq_number_t irq_number, Nvic_irq_priority_t priority);
void Spi_handle_irq(Spi_handle_t *handle);

void Spi_clear_ovr_flag(Spi_reg_t *reg);
void Spi_close_transmission(Spi_handle_t *handle);
void Spi_close_reception(Spi_handle_t *handle);

void Spi_on_app_event(Spi_handle_t *handle, Spi_event_e event);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
