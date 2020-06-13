/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: May 21, 2020
 *      Author: joaquin
 */

#include "stm32f446xx_spi_driver.h"

void Spi_init(Spi_handle_t *handle)
{
    Spi_peripheral_clock_control(handle->reg, true);

    handle->reg->CR1.MSTR = (uint8_t)handle->config.device_mode;

    switch (handle->config.bus_config)
    {
    case SPI_BUS_CONFIG_FULL_DUPLEX:
        handle->reg->CR1.BIDIMODE = 0u;
        break;
    case SPI_BUS_CONFIG_HALF_DUPLEX:
        handle->reg->CR1.BIDIMODE = 1u;
        break;
    case SPI_BUS_CONFIG_SIMPLE_RX_ONLY:
        handle->reg->CR1.BIDIMODE = 0u;
        handle->reg->CR1.RXONLY = 1u;
        break;

    default:
        break;
    }

    handle->reg->CR1.BR = (uint8_t)handle->config.sclk_speed;
    handle->reg->CR1.DFF = (uint8_t)handle->config.DFF;
    handle->reg->CR1.CPOL = (uint8_t)handle->config.CPOL;
    handle->reg->CR1.CPHA = (uint8_t)handle->config.CPHA;
    handle->reg->CR1.SSM = (uint8_t)handle->config.SSM;
}
void Spi_deinit(Spi_reg_t *reg)
{
    if (reg == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (reg == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (reg == SPI3)
    {
        SPI3_REG_RESET();
    }
    else if (reg == SPI4)
    {
        SPI4_REG_RESET();
    }
}

void Spi_peripheral_clock_control(Spi_reg_t *reg, bool enable)
{

    if (enable == true)
    {
        if (reg == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (reg == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (reg == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (reg == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if (reg == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (reg == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (reg == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (reg == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

void Spi_enable_peripheral(Spi_reg_t *reg, bool enable)
{
    reg->CR1.SPE = (uint8_t)enable;
}

void Spi_enable_ssi(Spi_reg_t *reg, bool enable)
{
    reg->CR1.SSI = (uint8_t)enable;
}

void Spi_enable_ssoe(Spi_reg_t *reg, bool enable)
{
    reg->CR2.SSOE = (uint8_t)enable;
}

/**
 * @brief Blocks
 * 
 * @param handle 
 */
void Spi_send(Spi_handle_t *handle)
{

    for (size_t i = handle->tx.size; i > 0u;)
    {
        while (!handle->reg->SR.TXE)
        {
        }

        if (handle->reg->CR1.DFF == SPI_DFF_8_BITS)
        {
            handle->reg->DR = *handle->tx.data;
            i--;
            handle->tx.data++;
        }
        else
        {
            handle->reg->DR = *((uint16_t *)handle->tx.data);
            i -= 2u;
            handle->tx.data += 2u;
        }
    }
}
void Spi_receive(Spi_handle_t *handle)
{
    size_t length = handle->rx.size;

    while (length > 0u)
    {
        while (!handle->reg->SR.RXNE)
        {
        }

        if (handle->reg->CR1.DFF == SPI_DFF_8_BITS)
        {
            *handle->rx.data = handle->reg->DR;
            length--;
            handle->rx.data++;
        }
        else
        {
            *((uint16_t *)handle->rx.data) = handle->reg->DR;
            length -= 2u;
            handle->rx.data += 2u;
        }
    }
}

void Spi_config_irq(Irq_number_t irq_number, bool enable)
{
}
void Spi_config_irq_priority(Irq_number_t irq_number, Nvic_irq_priority_t priority)
{
}
void Spi_irq_handling(Spi_handle_t pin)
{
}
