/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: May 21, 2020
 *      Author: joaquin
 */

#include "stm32f446xx_spi_driver.h"

static void Handle_txe_interrupt(spi_handle_t *handle)
{
    if (handle->p_reg->CR1.DFF == SPI_DFF_8_BITS)
    {
        handle->p_reg->DR = *handle->tx.data;
        handle->tx.size--;
        handle->tx.data++;
    }
    else
    {
        handle->p_reg->DR = *((uint16_t *)handle->tx.data);
        handle->tx.size -= 2u;
        handle->tx.data += 2u;
    }
    if (!handle->tx.size)
    {
        /* Closes SPI Transmission */
        Spi_close_transmission(handle);
        Spi_on_app_event(handle, SPI_EVENT_TX_DONE);
    }
}

static void Handle_rxne_interrupt(spi_handle_t *handle)
{
    if (handle->p_reg->CR1.DFF == SPI_DFF_8_BITS)
    {
        *handle->rx.data = (uint8_t)handle->p_reg->DR;
        handle->rx.size--;
        handle->rx.data++;
    }
    else
    {
        *((uint16_t *)handle->rx.data) = (uint16_t)handle->p_reg->DR;
        handle->rx.size -= 2u;
        handle->rx.data += 2u;
    }

    if (!handle->rx.size)
    {
        /* Closes SPI reception */
        Spi_close_reception(handle);
        Spi_on_app_event(handle, SPI_EVENT_RX_DONE);
    }
}
static void Handle_ovr_err_interrupt(spi_handle_t *handle)
{
    /* Clears the OVR flag */
    if (handle->tx.state != SPI_STATE_BUSY)
    {
        Spi_clear_ovr_flag(handle->p_reg);
    }
    Spi_on_app_event(handle, SPI_EVENT_OVR_ERR);
}

void Spi_init(spi_handle_t *handle)
{
    Spi_peripheral_clock_control(handle->p_reg, true);

    handle->p_reg->CR1.MSTR = (unsigned int)handle->cfg.device_mode;

    switch (handle->cfg.bus_config)
    {
    case SPI_BUS_CONFIG_FULL_DUPLEX:
        handle->p_reg->CR1.BIDIMODE = 0u;
        break;
    case SPI_BUS_CONFIG_HALF_DUPLEX:
        handle->p_reg->CR1.BIDIMODE = 1u;
        break;
    case SPI_BUS_CONFIG_SIMPLE_RX_ONLY:
        handle->p_reg->CR1.BIDIMODE = 0u;
        handle->p_reg->CR1.RXONLY = 1u;
        break;

    default:
        break;
    }

    handle->p_reg->CR1.BR = (uint8_t)handle->cfg.sclk_speed;
    handle->p_reg->CR1.DFF = (uint8_t)handle->cfg.DFF;
    handle->p_reg->CR1.CPOL = (uint8_t)handle->cfg.CPOL;
    handle->p_reg->CR1.CPHA = (uint8_t)handle->cfg.CPHA;
    handle->p_reg->CR1.SSM = (uint8_t)handle->cfg.SSM;
}
void Spi_deinit(Spi_reg_t *p_reg)
{
    if (p_reg == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if (p_reg == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if (p_reg == SPI3)
    {
        SPI3_REG_RESET();
    }
    else if (p_reg == SPI4)
    {
        SPI4_REG_RESET();
    }
}

void Spi_peripheral_clock_control(Spi_reg_t *p_reg, bool enable)
{
    if (enable == true)
    {
        if (p_reg == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (p_reg == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (p_reg == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (p_reg == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if (p_reg == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (p_reg == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (p_reg == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (p_reg == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

void Spi_enable_peripheral(Spi_reg_t *p_reg, bool enable)
{
    p_reg->CR1.SPE = (uint8_t)enable;
}

void Spi_enable_ssi(Spi_reg_t *p_reg, bool enable)
{
    p_reg->CR1.SSI = (uint8_t)enable;
}

void Spi_enable_ssoe(Spi_reg_t *p_reg, bool enable)
{
    p_reg->CR2.SSOE = (uint8_t)enable;
}

/**
 * @brief Blocks
 * 
 * @param handle 
 */
void Spi_send(spi_handle_t *handle)
{
    for (size_t i = handle->tx.size; i > 0u;)
    {
        while (!handle->p_reg->SR.TXE)
        {
        }

        if (handle->p_reg->CR1.DFF == SPI_DFF_8_BITS)
        {
            handle->p_reg->DR = *handle->tx.data;
            i--;
            handle->tx.data++;
        }
        else
        {
            handle->p_reg->DR = *((uint16_t *)handle->tx.data);
            i -= 2u;
            handle->tx.data += 2u;
        }
    }
}
void Spi_receive(spi_handle_t *handle)
{
    size_t length = handle->rx.size;

    while (length > 0u)
    {
        while (!handle->p_reg->SR.RXNE)
        {
        }

        if (handle->p_reg->CR1.DFF == SPI_DFF_8_BITS)
        {
            *handle->rx.data = handle->p_reg->DR;
            length--;
            handle->rx.data++;
        }
        else
        {
            *((uint16_t *)handle->rx.data) = handle->p_reg->DR;
            length -= 2u;
            handle->rx.data += 2u;
        }
    }
}
/* TODO: Reuse code */
void Spi_config_irq(irq_num_t irq_number, bool enable)
{
    if (enable == true)
    {
        if (irq_number <= 31u)
        {
            *NVIC_ISER0 |= 1u << irq_number;
        }
        else if ((irq_number > 31u) && (irq_number < 64u))
        {
            *NVIC_ISER1 |= 1u << (irq_number % 32u);
        }
        else if ((irq_number >= 64u) && (irq_number < 96u))
        {
            *NVIC_ISER3 |= 1u << (irq_number % 64u);
        }
    }
    else
    {
        if (irq_number <= 31u)
        {
            *NVIC_ICER0 |= 1u << irq_number;
        }
        else if ((irq_number > 31u) && (irq_number < 64u))
        {
            *NVIC_ICER1 |= 1u << (irq_number % 32u);
        }
        else if ((irq_number >= 64u) && (irq_number < 96u))
        {
            *NVIC_ICER3 |= 1u << (irq_number % 64u);
        }
    }
}

void Spi_send_interrupt(spi_handle_t *handle)
{
    if (handle->tx.state != SPI_STATE_BUSY)
    {
        handle->tx.state = SPI_STATE_BUSY;
        handle->p_reg->CR2.TXEIE = 1u;
    }
}
void Spi_receive_interrupt(spi_handle_t *handle)
{
    if (handle->rx.state != SPI_STATE_BUSY)
    {
        handle->tx.state = SPI_STATE_BUSY;
        handle->p_reg->CR2.RXNEIE = 1u;
    }
}

void Spi_config_irq_priority(irq_num_t irq_number, nvic_irq_prio_t priority)
{
    const uint8_t index = irq_number / 4u;
    const uint8_t section = irq_number % 4u;
    const uint8_t shift_amount = (8u * section) + (8u - NO_PR_BITS_IMPLEMENTED);
    NVIC_PR_BASE_ADDR[index] |= (uint32_t)priority << shift_amount;
}

void Spi_handle_irq(spi_handle_t *handle)
{
    if (handle->p_reg->SR.TXE && handle->p_reg->CR2.TXEIE)
    {
        /* handles TXE */
        Handle_txe_interrupt(handle);
    }

    if (handle->p_reg->SR.RXNE && handle->p_reg->CR2.RXNEIE)
    {
        /* handles RXNE */
        Handle_rxne_interrupt(handle);
    }

    if (handle->p_reg->SR.OVR && handle->p_reg->CR2.ERRIE)
    {
        /* handles OVR */
        Handle_ovr_err_interrupt(handle);
    }
}

void Spi_close_transmission(spi_handle_t *handle)
{
    /* Closes SPI Transmission */
    handle->p_reg->CR2.TXEIE = 0u;
    handle->tx.data = NULL;
    handle->tx.size = 0u;
    handle->tx.state = SPI_STATE_READY;
}

void Spi_close_reception(spi_handle_t *handle)
{
    /* Closes SPI reception */
    handle->p_reg->CR2.RXNEIE = 0u;
    handle->rx.data = NULL;
    handle->rx.size = 0u;
    handle->rx.state = SPI_STATE_READY;
}

void Spi_clear_ovr_flag(Spi_reg_t *p_reg)
{
    uint8_t tmp = p_reg->DR;
    (void)p_reg->SR;
    (void)tmp;
}

__attribute__((weak)) void Spi_on_app_event(spi_handle_t *handle, Spi_event_e event)
{
    /* weak implementation */
}

void Spi_read_dummy(spi_handle_t *spi)
{
    /* Does dummy read to clear off the RXNE */
    uint8_t dummy_read = 0u;
    spi->rx.data = &dummy_read;
    spi->rx.size = sizeof(dummy_read);
    Spi_receive(spi);
}

void Spi_send_dummy(spi_handle_t *spi)
{
    /* Sends some dummy bits to fetch the response from the slave */
    uint8_t dummy_byte = 0xFFu;
    spi->tx.data = &dummy_byte;
    spi->tx.size = sizeof(dummy_byte);
    Spi_send(spi);
}