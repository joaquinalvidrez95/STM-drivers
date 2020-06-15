/*
 * arduino.c
 *
 *  Created on: Jun 15, 2020
 *      Author: joaquin
 */

#include <string.h>
#include "arduino.h"
#include "utils.h"

static bool is_response_correct(uint8_t ack_byte)
{
    return ack_byte == 0xF5u;
}

static bool verify_response(Spi_handle_t *spi)
{

    uint8_t ack_byte = 0u;
    spi->rx.data = &ack_byte;
    spi->rx.size = sizeof(ack_byte);
    Spi_receive(spi);

    return is_response_correct(ack_byte);
}

static bool send_command(Spi_handle_t *spi, Command_e command, uint8_t arguments[], size_t arguments_length)
{
    /* Command 1 */
    spi->tx.data = (uint8_t *)&command;
    spi->tx.size = sizeof(uint8_t);
    Spi_send(spi);

    Spi_read_dummy(spi);
    Spi_send_dummy(spi);

    const bool result = verify_response(spi);

    if (result && (arguments != NULL))
    {
        spi->tx.data = &arguments[0];
        spi->tx.size = arguments_length;
        Spi_send(spi);
    }

    return result;
}

Led_status_e Arduino_read_digital(Spi_handle_t *spi, Arduino_digital_pin_e pin)
{
    Led_status_e read = 0u;
    if (send_command(spi, COMMAND_LED_READ, (uint8_t *)&pin, sizeof(uint8_t)))
    {
        Spi_read_dummy(spi);
        Utils_delay();
        Spi_send_dummy(spi);

        spi->rx.data = (uint8_t *)&read;
        spi->rx.size = sizeof(uint8_t);
        Spi_receive(spi);
    }

    return read;
}

void Arduino_write_led(Spi_handle_t *spi, Led_status_e status, Arduino_digital_pin_e pin)
{
    uint8_t arguments[] = {(uint8_t)pin, (uint8_t)status};
    send_command(spi, COMMAND_LED_CTRL, arguments, sizeof(arguments));
}

uint8_t Arduino_read_analog(Spi_handle_t *spi, Arduino_analog_pin_e pin)
{
    uint8_t analog_read = 0u;

    if (send_command(spi, COMMAND_SENSOR_READ, (uint8_t *)&pin, sizeof(uint8_t)))
    {
        Spi_read_dummy(spi);
        Utils_delay();
        Spi_send_dummy(spi);

        spi->rx.data = &analog_read;
        spi->rx.size = sizeof(analog_read);
        Spi_receive(spi);
    }

    return analog_read;
}

void Arduino_print(Spi_handle_t *spi, const char message[])
{
    char new_message[UINT8_MAX + 1u] = {'\0'};
    size_t s = strlen(message);
    strcpy(new_message, (const char *)&s);
    strcat(new_message, message);
    send_command(spi, COMMAND_PRINT, (uint8_t *)new_message, strlen(new_message));
}

void Arduino_read_id(Spi_handle_t *spi, char id[], size_t length)
{
    if (send_command(spi, COMMAND_ID_READ, NULL, 0u))
    {
        spi->rx.size = sizeof(uint8_t);
        for (size_t i = 0u; i < length; i++)
        {
            Spi_send_dummy(spi);
            spi->rx.data = (uint8_t *)&id[i];
            Spi_receive(spi);
        }
    }
}