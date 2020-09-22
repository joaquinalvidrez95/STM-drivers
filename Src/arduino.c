/*
 * arduino.c
 *
 *  Created on: Jun 15, 2020
 *      Author: joaquin
 */

#include <string.h>
#include "arduino.h"
#include "utils.h"
#include "stm32f446xx_i2c_driver.h"

typedef enum
{
    ARDUINO_COMMAND_LED_CTRL = 0x50u,
    ARDUINO_COMMAND_SENSOR_READ = 0x51u,
    ARDUINO_COMMAND_LED_READ = 0x52u,
    ARDUINO_COMMAND_PRINT = 0x53u,
    ARDUINO_COMMAND_ID_READ = 0x54,
} arduino_spi_command_t;

static inline bool is_response_correct(uint8_t ack_byte);
static bool verify_response(spi_handle_t *spi);
static bool send_command(spi_handle_t *spi, arduino_spi_command_t command, uint8_t arguments[], size_t arguments_length);

arduino_digital_status_t arduino_read_digital(spi_handle_t *spi, arduino_digital_pin_t pin)
{
    arduino_digital_status_t read = 0u;
    if (send_command(spi, ARDUINO_COMMAND_LED_READ, (uint8_t *)&pin, sizeof(uint8_t)))
    {
        Spi_read_dummy(spi);
        utils_delay();
        Spi_send_dummy(spi);

        spi->rx.data = (uint8_t *)&read;
        spi->rx.size = sizeof(uint8_t);
        Spi_receive(spi);
    }

    return read;
}

void arduino_write_led(spi_handle_t *spi, arduino_digital_status_t status, arduino_digital_pin_t pin)
{
    uint8_t arguments[] = {(uint8_t)pin, (uint8_t)status};
    send_command(spi, ARDUINO_COMMAND_LED_CTRL, arguments, sizeof(arguments));
}

uint8_t arduino_read_analog(spi_handle_t *spi, arduino_analog_pin_t pin)
{
    uint8_t analog_read = 0u;

    if (send_command(spi, ARDUINO_COMMAND_SENSOR_READ, (uint8_t *)&pin, sizeof(uint8_t)))
    {
        Spi_read_dummy(spi);
        utils_delay();
        Spi_send_dummy(spi);

        spi->rx.data = &analog_read;
        spi->rx.size = sizeof(analog_read);
        Spi_receive(spi);
    }

    return analog_read;
}

void arduino_print(spi_handle_t *spi, const char message[])
{
    char new_message[UINT8_MAX + 1u] = {'\0'};
    size_t s = strlen(message);
    strcpy(new_message, (const char *)&s);
    strcat(new_message, message);
    send_command(spi, ARDUINO_COMMAND_PRINT, (uint8_t *)new_message, strlen(new_message));
}

void arduino_read_id(spi_handle_t *spi, char id[], size_t length)
{
    if (send_command(spi, ARDUINO_COMMAND_ID_READ, NULL, 0u))
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

uint8_t arduino_i2c_get_length(i2c_bus_t bus, utils_mechanism_t mechanism)
{
    uint8_t length = 0u;

    i2c_transmit_as_master(bus,
                           &(i2c_msg_t){
                               .slave_address = ARDUINO_I2C_ADDRESS,
                               .size = sizeof(uint8_t),
                               .p_buffer = &(uint8_t){ARDUINO_I2C_COMMAND_READ_LENGTH},
                               .repeated_start = I2C_REPEATED_START_ENABLED,
                           },
                           mechanism);

    while ((UTILS_MECHANISM_INTERRUPT == mechanism) && (!i2c_is_interrupt_rx_tx_done(bus)))
    {
    }

    i2c_receive_as_master(bus,
                          &(i2c_msg_t){
                              .slave_address = ARDUINO_I2C_ADDRESS,
                              .size = sizeof(length),
                              .p_buffer = &length,
                              .repeated_start = I2C_REPEATED_START_ENABLED,
                          },
                          mechanism);

    while ((UTILS_MECHANISM_INTERRUPT == mechanism) && (!i2c_is_interrupt_rx_tx_done(bus)))
    {
    }

    return length;
}

static inline bool is_response_correct(uint8_t ack_byte)
{
    return ack_byte == 0xF5u;
}

static bool verify_response(spi_handle_t *spi)
{

    uint8_t ack_byte = 0u;
    spi->rx.data = &ack_byte;
    spi->rx.size = sizeof(ack_byte);
    Spi_receive(spi);

    return is_response_correct(ack_byte);
}

static bool send_command(spi_handle_t *spi, arduino_spi_command_t command, uint8_t arguments[], size_t arguments_length)
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
