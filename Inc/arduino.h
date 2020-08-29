/*
 * arduino.h
 *
 *  Created on: Jun 15, 2020
 *      Author: joaquin
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_

#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include <stdbool.h>

typedef enum
{
    ARDUINO_DIGITAL_STATUS_OFF = 0u,
    ARDUINO_DIGITAL_STATUS_ON = 1u
} arduino_digital_status_t;

typedef enum
{
    ARDUINO_ANALOG_PIN_0 = 0u,
    ARDUINO_ANALOG_PIN_1 = 1u,
    ARDUINO_ANALOG_PIN_2 = 2u,
    ARDUINO_ANALOG_PIN_3 = 3u,
    ARDUINO_ANALOG_PIN_4 = 4u,
} arduino_analog_pin_t;

typedef enum
{
    ARDUINO_I2C_COMMAND_READ_LENGTH = 0x51u,
    ARDUINO_I2C_COMMAND_READ_MSG = 0x52u,
} arduino_i2c_command_t;

typedef enum
{
    ARDUINO_DIGITAL_PIN_0 = 0u,
    ARDUINO_DIGITAL_PIN_1,
    ARDUINO_DIGITAL_PIN_2,
    ARDUINO_DIGITAL_PIN_3,
    ARDUINO_DIGITAL_PIN_4,
    ARDUINO_DIGITAL_PIN_5,
    ARDUINO_DIGITAL_PIN_6,
    ARDUINO_DIGITAL_PIN_7,
    ARDUINO_DIGITAL_PIN_8,
    ARDUINO_DIGITAL_PIN_9,
    ARDUINO_DIGITAL_PIN_10,
    ARDUINO_DIGITAL_PIN_11,
    ARDUINO_DIGITAL_PIN_12,
    ARDUINO_DIGITAL_PIN_13,
} arduino_digital_pin_t;

#define ARDUINO_ID_SIZE 10u
#define ARDUINO_I2C_ADDRESS (0x68u)

arduino_digital_status_t arduino_read_digital(spi_handle_t *spi, arduino_digital_pin_t pin);

void arduino_write_led(spi_handle_t *spi, arduino_digital_status_t status, arduino_digital_pin_t pin);

uint8_t arduino_read_analog(spi_handle_t *spi, arduino_analog_pin_t pin);

void arduino_print(spi_handle_t *spi, const char message[]);

void arduino_read_id(spi_handle_t *spi, char id[], size_t length);

uint8_t arduino_i2c_get_length(i2c_bus_t bus);

#endif /* ARDUINO_H_ */
