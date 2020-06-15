/*
 * arduino.h
 *
 *  Created on: Jun 15, 2020
 *      Author: joaquin
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_

#include "stm32f446xx_spi_driver.h"
#include <stdbool.h>

typedef enum
{
    ARDUINO_DIGITAL_STATUS_OFF = 0u,
    ARDUINO_DIGITAL_STATUS_ON = 1u
} Arduino_digital_status_e;

typedef enum
{
    ARDUINO_ANALOG_PIN_0 = 0u,
    ARDUINO_ANALOG_PIN_1 = 1u,
    ARDUINO_ANALOG_PIN_2 = 2u,
    ARDUINO_ANALOG_PIN_3 = 3u,
    ARDUINO_ANALOG_PIN_4 = 4u,
} Arduino_analog_pin_e;

typedef enum
{
    ARDUINO_COMMAND_LED_CTRL = 0x50u,
    ARDUINO_COMMAND_SENSOR_READ = 0x51u,
    ARDUINO_COMMAND_LED_READ = 0x52u,
    ARDUINO_COMMAND_PRINT = 0x53u,
    ARDUINO_COMMAND_ID_READ = 0x54,
} Arduino_command_e;

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
} Arduino_digital_pin_e;

#define ARDUINO_ID_SIZE 10u

Arduino_digital_status_e Arduino_read_digital(Spi_handle_t *spi, Arduino_digital_pin_e pin);

void Arduino_write_led(Spi_handle_t *spi, Arduino_digital_status_e status, Arduino_digital_pin_e pin);

uint8_t Arduino_read_analog(Spi_handle_t *spi, Arduino_analog_pin_e pin);

void Arduino_print(Spi_handle_t *spi, const char message[]);

void Arduino_read_id(Spi_handle_t *spi, char id[], size_t length);

#endif /* ARDUINO_H_ */
