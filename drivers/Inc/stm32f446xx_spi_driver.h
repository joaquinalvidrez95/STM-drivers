/*
 * stm32f446xx_spi_driver.h
 *
 *  Created on: May 21, 2020
 *      Author: joaquin
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include <stdint.h>

typedef struct
{
    uint8_t device_mode;
    uint8_t bus_config;
    uint8_t sclk_speed;
    uint8_t DFF;
    uint8_t CPOL;
    uint8_t CPHA;
    uint8_t SSM;
} Spi_config_t;

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
