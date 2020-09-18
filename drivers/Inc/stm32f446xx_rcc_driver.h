/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Aug 12, 2020
 *      Author: joaquin
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include <stdint.h>
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"

uint32_t rcc_get_pclk1(void);
void rcc_set_i2c_peripheral_clock_enabled(i2c_bus_t bus, bool b_enabled);
void rcc_set_usart_peripheral_clock_enabled(usart_bus_t bus, bool b_enabled);

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
