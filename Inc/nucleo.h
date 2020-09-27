/*
 * nucleo.h
 *
 *  Created on: Aug 22, 2020
 *      Author: joaquin
 */

#ifndef NUCLEO_H_
#define NUCLEO_H_

#include <stdbool.h>

#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"

#define NUCLEO_I2C_BUS (I2C_BUS_1)
#define NUCLEO_USART_BUS (USART_BUS_2)

void nucleo_init_button(void);
bool nucleo_is_button_pressed(void);
void wait_till_button_pressed(void);

void nucleo_init_i2c(i2c_interrupt_callback_t callback);

void nucleo_init_usart(usart_cfg_t *p_cfg);


#endif /* NUCLEO_H_ */
