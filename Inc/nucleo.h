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

void nucleo_init_button(void);
void nucleo_init_i2c(i2c_handle_t *p_handle);
bool nucleo_is_button_pressed(void);

#endif /* NUCLEO_H_ */
