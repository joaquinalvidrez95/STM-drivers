/*
 * stm32f407xx_nvic_driver.h
 *
 *  Created on: Aug 29, 2020
 *      Author: joaquin
 */

#ifndef INC_STM32F446XX_NVIC_DRIVER_H_
#define INC_STM32F446XX_NVIC_DRIVER_H_

#include <stdbool.h>

typedef enum
{
	NVIC_IRQ_NUM_EXTI0 = 6u,
	NVIC_IRQ_NUM_EXTI1 = 7u,
	NVIC_IRQ_NUM_EXTI2 = 8u,
	NVIC_IRQ_NUM_EXTI3 = 9u,
	NVIC_IRQ_NUM_EXTI4 = 10,
	NVIC_IRQ_NUM_EXTI9_5 = 23,
	NVIC_IRQ_NUM_I2C1_EV = 31,
	NVIC_IRQ_NUM_I2C1_ER = 32,
	NVIC_IRQ_NUM_I2C2_EV = 33,
	NVIC_IRQ_NUM_I2C2_ER = 34,
	NVIC_IRQ_NUM_SPI1 = 35,
	NVIC_IRQ_NUM_SPI2 = 36,
	NVIC_IRQ_NUM_USART1 = 37,
	NVIC_IRQ_NUM_USART2 = 38,
	NVIC_IRQ_NUM_USART3 = 39,
	NVIC_IRQ_NUM_EXTI15_10 = 40,
	NVIC_IRQ_NUM_SPI3 = 51,
	NVIC_IRQ_NUM_SPI4,
	NVIC_IRQ_NUM_UART4 = 52,
	NVIC_IRQ_NUM_UART5 = 53,
	NVIC_IRQ_NUM_USART6 = 71,
	NVIC_IRQ_NUM_I2C3_EV = 72,
	NVIC_IRQ_NUM_I2C3_ER = 73,
} nvic_irq_num_t;

/**
 * @brief Priorities
 * 
 */
typedef enum
{
	NVIC_IRQ_PRIO_0 = 0u,
	NVIC_IRQ_PRIO_1,
	NVIC_IRQ_PRIO_2,
	NVIC_IRQ_PRIO_3,
	NVIC_IRQ_PRIO_4,
	NVIC_IRQ_PRIO_5,
	NVIC_IRQ_PRIO_6,
	NVIC_IRQ_PRIO_7,
	NVIC_IRQ_PRIO_8,
	NVIC_IRQ_PRIO_9,
	NVIC_IRQ_PRIO_10,
	NVIC_IRQ_PRIO_11,
	NVIC_IRQ_PRIO_12,
	NVIC_IRQ_PRIO_13,
	NVIC_IRQ_PRIO_14,
	NVIC_IRQ_PRIO_15,
} nvic_irq_prio_t;

void nvic_set_irq_enabled(nvic_irq_num_t irq_num, bool b_enabled);
void nvic_set_irq_priority(nvic_irq_num_t irq_num, nvic_irq_prio_t prio);

#endif /* INC_STM32F446XX_NVIC_DRIVER_H_ */
