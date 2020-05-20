/*
 * stm32f446xx.h
 *
 *  Created on: May 19, 2020
 *      Author: joaquin
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define FLASH_BASEADDR 0x08000000LU

/** 112KB */
#define SRAM1_BASEADDR 0x20000000LU

#define SRAM2_BASEADDR 0x20001C00LU

/** */
#define ROM_BASEADDR

#define SRAM SRAM1_BASEADDR

#define PERIPH_BASE 0x40000000LU
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000LU
#define AHB1PERIPH_BASE 0x40020000LU
#define AHB2PERIPH_BASE 0x50000000LU

/** AHB1 */
#define GPIOA_BASEADDR (AHB1PERIPH_BASE + 0x0000u)
#define GPIOB_BASEADDR (AHB1PERIPH_BASE + 0x0400u)
#define GPIOC_BASEADDR (AHB1PERIPH_BASE + 0x0800u)
#define GPIOD_BASEADDR (AHB1PERIPH_BASE + 0x0C00u)
#define GPIOE_BASEADDR (AHB1PERIPH_BASE + 0x1000u)
#define GPIOF_BASEADDR (AHB1PERIPH_BASE + 0x1400u)
#define GPIOG_BASEADDR (AHB1PERIPH_BASE + 0x1800u)
#define GPIOH_BASEADDR (AHB1PERIPH_BASE + 0x1C00u)

#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x3800u)

/** APB1 */
#define I2C1_BASEADDR (APB1PERIPH_BASE + 0x5400u)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0x5800u)
#define I2C3_BASEADDR (APB1PERIPH_BASE + 0x5C00u)

#define SPI2_BASEADDR (APB1PERIPH_BASE + 0x3800u)
#define SPI3_BASEADDR (APB1PERIPH_BASE + 0x3C00u)

#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400u)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800u)

#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00u)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000u)

/** APB2 */
#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000u)

#define EXTI_BASEADDR (APB2PERIPH_BASE + 0x3C00u)

#define USART1_BASEADDR (APB2PERIPH_BASE + 0x1000u)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0x1400u)

#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0x3800u)

/** Registers */
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDER;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} Gpio_reg_t;

typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLL_CFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB_RSTR[3];
	uint32_t RESERVED1;
	volatile uint32_t APB_RSTR[2];
	uint32_t RESERVED2[2];
	volatile uint32_t AHBENR[3];
	uint32_t RESERVED3;
	volatile uint32_t APBENR[2];
	uint32_t RESERVED4[2];
	volatile uint32_t AHB_LPENR[3];
	uint32_t RESERVED5;
	volatile uint32_t APB_LPENR[2];
	uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED7[2];
	volatile uint32_t SS_CGR;
	volatile uint32_t PLLI2_SCFGR;
	volatile uint32_t PLL_SAI_CFGR;
	volatile uint32_t DCK_CFGR;
	volatile uint32_t CK_GATENR;
	volatile uint32_t DCK_CFGR2;
} Rcc_reg_t;

#define GPIOA ((Gpio_reg_t *)GPIOA_BASEADDR)
#define GPIOB ((Gpio_reg_t *)GPIOB_BASEADDR)
#define GPIOC ((Gpio_reg_t *)GPIOC_BASEADDR)
#define GPIOD ((Gpio_reg_t *)GPIOD_BASEADDR)
#define GPIOE ((Gpio_reg_t *)GPIOE_BASEADDR)
#define GPIOF ((Gpio_reg_t *)GPIOF_BASEADDR)
#define GPIOG ((Gpio_reg_t *)GPIOG_BASEADDR)
#define GPIOH ((Gpio_reg_t *)GPIOH_BASEADDR)

#define RCC ((Rcc_reg_t *)RCC_BASEADDR)

/** Enables GPIOx */
#define GPIOA_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 0u))
#define GPIOB_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 1u))
#define GPIOC_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 2u))
#define GPIOD_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 3u))
#define GPIOE_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 4u))
#define GPIOF_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 5u))
#define GPIOG_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 6u))
#define GPIOH_PCLCK_EN() (RCC->AHBENR[0] |= (1u << 7u))

/** Disables GPIOx */
#define GPIOA_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 0u))
#define GPIOB_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 1u))
#define GPIOC_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 2u))
#define GPIOD_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 3u))
#define GPIOE_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 4u))
#define GPIOF_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 5u))
#define GPIOG_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 6u))
#define GPIOH_PCLCK_DI() (RCC->AHBENR[0] &= ~(1u << 7u))

/** Enables I2Cx */
#define I2C1_PCLK_EN() (RCC->APBENR[0] |= (1u << 21u))
#define I2C2_PCLK_EN() (RCC->APBENR[0] |= (1u << 22u))
#define I2C3_PCLK_EN() (RCC->APBENR[0] |= (1u << 23u))

/** Enables SPIx */
#define SPI1_PCLK_EN() (RCC->APBENR[1] |= (1u << 12u))
#define SPI2_PCLK_EN() (RCC->APBENR[0] |= (1u << 14u))
#define SPI3_PCLK_EN() (RCC->APBENR[0] |= (1u << 15u))

/** Enables USARTx */
#define USART1_PCLK_EN() (RCC->APBENR[1] |= (1u << 4u))
#define USART2_PCLK_EN() (RCC->APBENR[0] |= (1u << 17u))
#define USART3_PCLK_EN() (RCC->APBENR[0] |= (1u << 18u))
#define USART6_PCLK_EN() (RCC->APBENR[1] |= (1u << 5u))

/** Enables SYSCFG */
#define SYSCFG_PCLK_EN() (RCC->APBENR[1] |= (1u << 14u))

typedef enum
{
	Gpio_reset_port_a = 0u,
	Gpio_reset_port_b,
	Gpio_reset_port_c,
	Gpio_reset_port_d,
	Gpio_reset_port_e,
	Gpio_reset_port_f,
	Gpio_reset_port_g,
	Gpio_reset_port_h,
} Gpio_reset_t;

#define GPIOX_REG_RESET(port)                \
	do                                       \
	{                                        \
		(RCC->AHB_RSTR[0] |= (1u << port));  \
		(RCC->AHB_RSTR[0] &= ~(1u << port)); \
	} while (0u)

typedef enum
{
	En_status_disable_ = 0u,
	En_status_enable = 1u
} En_status_t;

typedef enum
{
	Gpio_pin_status_reset = 0u,
	Gpio_pin_status_set = 1u,
} Gpio_pin_status_t;

#define SET ENABLE
#define RESET DISABLE
// #define GPIO_PIN_SET SET
// #define GPIO_PIN_RESET RESET
#include "stm32f446xx_gpio_driver.h"

#endif /* INC_STM32F446XX_H_ */
