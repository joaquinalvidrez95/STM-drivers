/*
 * stm32f446xx.h
 *
 *  Created on: May 19, 2020
 *      Author: joaquin
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

/* ARM Cortex */
#define NVIC_ISER0 ((volatile uint32_t *)0xE000E100u)
#define NVIC_ISER1 ((volatile uint32_t *)0xE000E104u)
#define NVIC_ISER2 ((volatile uint32_t *)0xE000E108u)
#define NVIC_ISER3 ((volatile uint32_t *)0xE000E10Cu)

#define NVIC_ICER0 ((volatile uint32_t *)0xE000E180u)
#define NVIC_ICER1 ((volatile uint32_t *)0xE000E184u)
#define NVIC_ICER2 ((volatile uint32_t *)0xE000E188u)
#define NVIC_ICER3 ((volatile uint32_t *)0xE000E18Cu)

#define NO_PR_BITS_IMPLEMENTED 4u
#define NVIC_PR_BASE_ADDR ((volatile uint32_t *)0xE000E400U)

typedef enum
{
	irq_number_exti0 = 6u,
	irq_number_exti1 = 7u,
	irq_number_exti2 = 8u,
	irq_number_exti3 = 9u,
	irq_number_exti4 = 10,
	irq_number_exti9_5 = 23,
	irq_number_i2c1_ev = 31,
	irq_number_i2c1_er = 32,
	irq_number_spi1 = 35,
	irq_number_spi2 = 36,
	irq_number_usart1 = 37,
	irq_number_usart2 = 38,
	irq_number_usart3 = 39,
	irq_number_exti15_10 = 40,
	irq_number_spi3 = 51,
	irq_number_spi4,
	irq_number_uart4 = 52,
	irq_number_uart5 = 53,
	irq_number_usart6 = 71,
} Irq_number_t;

typedef enum
{
	nvic_irq_priority_0 = 0u,
	nvic_irq_priority_1,
	nvic_irq_priority_2,
	nvic_irq_priority_3,
	nvic_irq_priority_4,
	nvic_irq_priority_5,
	nvic_irq_priority_6,
	nvic_irq_priority_7,
	nvic_irq_priority_8,
	nvic_irq_priority_9,
	nvic_irq_priority_10,
	nvic_irq_priority_11,
	nvic_irq_priority_12,
	nvic_irq_priority_13,
	nvic_irq_priority_14,
	nvic_irq_priority_15,
} Nvic_irq_priority_t;

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
#define SPI4_BASEADDR (APB2PERIPH_BASE + 0x3400u)

#define EXTI_BASEADDR (APB2PERIPH_BASE + 0x3C00u)

#define USART1_BASEADDR (APB2PERIPH_BASE + 0x1000u)
#define USART6_BASEADDR (APB2PERIPH_BASE + 0x1400u)

#define SYSCFG_BASEADDR (APB2PERIPH_BASE + 0x3800u)

/** Registers */
typedef struct
{
	volatile uint32_t CR[2];
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} Spi_reg_t;

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

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} Exti_reg_t;

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t reserved1[2];
	volatile uint32_t CMPCR;
	uint32_t reserved2[2];
	volatile uint32_t CFGR;
} Syscfg_reg_t;

#define GPIOA ((Gpio_reg_t *)GPIOA_BASEADDR)
#define GPIOB ((Gpio_reg_t *)GPIOB_BASEADDR)
#define GPIOC ((Gpio_reg_t *)GPIOC_BASEADDR)
#define GPIOD ((Gpio_reg_t *)GPIOD_BASEADDR)
#define GPIOE ((Gpio_reg_t *)GPIOE_BASEADDR)
#define GPIOF ((Gpio_reg_t *)GPIOF_BASEADDR)
#define GPIOG ((Gpio_reg_t *)GPIOG_BASEADDR)
#define GPIOH ((Gpio_reg_t *)GPIOH_BASEADDR)

#define RCC ((Rcc_reg_t *)RCC_BASEADDR)
#define EXTI ((Exti_reg_t *)EXTI_BASEADDR)
#define SYSCFG ((Syscfg_reg_t *)SYSCFG_BASEADDR)
#define SYSCFG ((Syscfg_reg_t *)SYSCFG_BASEADDR)

#define SPI1 ((Spi_reg_t *)SPI1_BASEADDR)
#define SPI2 ((Spi_reg_t *)SPI2_BASEADDR)
#define SPI3 ((Spi_reg_t *)SPI3_BASEADDR)
#define SPI4 ((Spi_reg_t *)SPI4_BASEADDR)

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
#define SPI4_PCLK_EN() (RCC->APBENR[1] |= (1u << 13u))

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
uint8_t Driver_gpio_address_to_code(Gpio_reg_t *);
#endif /* INC_STM32F446XX_H_ */
