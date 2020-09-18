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

#define NVIC_PR_BASE_ADDR ((volatile uint32_t *)0xE000E400U)

#define FLASH_BASEADDR 0x08000000LU

/** 112KB */
#define SRAM1_BASEADDR 0x20000000LU

#define SRAM2_BASEADDR 0x20001C00LU

/** */
#define ROM_BASEADDR

#define SRAM SRAM1_BASEADDR

#define PERIPH_BASE (0x40000000LU)
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE (0x40010000LU)
#define AHB1PERIPH_BASE (0x40020000LU)
#define AHB2PERIPH_BASE (0x50000000LU)

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
	struct
	{
		volatile unsigned int CPHA : 1;
		volatile unsigned int CPOL : 1;
		volatile unsigned int MSTR : 1;
		volatile unsigned int BR : 3;
		volatile unsigned int SPE : 1;
		volatile unsigned int LSBFIRST : 1;
		volatile unsigned int SSI : 1;
		volatile unsigned int SSM : 1;
		volatile unsigned int RXONLY : 1;
		volatile unsigned int DFF : 1;
		volatile unsigned int CRCNEXT : 1;
		volatile unsigned int CRCEN : 1;
		volatile unsigned int BIDIOE : 1;
		volatile unsigned int BIDIMODE : 1;
		unsigned int reserved : 16;
	} CR1;
	struct
	{
		volatile unsigned int RXDMAEN : 1;
		volatile unsigned int TXDMAEN : 1;
		volatile unsigned int SSOE : 1;
		unsigned int reserved_0 : 1;
		volatile unsigned int FRF : 1;
		volatile unsigned int ERRIE : 1;
		volatile unsigned int RXNEIE : 1;
		volatile unsigned int TXEIE : 1;
		unsigned int reserved_1 : 24;
	} CR2;

	struct
	{
		volatile unsigned int RXNE : 1;
		volatile unsigned int TXE : 1;
		volatile unsigned int CHSIDE : 1;
		volatile unsigned int UDR : 1;
		volatile unsigned int CRCERR : 1;
		volatile unsigned int MODF : 1;
		volatile unsigned int OVR : 1;
		volatile unsigned int BSY : 1;
		volatile unsigned int FRE : 1;
		unsigned int reserved : 23;
	} SR;

	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
} Spi_reg_t;

typedef struct
{
	uint16_t CR1;
	const uint16_t _unused_0;
	uint16_t CR2;
	const uint16_t _unused_1;
	uint16_t OAR1;
	const uint16_t _unused_2;
	uint16_t OAR2;
	const uint16_t _unused_3;
	uint8_t DR;
	const uint8_t _unused_4;
	const uint16_t _unused_5;
	uint16_t SR1;
	const uint16_t _unused_6;
	uint16_t SR2;
	const uint16_t _unused_7;
	uint16_t CCR;
	const uint16_t _unused_8;
	uint16_t TRISE;
	const uint16_t _unused_9;
	uint16_t FLTR;
	const uint16_t _unused_10;
} i2c_reg_t;

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
} gpio_reg_t;

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
} rcc_reg_t;

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} exti_reg_t;

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

#define GPIOA ((gpio_reg_t *const)GPIOA_BASEADDR)
#define GPIOB ((gpio_reg_t *const)GPIOB_BASEADDR)
#define GPIOC ((gpio_reg_t *const)GPIOC_BASEADDR)
#define GPIOD ((gpio_reg_t *const)GPIOD_BASEADDR)
#define GPIOE ((gpio_reg_t *const)GPIOE_BASEADDR)
#define GPIOF ((gpio_reg_t *const)GPIOF_BASEADDR)
#define GPIOG ((gpio_reg_t *const)GPIOG_BASEADDR)
#define GPIOH ((gpio_reg_t *const)GPIOH_BASEADDR)

#define RCC ((rcc_reg_t *)RCC_BASEADDR)
#define EXTI ((exti_reg_t *)EXTI_BASEADDR)
#define SYSCFG ((Syscfg_reg_t *)SYSCFG_BASEADDR)
#define SYSCFG ((Syscfg_reg_t *)SYSCFG_BASEADDR)

#define SPI1 ((Spi_reg_t *)SPI1_BASEADDR)
#define SPI2 ((Spi_reg_t *)SPI2_BASEADDR)
#define SPI3 ((Spi_reg_t *)SPI3_BASEADDR)
#define SPI4 ((Spi_reg_t *)SPI4_BASEADDR)

#define I2C1 ((volatile i2c_reg_t *)I2C1_BASEADDR)
#define I2C2 ((volatile i2c_reg_t *)I2C2_BASEADDR)
#define I2C3 ((volatile i2c_reg_t *)I2C3_BASEADDR)

/* TODO: Move to RCC module */
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

/** Enables SPIx */
#define SPI1_PCLK_EN() (RCC->APBENR[1] |= (1u << 12u))
#define SPI2_PCLK_EN() (RCC->APBENR[0] |= (1u << 14u))
#define SPI3_PCLK_EN() (RCC->APBENR[0] |= (1u << 15u))
#define SPI4_PCLK_EN() (RCC->APBENR[1] |= (1u << 13u))

#define SPI1_PCLK_DI() (RCC->APBENR[1] &= ~(1u << 12u))
#define SPI2_PCLK_DI() (RCC->APBENR[0] &= ~(1u << 14u))
#define SPI3_PCLK_DI() (RCC->APBENR[0] &= ~(1u << 15u))
#define SPI4_PCLK_DI() (RCC->APBENR[1] &= ~(1u << 13u))

/** Enables USARTx */
#define USART1_PCLK_EN() (RCC->APBENR[1] |= (1u << 4u))
#define USART2_PCLK_EN() (RCC->APBENR[0] |= (1u << 17u))
#define USART3_PCLK_EN() (RCC->APBENR[0] |= (1u << 18u))
#define USART6_PCLK_EN() (RCC->APBENR[1] |= (1u << 5u))

/** Enables SYSCFG */
#define SYSCFG_PCLK_EN() (RCC->APBENR[1] |= (1u << 14u))

/** Resets peripheral */
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
} gpio_reset_t;

#define GPIOX_REG_RESET(port)                \
	do                                       \
	{                                        \
		(RCC->AHB_RSTR[0] |= (1u << port));  \
		(RCC->AHB_RSTR[0] &= ~(1u << port)); \
	} while (0u)

/* SPI */
#define SPI1_REG_RESET()                    \
	do                                      \
	{                                       \
		(RCC->APB_RSTR[1] |= (1u << 12u));  \
		(RCC->APB_RSTR[1] &= ~(1u << 12u)); \
	} while (0u)

#define SPI2_REG_RESET()                    \
	do                                      \
	{                                       \
		(RCC->APB_RSTR[0] |= (1u << 14u));  \
		(RCC->APB_RSTR[0] &= ~(1u << 14u)); \
	} while (0u)

#define SPI3_REG_RESET()                    \
	do                                      \
	{                                       \
		(RCC->APB_RSTR[0] |= (1u << 15u));  \
		(RCC->APB_RSTR[0] &= ~(1u << 15u)); \
	} while (0u)

#define SPI4_REG_RESET()                    \
	do                                      \
	{                                       \
		(RCC->APB_RSTR[1] |= (1u << 13u));  \
		(RCC->APB_RSTR[1] &= ~(1u << 13u)); \
	} while (0u)

#define SET ENABLE
#define RESET DISABLE
uint8_t Driver_gpio_address_to_code(gpio_reg_t *);
#endif /* INC_STM32F446XX_H_ */
