/*
 * stm32f407xx.h
 *
 *  Created on: May 18, 2024
 *      Author: User
 */
#include <stdint.h>

typedef unsigned char UInt8;
typedef unsigned short int UInt16;
typedef unsigned int UInt32;

#define NVIC_ISER0	((volatile UInt32*)(0xE000E100))
#define NVIC_ISER1	((volatile UInt32*)(0xE000E104))
#define NVIC_ISER2	((volatile UInt32*)(0xE000E108))
#define NVIC_ISER3	((volatile UInt32*)(0xE000E10C))

#define NVIC_ICER0	((volatile UInt32*)(0xE000E180))
#define NVIC_ICER1	((volatile UInt32*)(0xE000E184))
#define NVIC_ICER2	((volatile UInt32*)(0xE000E188))
#define NVIC_ICER3	((volatile UInt32*)(0xE000E18C))

#define NVIC_PR_BASE_ADDR	((volatile UInt32*)(0xE000E400))

#define NO_PR_BITS_IMPLEMENTED	4

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U

#define SRAM2_BASEADDR			0x2001C000U
#define ROM						0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

// Bus Base Addresses
#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB2PERIPH_BASE			0x50000000U
#define AHB1PERIPH_BASE 		0x40020000U


//AHB1 Bus
#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0X0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0X0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0X0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0X1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0X1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0X1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0X1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE + 0X2000)
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)

//APB1 Bus
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0X5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0X5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0X5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0X3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0X3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0X4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0X4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0X4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0X5000)

//APB2 BUS
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0X3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0X1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0X1400)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0X3800)

//SYSCFG



#ifndef GPIO_REGDEF_T_H
#define GPIO_REGDEF_T_H
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;
#endif // RCC_REGDEF_T_H

#ifndef SPI_REGDEF_T_H
#define SPI_REGDEF_T_H
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

}SPI_RegDef_t;
#endif // RCC_REGDEF_T_H




#ifndef RCC_REGDEF_T_H
#define RCC_REGDEF_T_H
typedef struct
{
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t 	RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;
}RCC_RegDef_t;
#endif // RCC_REGDEF_T_H

#ifndef EXTI_REGDEF_T_H
#define EXTI_REGDEF_T_H
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_RegDef_t;
#endif // EXTI_REGDEF_T_H

#ifndef SYSCFG_REGDEF_T_H
#define SYSCFG_REGDEF_T_H
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	volatile uint32_t RESERVED2[2];
	volatile uint32_t CFGR;

}SYSCFG_RegDef_t;
#endif // EXTI_REGDEF_T_H



#define GPIOA	 ((GPIO_RegDef_t*)(GPIOA_BASEADDR))
#define GPIOB	 ((GPIO_RegDef_t*)(GPIOB_BASEADDR))
#define GPIOC	 ((GPIO_RegDef_t*)(GPIOC_BASEADDR))
#define GPIOD	 ((GPIO_RegDef_t*)(GPIOD_BASEADDR))
#define GPIOE	 ((GPIO_RegDef_t*)(GPIOE_BASEADDR))
#define GPIOF	 ((GPIO_RegDef_t*)(GPIOF_BASEADDR))
#define GPIOG	 ((GPIO_RegDef_t*)(GPIOG_BASEADDR))
#define GPIOH	 ((GPIO_RegDef_t*)(GPIOH_BASEADDR))
#define GPIOI	 ((GPIO_RegDef_t*)(GPIOI_BASEADDR))

#define RCC 	 ((RCC_RegDef_t *)(RCC_BASEADDR))

#define EXTI	((EXTI_RegDef_t *)(EXTI_BASEADDR))

#define SYSCFG	((SYSCFG_RegDef_t*)(SYSCFG_BASEADDR))

#define SPI1	((SPI_RegDef_t*)(SPI1_BASEADDR))
#define SPI2	((SPI_RegDef_t*)(SPI2_BASEADDR))
#define SPI3	((SPI_RegDef_t*)(SPI3_BASEADDR))



//(RCC_RegDef_t*((0x40020000U) + 0X3800)->AHB1ENR | (1<<0))
// cLOCK ENABLE MACROS FOR GPIO PHERIPHERALS
#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()	(RCC-> AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()	(RCC-> AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()	((RCC)-> AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()	((RCC)-> AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()	((RCC)-> AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()	((RCC)-> AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()	((RCC)-> AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()	((RCC)-> AHB1ENR |= (1<<8))

// cLOCK ENABLE MACROS FOR I2C PHERIPHERALS
#define I2C1_PCLK_EN()	((RCC)->APB1ENR | (1<<21))
#define I2C2_PCLK_EN()	((RCC)->APB1ENR | (1<<22))
#define I2C3_PCLK_EN()	((RCC)->APB1ENR | (1<<23))


// cLOCK ENABLE MACROS FOR SPI PHERIPHERALS
#define SPI1_PCLK_EN()	((RCC)->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()	((RCC)->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()	((RCC)->APB1ENR |= (1<<15))

// cLOCK ENABLE MACROS FOR USART PHERIPHERALS
#define USART1_PCLK_EN()	((RCC)->APB2ENR | (1<<4))
#define USART2_PCLK_EN()	((RCC)->APB1ENR | (1<<17))
#define USART3_PCLK_EN()	((RCC)->APB1ENR | (1<<18))
#define UART4_PCLK_EN()		((RCC)->APB1ENR | (1<<19))
#define UART5_PCLK_EN()		((RCC)->APB1ENR | (1<<20))
#define USART6_PCLK_EN()	((RCC)->APB2ENR | (1<<5))

// cLOCK ENABLE MACROS FOR SYSCFG PHERIPHERALS
#define SYSCFG_PCLK_EN()	((RCC)->APB2ENR | (1<<14))


// cLOCK DISABLE MACROS FOR GPIO PHERIPHERALS
#define GPIOA_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()	((RCC)->AHB1ENR &= ~(1<<8))

// cLOCK DISABLE MACROS FOR I2C PHERIPHERALS
#define I2C1_PCLK_DI()	((RCC)->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()	((RCC)->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()	((RCC)->APB1ENR &= ~(1<<23))

// cLOCK DISABLE MACROS FOR SPI PHERIPHERALS
#define SPI1_PCLK_DI()	((RCC)->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()	((RCC)->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()	((RCC)->APB1ENR &= ~(1<<15))

// cLOCK DISABLE MACROS FOR USART PHERIPHERALS
#define USART1_PCLK_DI()	((RCC)->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()	((RCC)->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()	((RCC)->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()		((RCC)->APB1ENR &= ~((1<<19))
#define UART5_PCLK_DI()		((RCC)->APB1ENR &= ~((1<<20))
#define USART6_PCLK_DI()	((RCC)->APB2ENR &= ~(1<<5))

// cLOCK DISABLE MACROS FOR SYSCFG PHERIPHERALS
#define SYSCFG_PCLK_DI()	((RCC)->APB2ENR &= ~(1<<14))


#define GPIOA_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<0));	((RCC)->AHB1RSTR &= ~(1<<0));} while(0)
#define GPIOB_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<1));	((RCC)->AHB1RSTR &= ~(1<<1));} while(0)
#define GPIOC_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<2));	((RCC)->AHB1RSTR &= ~(1<<2));} while(0)
#define GPIOD_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<3));	((RCC)->AHB1RSTR &= ~(1<<3));} while(0)
#define GPIOE_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<4));	((RCC)->AHB1RSTR &= ~(1<<4));} while(0)
#define GPIOF_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<5));	((RCC)->AHB1RSTR &= ~(1<<5));} while(0)
#define GPIOG_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<6));	((RCC)->AHB1RSTR &= ~(1<<6));} while(0)
#define GPIOH_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<7));	((RCC)->AHB1RSTR &= ~(1<<7));} while(0)
#define GPIOI_REG_RESET()	do {((RCC)->AHB1RSTR |= (1<<8));	((RCC)->AHB1RSTR &= ~(1<<8));} while(0)

#define GPIO_BASEADDR_TO_CODE(x) 	((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
								    (x == GPIOD) ? 3 :\
								    (x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8:0 )


#define IRQ_N0_EXTI0	6
#define IRQ_N0_EXTI1	7
#define IRQ_N0_EXTI2	8
#define IRQ_N0_EXTI3	9
#define IRQ_N0_EXTI4	10
#define IRQ_N0_EXTI9_5	23
#define IRQ_N0_EXTI15_10	40


#define ENABLE 1
#define DISABLE 0

#define SET		ENABLE
#define RESET	DISABLE

#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET 	RESET

#define FLAG_RESET		RESET
#define FLAG_SET		SET
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"

