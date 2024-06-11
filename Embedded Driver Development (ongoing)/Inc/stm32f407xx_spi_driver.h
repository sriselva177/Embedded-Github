/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 2, 2024
 *      Author: User
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

typedef struct
{
	UInt8 SPI_DeviceMode;
	UInt8 SPI_BusConfig;
	UInt8 SPI_SclkSpeed;
	UInt8 SPI_DFF;
	UInt8 SPI_CPOL;
	UInt8 SPI_CPHA;
	UInt8 SPI_SSM;
	UInt8 SPI_SSI;
}SPI_Config_t;

typedef struct

{	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

//bit position

#define SPI_CR1_DFF		11
#define SPI_CR1_SPE		6



#define SPI_SR_RXNE		0
#define SPI_SR_TXE		1
#define SPI_SR_CHSIDE	2
#define SPI_SR_UDR		3
#define SPI_SR_CRCERR	4
#define SPI_SR_MODF		5
#define SPI_SR_OVR		6
#define SPI_SR_BSY		7
#define SPI_SR_FRE		8

#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

#define SPI_BUS_CONFIG_FD			0
#define SPI_BUS_CONFIG_HD			1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3

#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

#define SPI_SSM_EN		1
#define SPI_SSM_DI		0

#define SPI_SSI_EN		1
#define SPI_SSI_DI		0

#define SPI_TXE_FLAG (1 << SPI_SR_TXE)

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, UInt8 *pTxBuffer, UInt32 Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, UInt8 *pRxBuffer, UInt32 Len);

//IRQ configuration
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
void SPI_IRQPriorityConfig(int8_t IRQNumber, uint32_t IRQPriority);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, UInt8 EnOrDi);


#endif  INC_STM32F407XX_SPI_DRIVER_H_
