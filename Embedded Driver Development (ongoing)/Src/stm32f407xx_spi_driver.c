/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 2, 2024
 *      Author: User
 */

#include "stm32f407xx_spi_driver.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();

		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}

	}
	else
	{
		if(pSPIx == SPI1)
				{
					SPI1_PCLK_DI();
				}
				else if (pSPIx == SPI2)
				{
					SPI2_PCLK_DI();
				}
				else if (pSPIx == SPI3)
				{
					SPI3_PCLK_DI();
				}

	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, UInt8 EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
UInt32 tempReg = 0;

tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempReg &= ~(1 <<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempReg |= (1 <<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		tempReg &= ~(1 <<15);
		tempReg |= (1 <<10);
	}

	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	tempReg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM << 9;
	tempReg |= pSPIHandle->SPIConfig.SPI_SSI << 8;

	pSPIHandle->pSPIx->CR1 = tempReg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//pSPIHandle->pSPIx->CR1 = 0;
}

UInt8 SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, UInt32 FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, UInt8 *pTxBuffer, UInt32 Len)
{

	while(Len > 0)
	{
		// wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET );
		//while(! (pSPIx->SR *(1 << 1))); // 1 means empty -- ready to send

		//2. CHECK THE dff BIT
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit data format
			pSPIx->DR = *((UInt16*)pTxBuffer);
			Len--;
			Len--;
			(UInt16*)pTxBuffer++;
		}
		else
		{
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}

	}
}

















