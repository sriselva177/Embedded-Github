/*
 * stm32f407xx_gpio_drivefr.c
 *
 *  Created on: May 21, 2024
 *      Author: User
 */
/**************************************************
 *@fn			- GPIO_PeriClockControl
 *
 *@brief		-
 *@
 *@param[in]	-
 *@param[in]	-
 *@param[in]	-
 *@
 *@return		-
 *
 *@Note			-
 */

#include "stm32f407xx_gpio_driver.h"


/**************************************************
 *@fn			- GPIO_PeriClockControl
 *
 *@brief		-This function enable or disables peripheral clock for given GPIO port
 *@
 *@param[in]	- Base address of the GPIO peripheral
 *@param[in]	- ENABLE OR DISABLE macro
 *@param[in]	-
 *@
 *@return		- None
 *
 *@Note			- None
 *
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();

		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();

		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
				else if (pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}
				else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
				else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
				else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}
				else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
				else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}
				else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
				else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	UInt32 temp;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//1 configure the mode
	if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
			temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle ->pGPIOx->MODER &= ~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
			pGPIOHandle ->pGPIOx->MODER |= temp;

	}
	else
	{
		//the part will code later
		if(pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			// configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber); // CLEAR THE RTSR
		}
		else if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
				{
			// configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber); // CLEAR THE RTSR
				}
		else if (pGPIOHandle ->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
		}

		// confuigure the gpio port selection in SYSCFG_EXTIVR
		UInt8 temp1 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber / 4;
		UInt8 temp2 = pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber % 4;
		UInt8 portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		SYSCFG_PCLK_EN();
		// enable the exti interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;
	//configure the speed
	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle ->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle ->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//configure the pupd settings
	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle ->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx->PUPDR |= temp;

	temp = 0;
	//configure op type
	temp = (pGPIOHandle ->GPIO_PinConfig.GPIO_PinOPType <<(1 * pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle ->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle ->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle -> pGPIOx->OTYPER |= temp;

	temp = 0;
	//configure the alt functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		UInt32 temp1,temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4 * temp2);




	}



}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();

	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_PCLK_EN();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

uint16_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	UInt16 value;
	value = (UInt8)((pGPIOx->IDR >> PinNumber)& 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	UInt16 value;
	value = (UInt16)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber); //write 1
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber); // write 0
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//program ISER Regiater  //To enable interrupt
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if ( IRQNumber >31 && IRQNumber <64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
			//program ISER1 Regiater
		}
		else if (IRQNumber >=64 && IRQNumber < 96)
		{
			//program ISER2 Regiater
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}

	}else
	{
		if (IRQNumber <= 31)
		{
			//program ICER Regiater
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		}
		else if ( IRQNumber >31 && IRQNumber <64)
		{
			//program ISER1 Regiater
			*NVIC_ICER1 &= ~(1 << (IRQNumber % 32));
		}
		else if (IRQNumber >=64 && IRQNumber < 96)
		{
			//program ISER2 Regiater
			*NVIC_ICER3 &= ~(1 << (IRQNumber % 64));
		}
	}

}

void GPIO_IRQPriorityConfig(int8_t IRQNumber, uint32_t IRQPriority)
{
	//1. find our IPR register
	UInt8 iprx = IRQNumber / 4;
	UInt8 iprx_section = IRQNumber % 4;
	UInt8 shift_amount = ( 8 * iprx_section) + (8 + NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
