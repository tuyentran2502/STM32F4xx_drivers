/*
 * stm32f407xx_gpio_drivers.c
 *
 *  Created on: Jul 7, 2021
 *      Author: Hi
 */
#include "stm32f407xx_gpio_drivers.h"

/**************************************************************************************************************
  * @fn              	-GPIO_PeriClkControl
  * @brief  			-This function enables or disable peripheral clock for the given GPIO port
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx ,uint8_t ENorDI )
{
	if(ENorDI==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx==GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if(pGPIOx==GPIOA)
				{
					GPIOA_PCLK_DI();
				}else if(pGPIOx==GPIOB)
				{
					GPIOB_PCLK_DI();
				}else if(pGPIOx==GPIOC)
				{
					GPIOC_PCLK_DI();
				}else if(pGPIOx==GPIOD)
				{
					GPIOD_PCLK_DI();
				}else if(pGPIOx==GPIOE)
				{
					GPIOE_PCLK_DI();
				}else if(pGPIOx==GPIOF)
				{
					GPIOF_PCLK_DI();
				}else if(pGPIOx==GPIOG)
				{
					GPIOG_PCLK_DI();
				}else if(pGPIOx==GPIOH)
				{
					GPIOH_PCLK_DI();
				}else if(pGPIOx==GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}
}

/**************************************************************************************************************
  * @fn              	-GPIO_Init
  * @brief  			-This function configure for the  GPIO port
  * @param[in]  		- *pGPIOHandle : handle struct of the GPIO peripheral
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//enable clock
	GPIO_PeriClkControl(pGPIOHandle->pGPIO,ENABLE);

	uint32_t temp;
     //1.Configure the mode of the gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_ANALOG_MODE)
	{
		// the none interrupt mode
		temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIO->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
		pGPIOHandle->pGPIO->MODER |=temp;
	}else
	{
		// the interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_IT_FT_MODE)
		{
			//1.Configure the FTSR
			EXTI->FTSR |=( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_IT_RT_MODE)
		{
			//1.Configure the RTSR
			EXTI->RTSR |=( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_IT_FRT_MODE)
		{
			//1.Configure the FTSR and RTSR
			EXTI->FTSR |=( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |=( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2.Configure the GPIO port selection in  SYSCFG_EXTICR
		 uint8_t temp1,temp2;
		 temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		 temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		 uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIO);

		 SYSCFG_CLK_EN();

		 SYSCFG->EXTICR[temp1] |=(portcode <<(temp2*4));
		//3.Enable external interrupt delivery using IRM
		EXTI->IRM |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp=0;
	 //2.Configure the speed
	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIO->OSPEEDR &= ~(0x03 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIO->OSPEEDR |=temp;

	temp=0;
	 //3.Configure the pu/pd mode
	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIO->PUPDR &= ~(0x03 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIO->PUPDR |=temp;

	temp=0;

	 //4.Configure the output type
	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIO->OTYPER &= ~(0x01 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing
	pGPIOHandle->pGPIO->OTYPER |=temp;

	temp=0;

	 //5.Configure the alternate function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode == GPIO_ALT_FUNC_MODE)
	{
		//configure the alternate function registers
		uint32_t temp1,temp2;
		temp1 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIO->AFR[temp1] &= ~ (0x0F <<(4*temp2));
		pGPIOHandle->pGPIO->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2) );
	}

}

/**************************************************************************************************************
  * @fn              	-GPIO_DeInit
  * @brief  			-This function enables or disable peripheral clock for the given GPIO port
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx==GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx==GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx==GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/**************************************************************************************************************
  * @fn              	-GPIO_ReadFromInputPin
  * @brief  			-This function read the pin of the GPIO peripheral
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- PinNumber :[0->15]
  *
  * @return             -0 or 1
  *
  * @Note				-none
  *
  ************************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	uint8_t value;
	value= (uint8_t)(pGPIOx ->IDR >> PinNumber) & (0x00000001);
 return value;
}

/**************************************************************************************************************
  * @fn              	-GPIO_ReadFromInputPort
  * @brief  			-This function read the port GPIO
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @return             - 0x0000 -> 0xFFFF
  *
  * @Note				-none
  *
  ************************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
		value= pGPIOx ->IDR ;
	 return value;
}

/**************************************************************************************************************
  * @fn              	-GPIO_WriteToOutputPin
  * @brief  			-This function write the pin GPIO
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- Value : GPIO_PIN_SET or GPIO_PIN_RESET
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value)
{
   if(Value == GPIO_PIN_SET)
   {
	   pGPIOx->ODR |=(1<<PinNumber);
   }else
   {
	   pGPIOx->ODR &= ~(1<<PinNumber);
   }
}

/**************************************************************************************************************
  * @fn              	-GPIO_WriteToOutputPort
  * @brief  			-This function write the port GPIO
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- Value : 0x0000->0xFFFF
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value)
{
   pGPIOx->ODR =Value;
}

/**************************************************************************************************************
  * @fn              	-GPIO_ToggleOutputPin
  * @brief  			-This function toggle the pin GPIO
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- PinNumber : [0->15]

  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<<PinNumber);
}

/**************************************************************************************************************
  * @fn              	-GPIO_IQR_Config
  * @brief  			-This function configure IQR GPIO
  * @param[in]  		-Number of GPIO IQR
  * @param[in] 			- ENorDI  : enable or disable (ENABLE ,DISABLE macros)
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_IQRInterrupt_Config(uint8_t IQRNumber,uint8_t ENorDI)
{
		if(ENorDI==ENABLE)
		{
			if(IQRNumber <=31)
			{
				//program ISER0 register
				*NIVC_ISER0 |=(1<<IQRNumber);

			}else if(IQRNumber >31 && IQRNumber <64)
			{
				//program ISER1 register
				*NIVC_ISER1 |=(1<<IQRNumber);

			}else if(IQRNumber>=64  && IQRNumber < 96)
			{
				//program ISER2 register
				*NIVC_ISER2 |=(1<<IQRNumber);
			}
		}else
		{
			if(IQRNumber <=31)
			{
				//program ICER0 register
				*NIVC_ICER0 |=(1<<IQRNumber);
			}else if(IQRNumber >31 && IQRNumber <64)
			{
				//program ICER1 register
				*NIVC_ICER1 |=(1<<IQRNumber % 32);

			}else if(IQRNumber>=64 && IQRNumber <96)
			{
				//program ICER2 register
				*NIVC_ICER2 |=(1<<IQRNumber % 64);
			}
		}
}

/**************************************************************************************************************
  * @fn              	-GPIO_IQRPriority_Config
  * @brief  			-This function enables or disable peripheral clock for the given GPIO port
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_IQRPriority_Config(uint8_t IQRNumber,uint8_t Priority )
{

	//1.first lets find out the priority register
    uint8_t iprx = IQRNumber/4;
    uint8_t iprx_sectin = IQRNumber % 4 ;
    uint8_t shift_amount = (8 *iprx_sectin) + (8-NO_PR_BITS_IMPLEMENTED);//4 bits lower not implemented
    *(NIVC_PR_BASEADDR+ iprx) |=(Priority << shift_amount );/* IPR0->IPR59,each IPRs have 4 register 8bit IQR,implemented 4bits high*/
}


/**************************************************************************************************************
  * @fn              	-GPIO_Init
  * @brief  			-This function enables or disable peripheral clock for the given GPIO port
  * @param[in]  		- *pGPIOx : base address of the GPIO peripheral
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void GPIO_IQR_Handling(uint8_t PinNumber)
{
	 //clear exti pr register corresponding to the pin number
	if(EXTI->PR & (1<<PinNumber))
	{
		EXTI->PR |= (1<<PinNumber); // In order clear bit , programing it to '1'
	}
}
