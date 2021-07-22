/*
 * stm32f407xx_spi_drivers.c
 *
 *  Created on: Jul 9, 2021
 *      Author: Hi
 */
#include "stm32f407xx_spi_drivers.h"

static void  spi_over_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*
 * Peripheral clock setup
 */
/**************************************************************************************************************
  * @fn              	-SPI_PeriClkControl
  * @brief  			-This function enables or disable peripheral clock for the given SPI port
  * @param[in]  		- *pSPIx : base address of the GPIO peripheral
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_PeriClkControl(SPI_RegDef_t *pSPIx ,uint8_t ENorDI )
{
	if(ENorDI==ENABLE)
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx==SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx==SPI3)
			{
				SPI3_PCLK_EN();
			}else if(pSPIx==SPI4)
			{
				SPI4_PCLK_EN();
			}else if(pSPIx==SPI5)
			{
				SPI5_PCLK_EN();
			}else if(pSPIx==SPI6)
			{
				SPI6_PCLK_EN();
			}
		}else
		{
			if(pSPIx==SPI1)
			{
				SPI1_PCLK_DI();
			}else if(pSPIx==SPI2)
			{
				SPI2_PCLK_DI();
			}else if(pSPIx==SPI3)
			{
				SPI3_PCLK_DI();
			}else if(pSPIx==SPI4)
			{
				SPI4_PCLK_DI();
			}else if(pSPIx==SPI5)
			{
				SPI5_PCLK_DI();
			}else if(pSPIx==SPI6)
			{
				SPI6_PCLK_DI();
			}
		}
}

/*
 * Init and de-Init
 */
/**************************************************************************************************************
  * @fn              	-SPI_Init
  * @brief  			-This function cofigure SPI mode
  * @param[in]  		-*pSPIHandle:type of @SPI_Handle_t

  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//enable clock
	SPI_PeriClkControl(pSPIHandle->pSPIx,ENABLE);
	// first let configure the SPI_CR1 register
	uint8_t tempreg=0;
	//1 Configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode <<SPI_CR1_MSTR;
	//2.Configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig==SPI_CONFIG_FULL_DUPLEX)
	{
		// clear bidi bit
		tempreg &= ~(1<<SPI_CR1_BIDIMODE );

	}else if(pSPIHandle->SPI_Config.SPI_BusConfig==SPI_CONFIG_HAFL_DUPLEX)
	{
		//bidi bit set
		tempreg |= (1<<SPI_CR1_BIDIMODE );
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig==SPI_CONFIG_SIMPLEX_ONLY_RX)
	{
		//biddi bit should be clear
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		//RXONLY bit set
		tempreg |= (1<<SPI_CR1_RXONLY);
	}
	// configure baud rate speed
	tempreg |= pSPIHandle->SPI_Config.SPI_Baud_Rate_Speed <<SPI_CR1_BR;
	//configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF <<SPI_CR1_DFF;
	//configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL <<SPI_CR1_CPOL;
	//configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA <<SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 |= tempreg;
}
/**************************************************************************************************************
  * @fn              	-SPI_Init
  * @brief  			-This function clear ,reset SPI register
  * @param[in]  		-*pSPIx :type of @SPI_RegDef_t

  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1)
				{
					SPI1_REG_RESET();
				}else if(pSPIx==SPI2)
				{
					SPI2_REG_RESET();
				}else if(pSPIx==SPI3)
				{
					SPI3_REG_RESET();
				}else if(pSPIx==SPI4)
				{
					SPI4_REG_RESET();
				}else if(pSPIx==SPI5)
				{
					SPI5_REG_RESET();
				}else if(pSPIx==SPI6)
				{
					SPI6_REG_RESET();
				}

}
/**************************************************************************************************************
  * @fn              	-SPI_GetFlagStatus
  * @brief  			-This function check the Flag status
  * @param[in]  		-*pSPIx :type of @SPI_RegDef_t
  * @param[in]  		- FlagName :reference of @FlagName
  * @return             - Status : FLAG_SET or FLAG_RESET
  *
  * @Note				-none
  *
  ************************************************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Transmit and Received data
 */
/**************************************************************************************************************
  * @fn              	-SPI_TransmitData
  * @brief  			-This function send data
  * @param[in]  		-*pSPIx :type of @SPI_RegDef_t
  * @param[in]  		-pTxBuffer : string transmit data
  * @param[in]  		- Len:leng of string pTxbuffer
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_TransmitData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer ,uint8_t Len)
{
	while(Len >0)
	{
		//wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET);
		// check DFF bit of SPI_CR1 register
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			// 16bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
/**************************************************************************************************************
  * @fn              	-SPI_ReceiveData
  * @brief  			-This function received data
  * @param[in]  		-*pSPIx :type of @SPI_RegDef_t
  * @param[in]  		-pRxBuffer : string received data
  * @param[in]  		- Len:leng of string pRxbuffer
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer ,uint8_t Len)
{
	while(Len >0)
		{
			//wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)==FLAG_RESET);
			// check DFF bit of SPI_CR1 register
			if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
			{
				// 16bit DFF
				  *((uint16_t*)pRxBuffer)= pSPIx->DR;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8bit DFF
				*pRxBuffer = pSPIx->DR  ;
				Len--;
				pRxBuffer++;
			}
		}
}

void SPI_TransmitData_IT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer ,uint8_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxbuffer=pTxBuffer;
		pSPIHandle->Txlen = Len;

		 pSPIHandle->TxState = SPI_BUSY_IN_TX;

		 pSPIHandle->pSPIx->CR2 |=(1 << SPI_CR2_TXEIE);
	}
}
void SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle ,uint8_t *pRxBuffer ,uint8_t Len)
{
	uint8_t state = pSPIHandle->RxState;
		if(state != SPI_BUSY_IN_RX)
		{
			pSPIHandle->pRxbuffer=pRxBuffer;
			pSPIHandle->Rxlen = Len;

			 pSPIHandle->RxState = SPI_BUSY_IN_RX;

			 pSPIHandle->pSPIx->CR2 |=(1 << SPI_CR2_RXNEIE);
		}
}
/**************************************************************************************************************
  * @fn              	-SPI_SSI_Config
  * @brief  			-This function enables or disable SSI bit for NSS pin of SPI
  * @param[in]  		-*pSPIx :type of @SPI_RegDef_t
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_SSI_Config(SPI_RegDef_t *pSPIx ,uint8_t ENorDI)
{
	if(ENorDI ==ENABLE)
		{
			pSPIx->CR1 |= (1<< SPI_CR1_SSI);
		}else
		{
			pSPIx->CR1 &= ~(1<< SPI_CR1_SSI);
		}
}
/**************************************************************************************************************
  * @fn              	-SPI_PeripheralControl
  * @brief  			-This function enables or disable SPE bit
  * @param[in]  		-*pSPIx :type of @SPI_RegDef_t
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx ,uint8_t ENorDI )
{
	if(ENorDI ==ENABLE)
	{
		pSPIx->CR1 |= (1<< SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SPE);
	}
}
/*
 * IQR Configuration and IQR handling
 */
/**************************************************************************************************************
  * @fn              	-SPI_IQRInterrupt_Config
  * @brief  			-This function configure IQR SPI
  * @param[in]  		-IQRNumber: Number of GPIO IQR
  * @param[in] 			- ENorDI  : enable or disable (ENABLE ,DISABLE macros)
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_IQRInterrupt_Config(uint8_t IQRNumber,uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if(IQRNumber <= 31)
		{
			//program ISER0 register
			*NIVC_ISER0 |= (1 << IQRNumber);

		}else if(IQRNumber >31 && IQRNumber <64)
		{
			//program ISER1 register
			*NIVC_ISER1 |= (1<<IQRNumber);

		}else if(IQRNumber>=64  && IQRNumber < 96)
		{
			//program ISER2 register
			*NIVC_ISER2 |= (1<<IQRNumber);
		}
	}else
	{
		if(IQRNumber <= 31)
		{
			//program ICER0 register
			*NIVC_ICER0 |= (1<<IQRNumber);
		}else if(IQRNumber >31 && IQRNumber < 64)
		{
			//program ICER1 register
			*NIVC_ICER1 |= (1 << IQRNumber % 32);

		}else if(IQRNumber >= 64 && IQRNumber < 96)
		{
			//program ICER2 register
			*NIVC_ICER2 |=(1 << IQRNumber % 64);
		}
	}
}
/**************************************************************************************************************
  * @fn              	-SPI_IQRPriority_Config
  * @brief  			-This function configure the priority IQR
  * @param[in]  		-IQRNumber: Number of GPIO IQR
  * @param[in] 			- ENorDI  : enable or disable clock peripheral(ENABLE ,DISABLE macros)
  * @param[in]
  *
  * @return             -none
  *
  * @Note				-none
  *
  ************************************************************************************************************/
void SPI_IQRPriority_Config(uint8_t IQRNumber,uint8_t Priority )
{

	//1.first lets find out the priority register
    uint8_t iprx = IQRNumber/4;
    uint8_t iprx_sectin = IQRNumber % 4 ;
    uint8_t shift_amount = (8 *iprx_sectin) + (8-NO_PR_BITS_IMPLEMENTED);//4 bits lower not implemented
    *(NIVC_PR_BASEADDR+ iprx) |=(Priority << shift_amount );/* IPR0->IPR59,each IPRs have 4 register 8bit IQR,implemented 4bits high*/
}
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
void SPI_IQR_Handling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1,temp2;
	// first  lets check for TXE
	temp1= pSPIHandle->pSPIx->SR &(1<<SPI_SR_TXE);
	temp2= pSPIHandle->pSPIx->SR &(1<<SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		//handle TXE
		spi_tx_interrupt_handle(pSPIHandle);
	}

	// first  lets check for RXE
	temp1= pSPIHandle->pSPIx->SR &(1<<SPI_SR_RXNE);
	temp2= pSPIHandle->pSPIx->SR &(1<<SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//handle RXE
		spi_rx_interrupt_handle(pSPIHandle);
	}

	// first  lets check for OVR
	temp1= pSPIHandle->pSPIx->SR &(1<<SPI_SR_OVR);
	temp2= pSPIHandle->pSPIx->SR &(1<<SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		//handle TXE
		spi_over_err_interrupt_handle(pSPIHandle);
	}

}


/*
 * some helper function implementation
 */

static void  spi_rx_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check DFF bit of SPI_CR1 register
					if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
					{
						// 16bit DFF
						  *((uint16_t*)pSPIHandle->pRxbuffer)= pSPIHandle->pSPIx->DR;
						  pSPIHandle->Rxlen--;
						  pSPIHandle->Rxlen--;
						(uint16_t*)pSPIHandle->pRxbuffer++;
					}else
					{
						//8bit DFF
						*pSPIHandle->pRxbuffer = pSPIHandle->pSPIx->DR  ;
						pSPIHandle->Rxlen--;
						pSPIHandle->pRxbuffer++;
					}

					if( !pSPIHandle->Rxlen)
						{
							pSPIHandle->pSPIx->CR2  &= ~(1<< SPI_CR2_RXNEIE);
							pSPIHandle->pRxbuffer =NULL;
							pSPIHandle->Rxlen=0;
							pSPIHandle->RxState= SPI_READY;
							SPI_applicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);

						}

}
static void  spi_tx_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check DFF bit of SPI_CR1 register
					if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
					{
						// 16bit DFF
						  *((uint16_t*)pSPIHandle->pTxbuffer)= pSPIHandle->pSPIx->DR;
						  pSPIHandle->Txlen--;
						  pSPIHandle->Txlen--;
						(uint16_t*)pSPIHandle->pTxbuffer++;
					}else
					{
						//8bit DFF
						*pSPIHandle->pTxbuffer = pSPIHandle->pSPIx->DR  ;
						pSPIHandle->Txlen--;
						pSPIHandle->pTxbuffer++;
					}
					if( !pSPIHandle->Txlen)
									{
										pSPIHandle->pSPIx->CR2  &= ~(1<< SPI_CR2_RXNEIE);
										pSPIHandle->pTxbuffer =NULL;
										pSPIHandle->Txlen=0;
										pSPIHandle->TxState= SPI_READY;
										SPI_applicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);

									}

}

static void spi_over_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp ;
	//clear the ovr flag
	if(pSPIHandle->TxState !=SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//inform th application
	SPI_applicationEventCallBack(pSPIHandle,SPI_EVENT_OVERRUN);
}



__weak void SPI_applicationEventCallBack(SPI_Handle_t *pSPIHandle ,uint8_t AppEv)
{


}



