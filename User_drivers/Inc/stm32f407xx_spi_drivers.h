/*
 * stm32f407xx_spi_drivers.h
 *
 *  Created on: Jul 9, 2021
 *      Author: Hi
 */

#ifndef INC_STM32F407XX_SPI_DRIVERS_H_
#define INC_STM32F407XX_SPI_DRIVERS_H_


#include "stm32f407xx.h"



/*
 * define SPI structure
 */

typedef struct
{
	uint8_t SPI_DeviceMode;   			/*< select mode: Master or slave,possible values from @SPI_DeviceMode 				>*/
	uint8_t SPI_BusConfig;				/*< select bus configure ,possible values from @SPI_BusConfig 						>*/
	uint8_t SPI_Baud_Rate_Speed;		/*< select baud rate control ,possible values from @SPI_Baud_Rate_Speed 			>*/
	uint8_t SPI_DFF;					/*< select data frame format ,possible values from @SPI_DFF 						>*/
	uint8_t SPI_CPOL;					/*< clock polarity,possible values from @SPI_CPOL 									>*/
	uint8_t SPI_CPHA;					/*< clock phase,possible values from @SPI_CPHA 										>*/
	uint8_t SPI_SSM;					/*< software slave management,possible values from @SPI_SSM 						>*/

}SPI_Config_t;


typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *pTxbuffer;
	uint8_t *pRxbuffer;
	uint32_t Txlen;
	uint32_t Rxlen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_Handle_t;



/*
 * define macro SPI
 */

/*
 *@SPI_DeviceMode
 */
#define SPI_DEVICE_MASTER_MODE			1
#define SPI_DEVICE_SLAVE_MODE			0


/*
 *@SPI_BusConfig
 */
#define SPI_CONFIG_FULL_DUPLEX			1
#define SPI_CONFIG_HAFL_DUPLEX			2
#define SPI_CONFIG_SIMPLEX_ONLY_RX		3
/*
 *@SPI_Baud_Rate_Speed
 */
#define SPI_PCLK_DIV2			0
#define SPI_PCLK_DIV4			1
#define SPI_PCLK_DIV8			2
#define SPI_PCLK_DIV16			3
#define SPI_PCLK_DIV32			4
#define SPI_PCLK_DIV64			5
#define SPI_PCLK_DIV128			6
#define SPI_PCLK_DIV256			7

/*
 *@SPI_DFF
 */
#define SPI_DATA_FRAM_8BIT			0
#define SPI_DATA_FRAM_16BIT			1

/*
 *@SPI_CPOL
 */
#define SPI_CPOL_HIGH			1
#define SPI_CPOL_LOW			0

/*
 *@SPI_CPHA
 */
#define SPI_CPHA_HIGH			1
#define SPI_CPHA_LOW			0

/*
 * @SPI_SSM
 */
#define SPI_SMM_SW_EN			1
#define SPI_SMM_SW_DI			0

/*
 * define SPI flag status
 * @FlagName
 */
#define SPI_TXE_FLAG			(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG			(1<<SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG			(1<<SPI_SR_CHSIDE)
#define SPI_UDR_FLAG			(1<<SPI_SR_UDR)
#define SPI_CRCERR_FLAG			(1<<SPI_SR_CRCERR)
#define SPI_MODF_FLAG			(1<<SPI_SR_MODF)
#define SPI_OVR_FLAG			(1<<SPI_SR_OVR)
#define SPI_BSY_FLAG			(1<<SPI_SR_BSY)
#define SPI_FRE_FLAG			(1<<SPI_SR_FRE)

/*
 * SPI_State Rx Tx
 */

#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

/*
 * define application events
 */
#define SPI_EVENT_TX_CMPLT		1
#define SPI_EVENT_RX_CMPLT		2
#define SPI_EVENT_OVERRUN		3
#define SPI_EVENT_CRC_ERORR		4
/******************************************************************************************
 *                     APIs supported by this drivers
 *     For more information about APIs check  the function definitions
 *****************************************************************************************/


/*
 * Peripheral clock setup
 */
void SPI_PeriClkControl(SPI_RegDef_t *pSPIx ,uint8_t ENorDI );

/*
 * Init and de-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);
/*
 * Transmit and Received data
 */
void SPI_TransmitData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer ,uint8_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer ,uint8_t Len);

void SPI_TransmitData_IT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer ,uint8_t Len);
void SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle ,uint8_t *pRxBuffer ,uint8_t Len);

/*
 * SSI configure
 */
void SPI_SSI_Config(SPI_RegDef_t *pSPIx ,uint8_t ENorDI);

/*
 * other peripherals SPI control enable
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx ,uint8_t ENorDI );
void SPI_applicationEventCallBack(SPI_Handle_t *pSPIHandle ,uint8_t AppEv);
/*
 * IQR Configuration and IQR handling
 */
void SPI_IQRInterrupt_Config(uint8_t IQRnNumber,uint8_t ENorDI);
void SPI_IQRPriority_Config(uint8_t PinNumber,uint8_t Priority );
void SPI_IQR_Handling(SPI_Handle_t *pSPIx);
#endif /* INC_STM32F407XX_SPI_DRIVERS_H_ */
