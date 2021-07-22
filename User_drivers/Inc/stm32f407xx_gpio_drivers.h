/*
 * stm32f407xx_gpio_drivers.h
 *
 *  Created on: Jul 7, 2021
 *      Author: Hi
 */

#ifndef INC_STM32F407XX_GPIO_DRIVERS_H_
#define INC_STM32F407XX_GPIO_DRIVERS_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/* <possible values from @GPIO_PIN_NUMBER> */
	uint8_t GPIO_PinMode; 			/* <possible values from @GPIO_PIN_MODE> */
	uint8_t GPIO_PinSpeed;			/* <possible values from @GPIO_OUTPUT_SPEED> */
	uint8_t GPIO_PinPuPdControl;	/* <possible values from @GPIO_PU_PD> */
	uint8_t GPIO_PinOPType;			/* <possible values from @GPIO_OUTPUT_TYPE> */
	uint8_t GPIO_PinAltFunMode;
}GPIO_Config_t;

/*
 * This is a handle structure for a GPIO pin
 */
typedef struct
{
	/*  pointer the hold the base address of the GPIO peripheral */
	GPIO_RegDef_t *pGPIO;
	GPIO_Config_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBER
 */
#define GPIO_PIN_NO					0
#define GPIO_PIN_N1					1
#define GPIO_PIN_N2					2
#define GPIO_PIN_N3					3
#define GPIO_PIN_N4					4
#define GPIO_PIN_N5					5
#define GPIO_PIN_N6					6
#define GPIO_PIN_N7					7
#define GPIO_PIN_N8					8
#define GPIO_PIN_N9					9
#define GPIO_PIN_N10				10
#define GPIO_PIN_N11				11
#define GPIO_PIN_N12				12
#define GPIO_PIN_N13				13
#define GPIO_PIN_N14				14
#define GPIO_PIN_N15				15


/*
 * @GPIO_PIN_MODE
 * GPIO pin possible mode
 */
#define   	GPIO_INPUT_MODE			0
#define 	GPIO_OUTPUT_MODE		1
#define		GPIO_ALT_FUNC_MODE		2
#define		GPIO_ANALOG_MODE		3
#define		GPIO_IT_FT_MODE			4  /*<GPIO mode interrupt falling edge trigger>*/
#define		GPIO_IT_RT_MODE			5  /*<GPIO mode interrupt rising edge trigger>*/
#define		GPIO_IT_FRT_MODE		6  /*<GPIO mode interrupt falling,rising edge trigger>*/

/*
 * @GPIO_OUTPUT_TYPE
 * GPIO port output type
 */
#define		GPIO_OPTYPE_PP			0   /*<GPIO output type push-pull>*/
#define		GPIO_OPTYPE_OD			1	/*<GPIO output type open drain>*/

/*
 * @GPIO_OUTPUT_SPEED
 * GPIO output speed
 */
#define		GPIO_SPEED_LOW			0
#define		GPIO_SPEED_MEDIUM		1
#define		GPIO_SPEED_HIGH			2
#define		GPIO_SPEED_VERY_HIGH	3

/*
 * @GPIO_PU_PD
 * GPIO port pull up/pull-down
 */
#define		GPIO_NO_PUPD			0
#define		GPIO_PIN_PULL_UP		1
#define		GPIO_PIN_PULL_DOWN		2

/******************************************************************************************
 *                     APIs supported by this drivers
 *     For more information about APIs check  the function definitions
 *****************************************************************************************/


/*
 * Peripheral clock setup
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx ,uint8_t ENorDI );

/*
 * Init and de-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write functions
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 * IQR Configuration and IQR handling
 */
void GPIO_IQRInterrupt_Config(uint8_t IQRnNumber,uint8_t ENorDI);
void GPIO_IQRPriority_Config(uint8_t PinNumber,uint8_t Priority );
void GPIO_IQR_Handling(uint8_t PinNumber);



#endif /* INC_STM32F407XX_GPIO_DRIVERS_H_ */
