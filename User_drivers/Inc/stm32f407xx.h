/*
 * stm32f407xx.h
 *
 *  Created on: Jul 6, 2021
 *      Author: Hi
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 *  include libraries
 */
#include<stddef.h>
#include <stdint.h>
/*
 * define macro
 */
#define __VO   volatile
#define __weak  __attribute__((weak))
/**************************************START:Processor Specific Details*****************************************************************
 *
 ******************************ARM Cortex M4 processor  NIVC ISERx,ICERx register address
**************************************************************************************************************/
#define NIVC_ISER0 						((__VO uint32_t*)0xE000E100)
#define NIVC_ISER1 						((__VO uint32_t*)0xE000E104)
#define NIVC_ISER2 						((__VO uint32_t*)0xE000E108)
#define NIVC_ISER3 						((__VO uint32_t*)0xE000E10C)

#define NIVC_ICER0 						((__VO uint32_t*)0xE000E180)
#define NIVC_ICER1 						((__VO uint32_t*)0xE000E184)
#define NIVC_ICER2 						((__VO uint32_t*)0xE000E188)
#define NIVC_ICER3 						((__VO uint32_t*)0xE000E18C)

#define NIVC_PR_BASEADDR				((__VO uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			4
/*
 * define base address of FLASH,RAM and ROM memories
 */
#define FLASH_BASEADDR				0x08000000U   /*  memory mapping at table 3, page 71 of RM    */
#define SRAM1_BASEADDR				0x20000000U
#define SRAM2_BASEADDR              0x2001C000U
#define ROM_BASEADDR				0x1FFF0000U
#define SRAM						SRAM1_BASEADDR

/*
 * define  APBx , AHBx bus peripheral base address
 */

#define PERIPH_BASEADDR             	0x40000000U
#define APB1_PERIPH_BASEADDR            PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR            0x40010000U
#define AHB1_PERIPH_BASEADDR            0x40020000U
#define AHB2_PERIPH_BASEADDR            0x50000000


/*
 *  define base address of peripherals which are hanging of AHB1
 */
#define GPIOA_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x3800)
/*
 * define base address of peripherals which are hanging of APB1
 */
#define I2C1_BASEADDR                 	(APB1_PERIPH_BASEADDR+ 0x5400)
#define I2C2_BASEADDR                 	(APB1_PERIPH_BASEADDR+ 0x5800)
#define I2C3_BASEADDR                 	(APB1_PERIPH_BASEADDR+ 0x5C00)

#define USART2_BASEADDR                 (APB1_PERIPH_BASEADDR+ 0x4400)
#define USART3_BASEADDR                 (APB1_PERIPH_BASEADDR+ 0x4800)
#define UART4_BASEADDR               	(APB1_PERIPH_BASEADDR+ 0x4C00)
#define UART5_BASEADDR                 	(APB1_PERIPH_BASEADDR+ 0x5000)

#define SPI2_BASEADDR                 	(APB1_PERIPH_BASEADDR+ 0x3800)
#define SPI3_BASEADDR                 	(APB1_PERIPH_BASEADDR+ 0x3C00)


/*
 * define base address of peripherals which are hanging of APB2
 */

#define SPI1_BASEADDR                	(APB2_PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR                 (APB2_PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                 (APB2_PERIPH_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR                 (APB2_PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR                 	(APB2_PERIPH_BASEADDR + 0x3C00)
#define SPI4_BASEADDR                	(APB2_PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR                	(APB2_PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR                	(APB2_PERIPH_BASEADDR + 0x5400)


/************************** peripheral register definition  structures***************************************/
typedef struct
{
	__VO uint32_t MODER;    	 /*  	GPIO port mode Register 									address offset 0x00 	*/
	__VO uint32_t OTYPER;   	 /* 	GPIO port output type Register								address offset 0x04  	*/
	__VO uint32_t OSPEEDR;  	 /* 	GPIO port output speed register								address offset 0x08 	*/
	__VO uint32_t PUPDR;		 /* 	GPIO port pull-up down register								address offset 0x0C 	*/
	__VO uint32_t IDR;			 /* 	GPIO port input data Register 								address offset 0x10 	*/
	__VO uint32_t ODR;			 /*  	GPIO port output data Register 								address offset 0x04 	*/
	__VO uint32_t BSRR;			 /*  	GPIO port bit set/reset  Register 							address offset 0x18	 	*/
	__VO uint32_t LCKR;			 /*  	GPIO port configuration clock Register 						address offset 0x1C 	*/
	__VO uint32_t AFR[2];   	 /* 	AFR[0] GPIO alternate function low	register				address offset 0x20 	*/
								 /*     AFR[1] GPIO alternate function high register                                     	*/
}GPIO_RegDef_t;                  /*     find at GPIO Register map (table 39) page 288										*/

/**************************reset and control clock register definition  structures***************************************/


typedef struct
{
	__VO uint32_t CR;               	/*   clock control register                    						address offset 0x00   */
	__VO uint32_t PLLCFGR;				/*   PLL configuration register                    					address offset 0x04   */
	__VO uint32_t CFGR;             	/*   clock configuration register                  					address offset 0x08   */
	__VO uint32_t CIR;			    	/*   clock interrupt register                     					address offset 0x0C   */
	__VO uint32_t AHB1RSTR;         	/*   AHB1 peripheral reset register                   				address offset 0x10   */
	__VO uint32_t AHB2RSTR;				/*   AHB2 peripheral reset register                   				ddress offset 0x14   */
	__VO uint32_t AHB3RSTR;         	/*   AHB3 peripheral reset register                   				address offset 0x18   */
	uint32_t RESERVED0;
	__VO uint32_t APB1RSTR;				/*   APB1 peripheral reset register                    				address offset 0x20   */
	__VO uint32_t APB2RSTR;         	/*   APB2 peripheral reset register                    				address offset 0x24   */
    uint32_t RESERVED1[2];
	__VO uint32_t AHB1ENR;				/*   AHB1 peripheral clock enable register                    		address offset 0x30   */
	__VO uint32_t AHB2ENR;          	/*   AHB2 peripheral clock enable register                  		address offset 0x34   */
	__VO uint32_t AHB3ENR;				/*   AHB3 peripheral clock enable register                 			address offset 0x38   */
	uint32_t RESERVED2;
	__VO uint32_t APB1ENR;          	/*   APB1 peripheral clock enable register                    		address offset 0x40   */
	__VO uint32_t APB2ENR;				/*   APB2 peripheral clock enable register                			address offset 0x44   */
	uint32_t RESERVED3[2];
	__VO uint32_t AHB1LPENR;            /*   AHB1 peripheral clock enable in low power mode register        address offset 0x50   */
	__VO uint32_t AHB2LPENR;			/*   AHB2 peripheral clock enable in low power mode register        address offset 0x54   */
	__VO uint32_t AHB3LPENR;			/*   AHB3 peripheral clock enable in low power mode register        address offset 0x58   */
	uint32_t RESERVED4;
	__VO uint32_t APB1LPENR;            /*   APB2 peripheral clock enable in low power mode register        address offset 0x60   */
	__VO uint32_t APB2LPENR;			/*   APB2 peripheral clock enable in low power mode register        address offset 0x64   */
	uint32_t RESERVED5[2];
	__VO uint32_t BDCR;                 /*   Backup domain control register                    	 			address offset 0x70   */
	__VO uint32_t CSR;					/*   Control clock & status register                    			address offset 0x74   */
	uint32_t RESERVED6[2];
	__VO uint32_t SSCGR;                /*   Spread spectrum clock generation register                 		address offset 0x80   */
	__VO uint32_t PLLI2SCFGR;			/*   PLLI2S configuration register                    	 			address offset 0x84   */

}RCC_RegDef_t;

/*
 * external interrupt structure
 */
typedef struct
{
	__VO uint32_t IRM;					/*         Interrupt mask register                    	 	 	address offset 0x00   */
	__VO uint32_t ERM;					/*         Event mask register                   	 	 		address offset 0x04   */
	__VO uint32_t RTSR;					/*         Rising trigger selection register                    address offset 0x08   */
	__VO uint32_t FTSR;					/*         Falling trigger selection register                   address offset 0x0C   */
	__VO uint32_t SWIER;				/*         Software interrupt event register                    address offset 0x10   */
	__VO uint32_t PR;					/*         Pending register                    	 	            address offset 0x14   */
}EXTI_RegDef_t;

typedef struct
{
	__VO uint32_t CR1;					/*         SPI control register 1                	 	 	address offset 0x00   */
	__VO uint32_t CR2;					/*         SPI control register 2                    	 	address offset 0x04   */
	__VO uint32_t SR;					/*         SPI status register                    	 	 	address offset 0x08   */
	__VO uint32_t DR;					/*         SPI data register                    	 	 	address offset 0x0C   */
	__VO uint32_t CRCPR;				/*         SPI CRC polynomial register                    	address offset 0x10   */
	__VO uint32_t RXCRCR;				/*         SPI RX CRC register                    	 	 	address offset 0x14   */
	__VO uint32_t TXCRCR;				/*         SPI TX CRC register                    	 	 	address offset 0x18   */
	__VO uint32_t I2SCFCR;				/*         I2S configuration register                    	address offset 0x1C   */
	__VO uint32_t I2SPR;				/*         I2S prescaler register                    	 	address offset 0x20   */
}SPI_RegDef_t;

/*
 * System configure structure
 */
typedef struct
{
	__VO uint32_t MEMRMP;					/*         Memory map register                    	 	 			     address offset 0x00   */
	__VO uint32_t PMC;					    /*         Peripheral mode configuration register                    	 address offset 0x04   */
	__VO uint32_t EXTICR[4];				/*         External interrupt configuration register 1,2,3,4                   address offset 0x08->0x14   */
	uint32_t Reserved;
	__VO uint32_t CMPCR;					/*         Compensation cell control register                    	 	 address offset 0x20   */

}SYSCFG_RegDef_t;

/*
 * peripheral definition
 */
#define GPIOA                  ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB                  ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC                  ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD                  ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE                  ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF                  ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG                  ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH                  ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI                  ((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC   				   ((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI				   ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				   ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5					((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6					((SPI_RegDef_t*)SPI6_BASEADDR)
/*
 * Clock enable macro for GPIOx peripherals
 */
#define  GPIOA_PCLK_EN()			( RCC->AHB1ENR |=(1<<0) )
#define  GPIOB_PCLK_EN()			( RCC->AHB1ENR |=(1<<1) )
#define  GPIOC_PCLK_EN()			( RCC->AHB1ENR |=(1<<2) )
#define  GPIOD_PCLK_EN()			( RCC->AHB1ENR |=(1<<3) )
#define  GPIOE_PCLK_EN()			( RCC->AHB1ENR |=(1<<4) )
#define  GPIOF_PCLK_EN()			( RCC->AHB1ENR |=(1<<5) )
#define  GPIOH_PCLK_EN()			( RCC->AHB1ENR |=(1<<6) )
#define  GPIOG_PCLK_EN()			( RCC->AHB1ENR |=(1<<7) )
#define  GPIOI_PCLK_EN()			( RCC->AHB1ENR |=(1<<8) )

/*
 * Clock enable macro for I2Cx peripherals
 */
#define  I2C1_PCLK_EN()			( RCC->APB1ENR |=(1<<21) )
#define  I2C2_PCLK_EN()			( RCC->APB1ENR |=(1<<22) )
#define  I2C3_PCLK_EN()			( RCC->APB1ENR |=(1<<23) )
/*
 * Clock enable macro for SPIx peripherals
 */


#define  SPI1_PCLK_EN()			( RCC->APB2ENR |=(1<<12) )
#define  SPI2_PCLK_EN()			( RCC->APB1ENR |=(1<<14) )
#define  SPI3_PCLK_EN()			( RCC->APB1ENR |=(1<<15) )
#define  SPI4_PCLK_EN()			( RCC->APB2ENR |=(1<<13) )
#define  SPI5_PCLK_EN()			( RCC->APB1ENR |=(1<<20) )
#define  SPI6_PCLK_EN()			( RCC->APB1ENR |=(1<<21) )




/*
 * Clock enable macro for USARTx peripherals
 */

#define  USART2_PCLK_EN()			( RCC->APB1ENR |=(1<<17) )
#define  USART3_PCLK_EN()			( RCC->APB1ENR |=(1<<18) )
#define  UART4_PCLK_EN()			( RCC->APB1ENR |=(1<<19) )
#define  UART5_PCLK_EN()			( RCC->APB1ENR |=(1<<20) )
#define  USART1_PCLK_EN()			( RCC->APB2ENR |=(1<<4) )
#define  USART6_PCLK_EN()			( RCC->APB2ENR |=(1<<5) )

/*
 * Clock enable macro for system configuration
 */

#define  SYSCFG_CLK_EN()			( RCC->APB2ENR |=(1<<14) )
/*********************************************************************/
/*
 * Clock disable macro for GPIOx peripherals
 */
#define  GPIOA_PCLK_DI()			( RCC->AHB1ENR |=(1<<0) )
#define  GPIOB_PCLK_DI()			( RCC->AHB1ENR |=(1<<1) )
#define  GPIOC_PCLK_DI()			( RCC->AHB1ENR |=(1<<2) )
#define  GPIOD_PCLK_DI()			( RCC->AHB1ENR |=(1<<3) )
#define  GPIOE_PCLK_DI()			( RCC->AHB1ENR |=(1<<4) )
#define  GPIOF_PCLK_DI()			( RCC->AHB1ENR |=(1<<5) )
#define  GPIOH_PCLK_DI()			( RCC->AHB1ENR |=(1<<6) )
#define  GPIOG_PCLK_DI()			( RCC->AHB1ENR |=(1<<7) )
#define  GPIOI_PCLK_DI()			( RCC->AHB1ENR |=(1<<8) )
/*
 * Clock disable macro for I2Cx peripherals
 */
#define  I2C1_PCLK_DI()			( RCC->APB1ENR &=~(1<<21) )
#define  I2C2_PCLK_DI()			( RCC->APB1ENR &=~(1<<22) )
#define  I2C3_PCLK_DI()			( RCC->APB1ENR &=~(1<<23) )
/*
 * Clock disable macro for SPIx peripherals
 */
#define  SPI1_PCLK_DI()			( RCC->APB2ENR &=~(1<<12) )
#define  SPI2_PCLK_DI()			( RCC->APB1ENR &=~(1<<14) )
#define  SPI3_PCLK_DI()			( RCC->APB1ENR &=~(1<<15) )
#define  SPI4_PCLK_DI()			( RCC->APB2ENR &=~(1<<13) )
#define  SPI5_PCLK_DI()			( RCC->APB1ENR &=~(1<<20) )
#define  SPI6_PCLK_DI()			( RCC->APB1ENR &=~(1<<21) )

/*
 * Clock disable macro for USARTx peripherals
 */

#define  USART2_PCLK_DI()			( RCC->APB1ENR &=~(1<<17) )
#define  USART3_PCLK_DI()			( RCC->APB1ENR &=~(1<<18) )
#define  UART4_PCLK_DI()			( RCC->APB1ENR &=~(1<<19) )
#define  UART5_PCLK_DI()			( RCC->APB1ENR &=~(1<<20) )
#define  USART1_PCLK_DI()			( RCC->APB2ENR &=~(1<<4) )
#define  USART6_PCLK_DI()			( RCC->APB2ENR &=~(1<<5) )

/*
 * Clock disable macro for system configuration
 */
#define  SYS_CONF_CLK_DI()			( RCC->APB2ENR &=~(1<<14) )


/*
 * macros reset GPIOx peripheral
 */

#define GPIOA_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOF_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &= ~(1<<5)); }while(0)
#define GPIOG_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &= ~(1<<6)); }while(0)
#define GPIOH_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)
#define GPIOI_REG_RESET()						do{(RCC->AHB1RSTR |=(1<<8)); (RCC->AHB1RSTR &= ~(1<<8)); }while(0)

/*
 * macros returns a code  between (0->7 ) for a given GPIO base address
 */
#define GPIO_BASEADDR_TO_CODE(x)				   ((x== GPIOA)? 0 :\
													(x== GPIOB)? 1 :\
													(x== GPIOC)? 2 :\
													(x== GPIOD)? 3 :\
													(x== GPIOE)? 4 :\
													(x== GPIOF)? 5 :\
                                                    (x== GPIOG)? 6 :\
                                                    (x== GPIOH)? 7 :\
													(x== GPIOI)? 8 :0)

/*
 * IQR(Interrupt request ) number of STM32F4 MCU
 */
#define IQR_EXTI_NO0			6
#define IQR_EXTI_NO1			7
#define IQR_EXTI_NO2			8
#define IQR_EXTI_NO3			9
#define IQR_EXTI_NO4			10
#define IQR_EXTI_NO9_5			23
#define IQR_EXTI_NO15_10		40

/*   some generic macros      */

#define ENABLE 				1
#define DISABLE 			0
#define SET 		    	1
#define RESET    			0
#define GPIO_PIN_SET    	1
#define GPIO_PIN_RESET    	0
#define FLAG_SET			SET
#define FLAG_RESET			RESET


/*****************************************************************************************
 * bit position definition of SPI peripheral
 *****************************************************************************************/
/*
 * bit position CR1 register
 */
#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_LSBFIRST					7
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_CRCNEXT						12
#define SPI_CR1_CRCEN						13
#define SPI_CR1_BIDIOE						14
#define SPI_CR1_BIDIMODE					15


/*
 * bit position CR2 register
 */
#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXEIE						7


/*
 * bit position SR register
 */
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRCERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8

/*
 * macros reset SPI peripheral
 */

#define SPI1_REG_RESET()						do{(RCC->APB2RSTR |=(1<<12)); (RCC->APB2RSTR &= ~(1<<12)); }while(0)
#define SPI2_REG_RESET()						do{(RCC->APB1RSTR |=(1<<14)); (RCC->APB1RSTR &= ~(1<<14)); }while(0)
#define SPI3_REG_RESET()						do{(RCC->APB1RSTR |=(1<<15)); (RCC->APB1RSTR &= ~(1<<15)); }while(0)
#define SPI4_REG_RESET()						do{(RCC->APB2RSTR |=(1<<13)); (RCC->APB2RSTR &= ~(1<<13)); }while(0)
#define SPI5_REG_RESET()						do{(RCC->APB2RSTR |=(1<<20)); (RCC->APB2RSTR &= ~(1<<20)); }while(0)
#define SPI6_REG_RESET()						do{(RCC->APB2RSTR |=(1<<21)); (RCC->APB2RSTR &= ~(1<<21)); }while(0)



#include "stm32f407xx_gpio_drivers.h"
#include "stm32f407xx_spi_drivers.h"




#endif /* INC_STM32F407XX_H_ */
