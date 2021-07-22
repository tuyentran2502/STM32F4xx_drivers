/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"


/*
 * PB14 SPI2_MISO
 * PB15 SPI2_MOSI
 * PB13 SPI2_SCLK
 * PB12 SPI2_NSS
 */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIO = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_ALT_FUNC_MODE;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType =GPIO_OPTYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed =GPIO_SPEED_VERY_HIGH;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_N13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_N15;
	GPIO_Init(&SPIPins);
	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_N14;
	GPIO_Init(&SPIPins);
	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_N12;
	GPIO_Init(&SPIPins);

}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MASTER_MODE;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_CONFIG_FULL_DUPLEX;
	SPI2handle.SPI_Config.SPI_Baud_Rate_Speed =SPI_PCLK_DIV2;
	SPI2handle.SPI_Config.SPI_DFF =SPI_DATA_FRAM_8BIT;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM  = SPI_SMM_SW_EN;//software slave enable for NSS

	SPI_Init(&SPI2handle);
}
int main(void)
{

	char user_data[]="Hello world";

	SPI2_GPIOInits();
	SPI2_Inits();
	SPI_SSI_Config(SPI2,ENABLE);
	SPI_PeripheralControl(SPI2,ENABLE);
	SPI_TransmitData(SPI2, (uint8_t*)user_data, strlen(user_data));
	while(SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG));//confirm SPI is not busy
	SPI_PeripheralControl(SPI2,DISABLE);// disable SPI2 peripheral
	while(1)
	{

	}
}





















