################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User_drivers/Src/stm32f407xx_gpio_drivers.c \
../User_drivers/Src/stm32f407xx_spi_drivers.c 

OBJS += \
./User_drivers/Src/stm32f407xx_gpio_drivers.o \
./User_drivers/Src/stm32f407xx_spi_drivers.o 

C_DEPS += \
./User_drivers/Src/stm32f407xx_gpio_drivers.d \
./User_drivers/Src/stm32f407xx_spi_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
User_drivers/Src/stm32f407xx_gpio_drivers.o: ../User_drivers/Src/stm32f407xx_gpio_drivers.c User_drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"F:/stm32/STM32_dev_driver/User_drivers/Inc" -I"F:/stm32/STM32_dev_driver/User_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"User_drivers/Src/stm32f407xx_gpio_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
User_drivers/Src/stm32f407xx_spi_drivers.o: ../User_drivers/Src/stm32f407xx_spi_drivers.c User_drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"F:/stm32/STM32_dev_driver/User_drivers/Inc" -I"F:/stm32/STM32_dev_driver/User_drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"User_drivers/Src/stm32f407xx_spi_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

