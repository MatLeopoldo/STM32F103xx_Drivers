################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/hal_gpio.c \
../Drivers/Src/hal_i2c.c \
../Drivers/Src/hal_nvic.c \
../Drivers/Src/hal_rcc.c \
../Drivers/Src/hal_spi.c \
../Drivers/Src/hal_usart.c 

OBJS += \
./Drivers/Src/hal_gpio.o \
./Drivers/Src/hal_i2c.o \
./Drivers/Src/hal_nvic.o \
./Drivers/Src/hal_rcc.o \
./Drivers/Src/hal_spi.o \
./Drivers/Src/hal_usart.o 

C_DEPS += \
./Drivers/Src/hal_gpio.d \
./Drivers/Src/hal_i2c.d \
./Drivers/Src/hal_nvic.d \
./Drivers/Src/hal_rcc.d \
./Drivers/Src/hal_spi.d \
./Drivers/Src/hal_usart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"/home/matheus/Dados/Estudos/Sistemas_Embarcados/Projetos/STM32F103xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/hal_gpio.d ./Drivers/Src/hal_gpio.o ./Drivers/Src/hal_i2c.d ./Drivers/Src/hal_i2c.o ./Drivers/Src/hal_nvic.d ./Drivers/Src/hal_nvic.o ./Drivers/Src/hal_rcc.d ./Drivers/Src/hal_rcc.o ./Drivers/Src/hal_spi.d ./Drivers/Src/hal_spi.o ./Drivers/Src/hal_usart.d ./Drivers/Src/hal_usart.o

.PHONY: clean-Drivers-2f-Src

