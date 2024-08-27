################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Invn/VSensor/VSensor.c 

OBJS += \
./Drivers/Invn/VSensor/VSensor.o 

C_DEPS += \
./Drivers/Invn/VSensor/VSensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Invn/VSensor/%.o Drivers/Invn/VSensor/%.su Drivers/Invn/VSensor/%.cyclo: ../Drivers/Invn/VSensor/%.c Drivers/Invn/VSensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers -I../Drivers/Invn -I../Drivers/Invn/Devices -I../Drivers/Invn/Devices/Drivers/Icm20948 -I../Drivers/Invn/DynamicProtocol -I../Drivers/Invn/EmbUtils -I../Drivers/Invn/Images -I../Drivers/Invn/VSensor -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@"  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Invn-2f-VSensor

clean-Drivers-2f-Invn-2f-VSensor:
	-$(RM) ./Drivers/Invn/VSensor/VSensor.cyclo ./Drivers/Invn/VSensor/VSensor.d ./Drivers/Invn/VSensor/VSensor.o ./Drivers/Invn/VSensor/VSensor.su

.PHONY: clean-Drivers-2f-Invn-2f-VSensor

