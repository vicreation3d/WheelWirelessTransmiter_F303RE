################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/nRF24/NRF24L01.c 

OBJS += \
./Core/lib/nRF24/NRF24L01.o 

C_DEPS += \
./Core/lib/nRF24/NRF24L01.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/nRF24/%.o Core/lib/nRF24/%.su Core/lib/nRF24/%.cyclo: ../Core/lib/nRF24/%.c Core/lib/nRF24/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I"F:/Workspace/Elektronika/Projects/WheelWirelessTransmiter_F303RE/Core/lib/AS5048A" -I"F:/Workspace/Elektronika/Projects/WheelWirelessTransmiter_F303RE/Core/lib/nRF24" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lib-2f-nRF24

clean-Core-2f-lib-2f-nRF24:
	-$(RM) ./Core/lib/nRF24/NRF24L01.cyclo ./Core/lib/nRF24/NRF24L01.d ./Core/lib/nRF24/NRF24L01.o ./Core/lib/nRF24/NRF24L01.su

.PHONY: clean-Core-2f-lib-2f-nRF24

