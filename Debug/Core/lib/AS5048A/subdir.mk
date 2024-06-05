################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/AS5048A/AS5048A.c 

OBJS += \
./Core/lib/AS5048A/AS5048A.o 

C_DEPS += \
./Core/lib/AS5048A/AS5048A.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/AS5048A/%.o Core/lib/AS5048A/%.su Core/lib/AS5048A/%.cyclo: ../Core/lib/AS5048A/%.c Core/lib/AS5048A/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I"F:/Workspace/Elektronika/Projects/WheelWirelessTransmiter_F303RE/Core/lib/AS5048A" -I"F:/Workspace/Elektronika/Projects/WheelWirelessTransmiter_F303RE/Core/lib/nRF24" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lib-2f-AS5048A

clean-Core-2f-lib-2f-AS5048A:
	-$(RM) ./Core/lib/AS5048A/AS5048A.cyclo ./Core/lib/AS5048A/AS5048A.d ./Core/lib/AS5048A/AS5048A.o ./Core/lib/AS5048A/AS5048A.su

.PHONY: clean-Core-2f-lib-2f-AS5048A

