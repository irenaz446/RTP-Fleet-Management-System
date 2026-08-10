################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Tools/Src/fleet_data.c \
../Tools/Src/io_tools.c 

OBJS += \
./Tools/Src/fleet_data.o \
./Tools/Src/io_tools.o 

C_DEPS += \
./Tools/Src/fleet_data.d \
./Tools/Src/io_tools.d 


# Each subdirectory must supply rules for building sources it contributes
Tools/Src/%.o Tools/Src/%.su Tools/Src/%.cyclo: ../Tools/Src/%.c Tools/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F756xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/iraze/STM32CubeIDE/workspace_1.19.0/Vehichle_fleet/Tools" -I"C:/Users/iraze/STM32CubeIDE/workspace_1.19.0/Vehichle_fleet/Tools/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Tools-2f-Src

clean-Tools-2f-Src:
	-$(RM) ./Tools/Src/fleet_data.cyclo ./Tools/Src/fleet_data.d ./Tools/Src/fleet_data.o ./Tools/Src/fleet_data.su ./Tools/Src/io_tools.cyclo ./Tools/Src/io_tools.d ./Tools/Src/io_tools.o ./Tools/Src/io_tools.su

.PHONY: clean-Tools-2f-Src

