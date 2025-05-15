################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HW_Drivers/AS5600_Mux/AS5600_Mux.c 

OBJS += \
./Core/HW_Drivers/AS5600_Mux/AS5600_Mux.o 

C_DEPS += \
./Core/HW_Drivers/AS5600_Mux/AS5600_Mux.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HW_Drivers/AS5600_Mux/%.o Core/HW_Drivers/AS5600_Mux/%.su Core/HW_Drivers/AS5600_Mux/%.cyclo: ../Core/HW_Drivers/AS5600_Mux/%.c Core/HW_Drivers/AS5600_Mux/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Controllers -I../Middlewares/ST/ARM/DSP/Inc -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers" -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers/AS5600_Mux" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/App" -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers/StepMotor" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-HW_Drivers-2f-AS5600_Mux

clean-Core-2f-HW_Drivers-2f-AS5600_Mux:
	-$(RM) ./Core/HW_Drivers/AS5600_Mux/AS5600_Mux.cyclo ./Core/HW_Drivers/AS5600_Mux/AS5600_Mux.d ./Core/HW_Drivers/AS5600_Mux/AS5600_Mux.o ./Core/HW_Drivers/AS5600_Mux/AS5600_Mux.su

.PHONY: clean-Core-2f-HW_Drivers-2f-AS5600_Mux

