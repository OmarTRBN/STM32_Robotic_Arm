################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HW_Drivers/DWT_Timing/Timing.c 

OBJS += \
./Core/HW_Drivers/DWT_Timing/Timing.o 

C_DEPS += \
./Core/HW_Drivers/DWT_Timing/Timing.d 


# Each subdirectory must supply rules for building sources it contributes
Core/HW_Drivers/DWT_Timing/%.o Core/HW_Drivers/DWT_Timing/%.su Core/HW_Drivers/DWT_Timing/%.cyclo: ../Core/HW_Drivers/DWT_Timing/%.c Core/HW_Drivers/DWT_Timing/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Controllers -I../Middlewares/ST/ARM/DSP/Inc -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/AS5600_Mux" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/App" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/StepMotor" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/DWT_Timing" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Controllers/Multivariable_PID" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/SerialComm" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-HW_Drivers-2f-DWT_Timing

clean-Core-2f-HW_Drivers-2f-DWT_Timing:
	-$(RM) ./Core/HW_Drivers/DWT_Timing/Timing.cyclo ./Core/HW_Drivers/DWT_Timing/Timing.d ./Core/HW_Drivers/DWT_Timing/Timing.o ./Core/HW_Drivers/DWT_Timing/Timing.su

.PHONY: clean-Core-2f-HW_Drivers-2f-DWT_Timing

