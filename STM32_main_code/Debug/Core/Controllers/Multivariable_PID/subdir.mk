################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Controllers/Multivariable_PID/PID_Control.c 

OBJS += \
./Core/Controllers/Multivariable_PID/PID_Control.o 

C_DEPS += \
./Core/Controllers/Multivariable_PID/PID_Control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Controllers/Multivariable_PID/%.o Core/Controllers/Multivariable_PID/%.su Core/Controllers/Multivariable_PID/%.cyclo: ../Core/Controllers/Multivariable_PID/%.c Core/Controllers/Multivariable_PID/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Controllers -I../Middlewares/ST/ARM/DSP/Inc -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers" -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers/AS5600_Mux" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/App" -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers/StepMotor" -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers/DWT_Timing" -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/Controllers/Multivariable_PID" -I"C:/Users/omart/Desktop/Projects/STM32_Robotic_Arm/STM32_main_code/Core/HW_Drivers/SerialComm" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Controllers-2f-Multivariable_PID

clean-Core-2f-Controllers-2f-Multivariable_PID:
	-$(RM) ./Core/Controllers/Multivariable_PID/PID_Control.cyclo ./Core/Controllers/Multivariable_PID/PID_Control.d ./Core/Controllers/Multivariable_PID/PID_Control.o ./Core/Controllers/Multivariable_PID/PID_Control.su

.PHONY: clean-Core-2f-Controllers-2f-Multivariable_PID

