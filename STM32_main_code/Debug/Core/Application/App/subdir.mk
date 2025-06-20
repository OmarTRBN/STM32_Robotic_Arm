################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Application/App/app.c \
../Core/Application/App/main_init.c 

OBJS += \
./Core/Application/App/app.o \
./Core/Application/App/main_init.o 

C_DEPS += \
./Core/Application/App/app.d \
./Core/Application/App/main_init.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Application/App/%.o Core/Application/App/%.su Core/Application/App/%.cyclo: ../Core/Application/App/%.c Core/Application/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Controllers -I../Middlewares/ST/ARM/DSP/Inc -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/AS5600_Mux" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/StepMotor" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/DWT_Timing" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Controllers/Multivariable_PID" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/SerialComm" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Controllers/TrajectoryGen" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Application" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Application/App" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Application/ParserFunctions" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Application-2f-App

clean-Core-2f-Application-2f-App:
	-$(RM) ./Core/Application/App/app.cyclo ./Core/Application/App/app.d ./Core/Application/App/app.o ./Core/Application/App/app.su ./Core/Application/App/main_init.cyclo ./Core/Application/App/main_init.d ./Core/Application/App/main_init.o ./Core/Application/App/main_init.su

.PHONY: clean-Core-2f-Application-2f-App

