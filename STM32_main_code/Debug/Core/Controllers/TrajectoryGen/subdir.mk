################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Controllers/TrajectoryGen/Trajectory.c 

OBJS += \
./Core/Controllers/TrajectoryGen/Trajectory.o 

C_DEPS += \
./Core/Controllers/TrajectoryGen/Trajectory.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Controllers/TrajectoryGen/%.o Core/Controllers/TrajectoryGen/%.su Core/Controllers/TrajectoryGen/%.cyclo: ../Core/Controllers/TrajectoryGen/%.c Core/Controllers/TrajectoryGen/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Controllers -I../Middlewares/ST/ARM/DSP/Inc -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/AS5600_Mux" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/StepMotor" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/DWT_Timing" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Controllers/Multivariable_PID" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/HW_Drivers/SerialComm" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Controllers/TrajectoryGen" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Application" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Application/App" -I"/Users/omarmac/Desktop/Projects/stm32_robot/STM32_main_code/Core/Application/ParserFunctions" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Controllers-2f-TrajectoryGen

clean-Core-2f-Controllers-2f-TrajectoryGen:
	-$(RM) ./Core/Controllers/TrajectoryGen/Trajectory.cyclo ./Core/Controllers/TrajectoryGen/Trajectory.d ./Core/Controllers/TrajectoryGen/Trajectory.o ./Core/Controllers/TrajectoryGen/Trajectory.su

.PHONY: clean-Core-2f-Controllers-2f-TrajectoryGen

