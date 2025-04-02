################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Controllers/PID_Control.c \
../Core/Controllers/Trajectory.c 

OBJS += \
./Core/Controllers/PID_Control.o \
./Core/Controllers/Trajectory.o 

C_DEPS += \
./Core/Controllers/PID_Control.d \
./Core/Controllers/Trajectory.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Controllers/%.o Core/Controllers/%.su Core/Controllers/%.cyclo: ../Core/Controllers/%.c Core/Controllers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/Controllers -I../Middlewares/ST/ARM/DSP/Inc -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Controllers

clean-Core-2f-Controllers:
	-$(RM) ./Core/Controllers/PID_Control.cyclo ./Core/Controllers/PID_Control.d ./Core/Controllers/PID_Control.o ./Core/Controllers/PID_Control.su ./Core/Controllers/Trajectory.cyclo ./Core/Controllers/Trajectory.d ./Core/Controllers/Trajectory.o ./Core/Controllers/Trajectory.su

.PHONY: clean-Core-2f-Controllers

