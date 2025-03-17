/*
 * StepMotor.c
 *
 *  Created on: Mar 2, 2025
 *      Author: omart
 */

#include "StepMotor.h"
#include "lut.h"

HAL_StatusTypeDef StepMotor_Init(StepMotor* motor, TIM_HandleTypeDef* timer, uint32_t tim_channel, GPIO_TypeDef* gpio_port, uint16_t gpio_pin){
	motor->timer = timer;
	motor->channel = tim_channel;
	motor->dir_gpio_port = gpio_port;
	motor->dir_gpio_pin = gpio_pin;
	motor->last_speed = 0;

	HAL_GPIO_WritePin(gpio_port, gpio_pin, GPIO_PIN_RESET);
	return HAL_TIM_OC_Start(motor->timer, motor->channel);
}

void StepMotor_SetSpeedLUT(StepMotor* motor, int16_t speed) {
	if (speed == motor->last_speed)
	{
		return;
	}
	else if (speed == -motor->last_speed)
	{
		motor->last_speed = speed;
		HAL_GPIO_TogglePin(motor->dir_gpio_port, motor->dir_gpio_pin);
		return;
	}
	else if (speed == 0)
	{
		__HAL_TIM_SET_PRESCALER(motor->timer, 0xFFFF);
		__HAL_TIM_SET_AUTORELOAD(motor->timer, 0xFFFF);
//		motor->timer->Instance->CNT = 0;  // Optional: reset counter
		motor->timer->Instance->EGR |= TIM_EGR_UG;
		motor->last_speed = speed;
		return;
	}
	else
	{
		if (speed < 0)
		{
			HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, STEP_MOTOR_CW);
			motor->last_speed = speed;
			speed = -speed;
		}
		else
		{
			HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, STEP_MOTOR_CCW);
			motor->last_speed = speed;
		}

		if (speed > MAX_LUT_SPEED)
		{
			speed = MAX_LUT_SPEED;
		}
		else if (speed < MIN_LUT_SPEED)
		{
			speed = MIN_LUT_SPEED;
		}

		__HAL_TIM_SET_PRESCALER(motor->timer, LUT_PSC[speed - 1]);
		__HAL_TIM_SET_AUTORELOAD(motor->timer, LUT_ARR[speed - 1]);

//		motor->timer->Instance->CNT = 0;
		motor->timer->Instance->EGR |= TIM_EGR_UG;
		return;
	}
}

void StepMotor_Stop(StepMotor* motor) {
    HAL_TIM_OC_Stop(motor->timer, motor->channel);
    motor->timer->Instance->CNT = 0;
}

void StepMotor_Start(StepMotor* motor) {
    HAL_TIM_OC_Start(motor->timer, motor->channel);
}

