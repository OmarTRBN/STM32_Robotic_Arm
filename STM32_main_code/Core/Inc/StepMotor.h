/*
 * StepMotor.h
 *
 *  Created on: Mar 2, 2025
 *      Author: omart
 */

#ifndef INC_STEPMOTOR_H_
#define INC_STEPMOTOR_H_

#include "main.h"

#define MIN_LUT_SPEED 1
#define MAX_LUT_SPEED LUT_SIZE

#define STEP_MOTOR_CW GPIO_PIN_RESET
#define STEP_MOTOR_CCW GPIO_PIN_SET

typedef struct {
    TIM_HandleTypeDef* timer;
    uint32_t channel;
    GPIO_TypeDef* dir_gpio_port;
    uint16_t dir_gpio_pin;
    int16_t last_speed;
    int8_t dir_polarity;
} StepMotor;

HAL_StatusTypeDef StepMotor_Init(
		StepMotor* motor,
		TIM_HandleTypeDef* timer,
		uint32_t tim_channel,
		GPIO_TypeDef* dir_gpio_port,
		uint16_t dir_gpio_pin);
void StepMotor_SetSpeedLUT(StepMotor* motor, int16_t speed);
void StepMotor_Stop(StepMotor* motor);
void StepMotor_Start(StepMotor* motor);

#endif /* INC_STEPMOTOR_H_ */
