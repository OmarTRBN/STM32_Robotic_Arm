/*
 * StepMotor.c
 *
 *  Created on: Mar 2, 2025
 *      Author: omart
 */

#include "StepMotor.h"

static const STEPMOTOR_HWConfig motor_hw_table[STEPMOTOR_MAX_CHANNELS] = {
    { &htim5, TIM_CHANNEL_1, M1_DIR_GPIO_Port, M1_DIR_Pin, M1_EN_GPIO_Port, M1_EN_Pin }, // Motor 1
    { &htim9, TIM_CHANNEL_1, M2_DIR_GPIO_Port, M2_DIR_Pin, M2_EN_GPIO_Port, M2_EN_Pin }, // Motor 2
	{ &htim3, TIM_CHANNEL_1, M3_DIR_GPIO_Port, M3_DIR_Pin, M3_EN_GPIO_Port, M3_EN_Pin },
	{ &htim1, TIM_CHANNEL_4, M4_DIR_GPIO_Port, M4_DIR_Pin, M4_EN_GPIO_Port, M4_EN_Pin }
};

static inline void STEPMOTOR_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
    HAL_GPIO_WritePin(port, pin, state);
}

static inline void STEPMOTOR_TogglePin(GPIO_TypeDef* port, uint16_t pin) {
    HAL_GPIO_TogglePin(port, pin);
}

static inline void STEPMOTOR_SetPSC(TIM_HandleTypeDef* timer, uint16_t psc) {
    __HAL_TIM_SET_PRESCALER(timer, psc);
}

static inline void STEPMOTOR_SetARR(TIM_HandleTypeDef* timer, uint16_t arr) {
    __HAL_TIM_SET_AUTORELOAD(timer, arr);
}

STEPMOTOR_StatusTypeDef STEPMOTOR_Init(STEPMOTOR_HandleTypeDef* motor, uint8_t index) {
    if (!motor || index >= STEPMOTOR_MAX_CHANNELS) return STEPMOTOR_ERROR;

    motor->hw = motor_hw_table[index];
    motor->last_speed = 0;

    STEPMOTOR_WritePin(motor->hw.dir_gpio_port, motor->hw.dir_gpio_pin, GPIO_PIN_RESET);
    STEPMOTOR_WritePin(motor->hw.en_gpio_port, motor->hw.en_gpio_pin, GPIO_PIN_RESET);

    return (HAL_TIM_OC_Start(motor->hw.timer, motor->hw.channel) == HAL_OK) ? STEPMOTOR_OK : STEPMOTOR_ERROR;
}

void STEPMOTOR_SetSpeedLUT(STEPMOTOR_HandleTypeDef* motor, int16_t speed) {
    if (!motor) return;

    if (speed == motor->last_speed)
    {

    }
    else if (speed == -motor->last_speed)
    {
        motor->last_speed = speed;
        STEPMOTOR_TogglePin(motor->hw.dir_gpio_port, motor->hw.dir_gpio_pin);
        return;
    }
    else if (speed == 0)
    {
        STEPMOTOR_SetPSC(motor->hw.timer, 0xFFFF);
        STEPMOTOR_SetARR(motor->hw.timer, 0xFFFF);
        motor->hw.timer->Instance->EGR |= TIM_EGR_UG;
        motor->last_speed = 0;
        return;
    }
    else
    {
        if (speed < 0)
        {
            STEPMOTOR_WritePin(motor->hw.dir_gpio_port, motor->hw.dir_gpio_pin, STEPMOTOR_CW);
            motor->last_speed = speed;
            speed = -speed;
        }
        else
        {
            STEPMOTOR_WritePin(motor->hw.dir_gpio_port, motor->hw.dir_gpio_pin, STEPMOTOR_CCW);
            motor->last_speed = speed;
        }

        if (speed > STEPMOTOR_MAX_LUT_SPEED)
        {
        	speed = STEPMOTOR_MAX_LUT_SPEED;
        }
        else if (speed < STEPMOTOR_MIN_LUT_SPEED)
        {
        	speed = STEPMOTOR_MIN_LUT_SPEED;
        }

        STEPMOTOR_SetPSC(motor->hw.timer, LUT_PSC[speed - 1]);
        STEPMOTOR_SetARR(motor->hw.timer, LUT_ARR[speed - 1]);
        motor->hw.timer->Instance->EGR |= TIM_EGR_UG;
    }
    return;
}

void STEPMOTOR_EnableControl(STEPMOTOR_HandleTypeDef* motor, bool enable) {
    if (!motor) return;

    if (enable) {
        STEPMOTOR_WritePin(motor->hw.en_gpio_port, motor->hw.en_gpio_pin, GPIO_PIN_RESET);
        HAL_TIM_OC_Start(motor->hw.timer, motor->hw.channel);
    } else {
        STEPMOTOR_WritePin(motor->hw.en_gpio_port, motor->hw.en_gpio_pin, GPIO_PIN_SET);
        HAL_TIM_OC_Stop(motor->hw.timer, motor->hw.channel);
        motor->hw.timer->Instance->CNT = 0;
    }
}

void STEPMOTOR_MultiSetSpeed(STEPMOTOR_HandleTypeDef* motorArray, float speedArray[], uint8_t motorCount) {
    for (int i = 0; i < motorCount; i++) {
        STEPMOTOR_SetSpeedLUT(&motorArray[i], speedArray[i]);
    }
}
void STEPMOTOR_MultiEnableControl(STEPMOTOR_HandleTypeDef* motorArray, uint8_t motorCount, bool enable) {
    for (int i = 0; i < motorCount; i++) {
        STEPMOTOR_EnableControl(&motorArray[i], enable);
    }
}
