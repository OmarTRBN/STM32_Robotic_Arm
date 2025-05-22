/*
 * StepMotor_def.h
 *
 *  Created on: May 15, 2025
 *      Author: omart
 */

#ifndef HW_DRIVERS_STEPMOTOR_STEPMOTOR_DEF_H_
#define HW_DRIVERS_STEPMOTOR_STEPMOTOR_DEF_H_

#include "main.h"

#define STEPMOTOR_MAX_CHANNELS 8
#define STEPMOTOR_MIN_LUT_SPEED 1
#define STEPMOTOR_MAX_LUT_SPEED LUT_SIZE

#define STEPMOTOR_CW  0
#define STEPMOTOR_CCW 1

typedef enum {
    STEPMOTOR_OK,
    STEPMOTOR_ERROR
} STEPMOTOR_StatusTypeDef;

typedef struct {
    TIM_HandleTypeDef* timer;
    uint32_t channel;

    GPIO_TypeDef* dir_gpio_port;
    uint16_t dir_gpio_pin;

    GPIO_TypeDef* en_gpio_port;
    uint16_t en_gpio_pin;
} STEPMOTOR_HWConfig;

typedef struct {
    STEPMOTOR_HWConfig hw;
    int16_t last_speed;
    uint16_t now_speed;
} STEPMOTOR_HandleTypeDef;

#endif /* HW_DRIVERS_STEPMOTOR_STEPMOTOR_DEF_H_ */
