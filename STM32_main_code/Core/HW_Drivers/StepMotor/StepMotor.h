/*
 * StepMotor.h
 *
 *  Created on: Mar 2, 2025
 *      Author: omart
 */

#ifndef HW_DRIVERS_STEPMOTOR_STEPMOTOR_H_
#define HW_DRIVERS_STEPMOTOR_STEPMOTOR_H_

#include "tim.h"

#include "StepMotor_Def.h"
#include "lut.h"

STEPMOTOR_StatusTypeDef STEPMOTOR_Init(STEPMOTOR_HandleTypeDef* motor, uint8_t index);

void STEPMOTOR_SetSpeedLUT(STEPMOTOR_HandleTypeDef* motor, int16_t speed);
void STEPMOTOR_EnableControl(STEPMOTOR_HandleTypeDef* motor, bool enable);

void STEPMOTOR_MultiSetSpeed(STEPMOTOR_HandleTypeDef* motorArray, float speedArray[], uint8_t motorCount);
void STEPMOTOR_MultiEnableControl(STEPMOTOR_HandleTypeDef* motorArray, uint8_t motorCount, bool enable);

#endif /* HW_DRIVERS_STEPMOTOR_STEPMOTOR_H_ */
