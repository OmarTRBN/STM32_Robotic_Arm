/*
 * app.h
 *
 *  Created on: May 11, 2025
 *      Author: omart
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"

#include "SerialComm.h"
#include "AS5600_Mux.h"
#include "StepMotor.h"
#include "PID_Control.h"

/* 🥊🥊🥊 Serial Communication 🥊🥊🥊 */
extern SerialComm_HandleTypeDef appSerialHandle;
extern SerialComm_StatusTypeDef appSerialStatus;
extern uint8_t appSerialDataArray[SERIALCOMM_BUFF_SIZE];

/* 🥊🥊🥊 Encoders 🥊🥊🥊 */
#define APP_ENCODER_FREQ 250

extern AS5600_MUX_HandleTypeDef appMuxHandle;
extern AS5600_MUX_StatusTypeDef appMuxStatus;

/*🥊🥊🥊 Motors 🥊🥊🥊*/
#define NUM_MOTORS 2

extern STEPMOTOR_HandleTypeDef appStepMotors[NUM_MOTORS];
extern STEPMOTOR_StatusTypeDef appStepMotorStatus;

void App_InitMotors(void);

/*🥊🥊🥊 Controller Variables 🥊🥊🥊*/
#define APP_CONTROLLER_FREQ 100

extern float q_set[NUM_JOINTS];
extern float q_meas[NUM_JOINTS];
extern float q_out[NUM_JOINTS];

extern MultivariablePID appPidObj;

#endif /* INC_APP_H_ */
