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
#include "Trajectory.h"

/* 🥊🥊🥊 Serial Communication 🥊🥊🥊 */
extern SerialComm_HandleTypeDef appSerialHandle;
extern SerialComm_StatusTypeDef appSerialStatus;
extern uint8_t appSerialRxArray[SERIALCOMM_RX_BUFF_SIZE];
extern uint8_t appSerialTxArray[SERIALCOMM_TX_BUFF_SIZE];

/* 🥊🥊🥊 Encoders 🥊🥊🥊 */

extern AS5600_MUX_HandleTypeDef appMuxHandle;
extern AS5600_MUX_StatusTypeDef appMuxStatus;

/*🥊🥊🥊 Motors 🥊🥊🥊*/
extern STEPMOTOR_HandleTypeDef appStepMotors[NUM_JOINTS];
extern STEPMOTOR_StatusTypeDef appStepMotorStatus;

void App_InitMotors(void);

/*🥊🥊🥊 Controller Variables 🥊🥊🥊*/

extern float q_set[NUM_JOINTS];
extern float q_meas[NUM_JOINTS];
extern float q_out[NUM_JOINTS];

extern MultivariablePID appPidObj;

/*🥊🥊🥊 Trajectory 🥊🥊🥊*/
extern Trajectory robotTraj;

#endif /* INC_APP_H_ */
