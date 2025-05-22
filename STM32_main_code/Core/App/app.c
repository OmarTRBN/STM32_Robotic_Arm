/*
 * app.c
 *
 *  Created on: May 11, 2025
 *      Author: omart
 */

#include "app.h"

/* 🥊🥊🥊 Serial Communication 🥊🥊🥊 */
SerialComm_HandleTypeDef appSerialHandle;
SerialComm_StatusTypeDef appSerialStatus;
uint8_t appSerialDataArray[SERIALCOMM_BUFF_SIZE] = {0};

/* 🥊🥊🥊 Encoders 🥊🥊🥊 */
AS5600_MUX_HandleTypeDef appMuxHandle;
AS5600_MUX_StatusTypeDef appMuxStatus;

/*🥊🥊🥊 Motors 🥊🥊🥊*/
STEPMOTOR_HandleTypeDef appStepMotors[NUM_MOTORS];
STEPMOTOR_StatusTypeDef appStepMotorStatus;

void App_InitMotors(void) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        appStepMotorStatus = STEPMOTOR_Init(&appStepMotors[i], i);
    }
}

/*🥊🥊🥊 PID Control 🥊🥊🥊*/
float q_set[NUM_MOTORS] = { 2048.0, 2048.0 };
float q_meas[NUM_MOTORS] = { 2048.0, 2048.0 };
float q_out[NUM_MOTORS] = {0, 0};
MultivariablePID appPidObj;
