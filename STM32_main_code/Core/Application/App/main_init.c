/*
 * main_init.c
 *
 *  Created on: Jun 19, 2025
 *      Author: omarmac
 */

#include "main_init.h"

void App_Init(void)
{
    // 🥊 SerialComm setup
    appSerialStatus = SerialComm_Init(&appSerialHandle, appSerialRxArray, appSerialTxArray);

    // 🥊 AS5600 multiplexer setup
    appMuxStatus = AS5600_MUX_Init(&appMuxHandle, NUM_JOINTS);

    // 🥊 StepMotor setup
    App_InitMotors();

    // 🥊 PID controller setup
    MultivariablePID_Init(&appPidObj);
    appPidObj.dt = 1.0f / ((float) APP_CONTROLLER_FREQ);

    for (int i = 0; i < NUM_JOINTS; i++) {
        q_set[i] = 2048.0f;
        q_meas[i] = 2048.0f;
        q_out[i] = 2048.0f;
    }
}
