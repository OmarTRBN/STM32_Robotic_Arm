/*
 * CommandProtocol.h
 *
 *  Created on: Feb 23, 2025
 *      Author: omart
 */

#ifndef INC_COMMANDPROTOCOL_H_
#define INC_COMMANDPROTOCOL_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "AS5600_Multi.h"


typedef enum {
    CMD_LED_CONTROL    = 'A',
	CMD_AS5600_DATA    = 'B',
	CMD_STEP_FREQ      = 'F',
//    CMD_SET_KP         = 'P',
//    CMD_SET_KI         = 'I',
//    CMD_SET_KD         = 'D',
//    CMD_GET_POSITION   = 'G',
//    CMD_SET_TRAJECTORY = 'T',
//    CMD_EMERGENCY_STOP = 'E'
} CommandID;

typedef struct {
	UART_HandleTypeDef* huart;
	uint32_t timeout;
    uint8_t rxBuffer[50];
    uint8_t rxIndex;
    bool isInitialized;
} CommandProtocol_Handle;

typedef void (*ProcessCommandFn)(CommandProtocol_Handle* handle, char *dataArray);

// Set this function pointer in main.c to your own implementation
void CommandProtocol_SetCommandProcessor(ProcessCommandFn processFn);

HAL_StatusTypeDef CommandProtocol_Init(CommandProtocol_Handle* handle, UART_HandleTypeDef* huart, uint32_t timeout);
HAL_StatusTypeDef CommandProtocol_ProcessByte(CommandProtocol_Handle* handle, uint8_t byte, char* dataArray);
HAL_StatusTypeDef CommandProtocol_SendResponse(CommandProtocol_Handle* handle, const char* response);

#endif /* INC_COMMANDPROTOCOL_H_ */




