/*
 * CommandProtocol.h
 *
 *  Created on: Feb 23, 2025
 *      Author: omart
 */

#ifndef INC_COMMANDPROTOCOL_H_
#define INC_COMMANDPROTOCOL_H_

#include "main.h"
#include "app.h"

typedef struct {
	UART_HandleTypeDef* huart;
	uint32_t timeout;
    uint8_t rxBuffer[1000];
    uint8_t rxIndex;
    bool isInitialized;
} CommandProtocol_Handle;

typedef void (*ProcessCommandFn)(CommandProtocol_Handle* handle);

// Set this function pointer in main.c to your own implementation
void CommandProtocol_SetCommandProcessor(ProcessCommandFn processFn);

HAL_StatusTypeDef CommandProtocol_Init(CommandProtocol_Handle* handle, UART_HandleTypeDef* huart, uint32_t timeout);
HAL_StatusTypeDef CommandProtocol_ProcessByte(CommandProtocol_Handle* handle, uint8_t byte);
HAL_StatusTypeDef CommandProtocol_SendResponse(CommandProtocol_Handle* handle, const char* response);

#endif /* INC_COMMANDPROTOCOL_H_ */




