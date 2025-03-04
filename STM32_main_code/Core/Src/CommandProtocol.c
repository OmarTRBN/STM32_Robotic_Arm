/*
 * CommandProtocol.c
 *
 *  Created on: Feb 23, 2025
 *      Author: omart
 */

#include "CommandProtocol.h"
#include "StepMotor.h"

static ProcessCommandFn CustomProcessCommand = NULL;

void CommandProtocol_SetCommandProcessor(ProcessCommandFn processFn) {
    CustomProcessCommand = processFn;
}

HAL_StatusTypeDef CommandProtocol_Init(CommandProtocol_Handle* handle, UART_HandleTypeDef* huart, uint32_t timeout) {
    if (handle == NULL || huart == NULL) {
        return HAL_ERROR;
    }

    handle->huart = huart;
	handle->timeout = timeout;
    handle->rxIndex = 0;
    handle->isInitialized = true;

    return HAL_UART_Receive_IT(handle->huart, &handle->rxBuffer[0], 1);
}

HAL_StatusTypeDef CommandProtocol_ProcessByte(CommandProtocol_Handle* handle, uint8_t byte, char* dataArray) {
    if (!handle->isInitialized) {
        return HAL_ERROR;
    }

    if (handle->rxIndex < sizeof(handle->rxBuffer) - 1)
    {
        if (byte == '\n' || byte == '\r')
        {
            handle->rxBuffer[handle->rxIndex] = '\0';

            if (CustomProcessCommand != NULL)
            {
                CustomProcessCommand(handle, dataArray);
            }
            else
            {
            	return HAL_ERROR;
            }

            handle->rxIndex = 0;
        }
        else
        {
            handle->rxBuffer[handle->rxIndex++] = byte;
        }
    }
    else
    {
        handle->rxIndex = 0;
    }
    return HAL_OK;
}

HAL_StatusTypeDef CommandProtocol_SendResponse(CommandProtocol_Handle* handle, const char* response) {
    if (!handle->isInitialized || response == NULL) {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(handle->huart, (uint8_t*)response, strlen(response),
                            handle->timeout);
}





