/*
 * SerialComm.c
 *
 *  Created on: May 22, 2025
 *      Author: omart
 */

#include "SerialComm.h"

static SerialComm_CommandCallback CommandCallback = NULL;
void SerialComm_SetCommandCallback(SerialComm_CommandCallback cb) {
    CommandCallback = cb;
}

SerialComm_StatusTypeDef SerialComm_Init(SerialComm_HandleTypeDef* hserial, uint8_t* pBuffer, uint16_t bufferSize, uint32_t timeout) {
    if (hserial == NULL || pBuffer == NULL || bufferSize == 0) {
        return SERIALCOMM_INVALID_PARAM;
    }

    hserial->pRxBuffer     = pBuffer;
    hserial->rxBufferSize  = bufferSize;
    hserial->rxIndex       = 0;
    hserial->timeout       = timeout;
    hserial->isInitialized = true;

    if (HAL_UART_Receive_IT(SERIALCOMM_UART, &hserial->pRxBuffer[hserial->rxIndex], 1) != HAL_OK) {
        return SERIALCOMM_ERROR;
    }

    return SERIALCOMM_OK;
}

SerialComm_StatusTypeDef SerialComm_ProcessByte(SerialComm_HandleTypeDef* hserial, uint8_t byte) {
    if (hserial == NULL || !hserial->isInitialized || hserial->pRxBuffer == NULL) {
        return SERIALCOMM_INVALID_PARAM;
    }

    if (hserial->rxIndex < hserial->rxBufferSize - 1) {
        if (byte == '\n' || byte == '\r') {
            // Terminate command string
            hserial->pRxBuffer[hserial->rxIndex] = '\0';

            if (CommandCallback != NULL) {
                CommandCallback(hserial);
            }
            else {
                return SERIALCOMM_ERROR;
            }

            hserial->rxIndex = 0;  // Reset for next command
        }
        else {
            hserial->pRxBuffer[hserial->rxIndex++] = byte;
        }
    }
    else {
        // Buffer overflow: reset index and optionally clear buffer
        hserial->rxIndex = 0;
        return SERIALCOMM_ERROR;
    }

    return SERIALCOMM_OK;
}

SerialComm_StatusTypeDef SerialComm_SendResponse(SerialComm_HandleTypeDef* hserial, const char* response) {
    if (hserial == NULL || response == NULL) {
        return SERIALCOMM_INVALID_PARAM;
    }

    if (HAL_UART_Transmit(SERIALCOMM_UART, (uint8_t*)response, strlen(response), HAL_MAX_DELAY) != HAL_OK) {
        return SERIALCOMM_ERROR;
    }

    return SERIALCOMM_OK;
}

void SerialComm_RxCpltCallback(SerialComm_HandleTypeDef* hserial) {
	SerialComm_ProcessByte(hserial, hserial->pRxBuffer[hserial->rxIndex]);
	HAL_UART_Receive_IT(SERIALCOMM_UART, &hserial->pRxBuffer[hserial->rxIndex], 1);
}

