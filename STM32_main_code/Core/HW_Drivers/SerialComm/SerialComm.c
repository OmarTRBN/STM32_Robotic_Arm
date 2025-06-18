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

SerialComm_StatusTypeDef SerialComm_Init(SerialComm_HandleTypeDef* hserial, uint8_t* pRxBuffer, uint8_t* pTxBuffer) {
    if (hserial == NULL || pRxBuffer == NULL || pTxBuffer == NULL) {
        return SERIALCOMM_INVALID_PARAM;
    }

    hserial->pRxBuffer     = pRxBuffer;
    hserial->pTxBuffer     = pTxBuffer;
    hserial->rxBufferSize  = SERIALCOMM_RX_BUFF_SIZE;
    hserial->txBufferSize  = SERIALCOMM_TX_BUFF_SIZE;

    hserial->rxIndex       = 0;
    hserial->isInitialized = true;
    hserial->responseReadyFlag = 0;

    HAL_UART_Receive_DMA(SERIALCOMM_UART, &(hserial->pRxBuffer[hserial->rxIndex]), 1);

    return SERIALCOMM_OK;
}

SerialComm_StatusTypeDef SerialComm_ProcessByte(SerialComm_HandleTypeDef* hserial) {
    if (hserial == NULL || !hserial->isInitialized || hserial->pRxBuffer == NULL) {
        return SERIALCOMM_INVALID_PARAM;
    }

    uint8_t byte = hserial->pRxBuffer[hserial->rxIndex];

    // If within range
    if (hserial->rxIndex < hserial->rxBufferSize - 1)
    {
        if (byte == '\n')
        {
            hserial->pRxBuffer[hserial->rxIndex] = '\0';	// Terminate command string
            hserial->rxIndex = 0;  							// Reset for next command
            hserial->responseReadyFlag = true;
        }
        else
        {
        	hserial->rxIndex++;
        }
    }
    else
    {
        // Buffer overflow: reset index and optionally clear buffer
        hserial->rxIndex = 0;
        return SERIALCOMM_ERROR;
    }

    return SERIALCOMM_OK;
}

SerialComm_StatusTypeDef SerialComm_Transmit(SerialComm_HandleTypeDef* hserial) {
    if (hserial == NULL) {
        return SERIALCOMM_INVALID_PARAM;
    }

    uint16_t len = strlen((char*)hserial->pTxBuffer);
    if (HAL_UART_Transmit_DMA(SERIALCOMM_UART, hserial->pTxBuffer, len) != HAL_OK) {
        return SERIALCOMM_ERROR;
    }

    return SERIALCOMM_OK;
}

void SerialComm_RxCpltCallback(SerialComm_HandleTypeDef* hserial) {
    SerialComm_ProcessByte(hserial);                									// Process byte
    HAL_UART_Receive_DMA(SERIALCOMM_UART, &(hserial->pRxBuffer[hserial->rxIndex]), 1);  // Restart reception
}
