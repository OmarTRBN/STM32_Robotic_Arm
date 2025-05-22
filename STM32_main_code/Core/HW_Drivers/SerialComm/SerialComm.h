/*
 * SerialComm.h
 *
 *  Created on: May 22, 2025
 *      Author: omart
 */

#ifndef HW_DRIVERS_SERIALCOMM_SERIALCOMM_H_
#define HW_DRIVERS_SERIALCOMM_SERIALCOMM_H_

#include "main.h"
#include "usart.h"

#define SERIALCOMM_UART &huart1
#define SERIALCOMM_BUFF_SIZE 1000

typedef enum {
    SERIALCOMM_OK = 0,
    SERIALCOMM_ERROR,
    SERIALCOMM_TIMEOUT,
    SERIALCOMM_BUSY,
    SERIALCOMM_INVALID_PARAM
} SerialComm_StatusTypeDef;

typedef struct {
    uint8_t*     pRxBuffer;       // External buffer pointer
    uint16_t     rxBufferSize;    // Buffer size
    uint16_t     rxIndex;         // Write index
    uint32_t     timeout;         // Timeout for processing
    bool         isInitialized;   // Init flag
} SerialComm_HandleTypeDef;

// Function pointer for processing complete command frames
typedef void (*SerialComm_CommandCallback)(SerialComm_HandleTypeDef* hserial);


SerialComm_StatusTypeDef SerialComm_Init(SerialComm_HandleTypeDef* hserial, uint8_t* pBuffer, uint16_t bufferSize, uint32_t timeout);
SerialComm_StatusTypeDef SerialComm_ProcessByte(SerialComm_HandleTypeDef* hserial, uint8_t byte);
SerialComm_StatusTypeDef SerialComm_SendResponse(SerialComm_HandleTypeDef *, const char* response);
void SerialComm_SetCommandCallback(SerialComm_CommandCallback cb);

void SerialComm_RxCpltCallback(SerialComm_HandleTypeDef* hserial);

#endif /* HW_DRIVERS_SERIALCOMM_SERIALCOMM_H_ */
