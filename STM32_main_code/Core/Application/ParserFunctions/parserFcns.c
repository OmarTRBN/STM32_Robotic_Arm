/*
 * parserFcns.c
 *
 *  Created on: Jun 19, 2025
 *      Author: omarmac
 */

#include <parserFcns.h>

void Parse_CMD_TEST_LED(SerialComm_HandleTypeDef* hserial) {
    HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
    sprintf((char*)hserial->pTxBuffer, "<d>LED toggled.\n");
}

void Parse_CMD_MOTOR_STATE(SerialComm_HandleTypeDef* hserial) {
    uint8_t* data = hserial->pRxBuffer;
    int index = data[2] - '0';
    int state = data[3] - '0';

    STEPMOTOR_EnableControl(&appStepMotors[index], state);
    sprintf((char*)hserial->pTxBuffer, "<d>Motor %d is at state %d\n", index, state);
}

void Parse_CMD_MOTOR_REF(SerialComm_HandleTypeDef* hserial) {
    uint8_t* data = hserial->pRxBuffer;
    int index = data[2] - '0';

    char* valueStr = (char*)&data[4];
    char* newline = strchr(valueStr, '\n');
    if (newline) *newline = '\0';

    int value = atoi(valueStr);

    if (index < NUM_JOINTS && index >=0) {
    	q_set[index] = (float)value;
        sprintf((char*)hserial->pTxBuffer, "<d>Set motor %d reference to %d\n", index, value);
    }
    else {
        sprintf((char*)hserial->pTxBuffer, "<d>Index out of reach!\n");
    }
}

uint8_t Parse_CMD_SET_PID(SerialComm_HandleTypeDef* hserial, uint8_t* receivedData, MultivariablePID* pid) {
    if (pid == NULL || receivedData == NULL) return 0;

    // Cast receivedData to char* for string handling
    char* receivedStr = (char*)receivedData;

    // Check header "CP"
    if (strncmp(receivedStr, "CP", 2) != 0) return 0;

    // Extract parameter type (KP, KI, KD)
    char paramType[3] = { receivedStr[2], receivedStr[3], '\0' };

    uint16_t chosen_param;
    if (strcmp(paramType, "KP") == 0) {
        chosen_param = CMD_SET_KP;
    } else if (strcmp(paramType, "KI") == 0) {
        chosen_param = CMD_SET_KI;
    } else if (strcmp(paramType, "KD") == 0) {
        chosen_param = CMD_SET_KD;
    } else {
        return 0;
    }

    float parsed_values[NUM_JOINTS] = {0};

    // Copy the data part to a mutable buffer
    char dataBuffer[128];  // Make sure this is large enough
    strncpy(dataBuffer, &receivedStr[4], sizeof(dataBuffer) - 1);
    dataBuffer[sizeof(dataBuffer) - 1] = '\0';  // Null-terminate

    char *token;
    char *rest = dataBuffer;
    int index = 0;

    while ((token = strtok_r(rest, ",", &rest)) != NULL && index < NUM_JOINTS) {
        parsed_values[index++] = atof(token);
    }

    if (index != NUM_JOINTS) return 0;

    MultivariablePID_SetParameter(pid, parsed_values, chosen_param);
//
//    snprintf((char *)hserial->pTxBuffer, hserial->txBufferSize,
//             "PID %s parameters updated successfully.\n", paramType);

    return 1;
}
