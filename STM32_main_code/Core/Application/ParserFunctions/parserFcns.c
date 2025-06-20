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

uint8_t Parse_CMD_SET_PID(SerialComm_HandleTypeDef* hserial, MultivariablePID* pid) {
    char paramType[3] = { hserial->pRxBuffer[2], hserial->pRxBuffer[3], '\0' };
    char *uart_str = (char *)&hserial->pRxBuffer[2];
    uint16_t len = strlen(uart_str);

    // Ensure null termination
    if (uart_str[len - 1] != '\0') {
        if (len < hserial->rxBufferSize - 1) {
            uart_str[len] = '\0';
        } else {
            sprintf((char *)hserial->pTxBuffer, "Error: PID input too long!\n");
            return 0;
        }
    }

    // Determine parameter type
    uint16_t chosen_param;
    if (strncmp(paramType, "KP", 2) == 0) {
        chosen_param = CMD_SET_KP;
    } else if (strncmp(paramType, "KI", 2) == 0) {
        chosen_param = CMD_SET_KI;
    } else if (strncmp(paramType, "KD", 2) == 0) {
        chosen_param = CMD_SET_KD;
    } else {
        sprintf((char *)hserial->pTxBuffer, "Error: Unknown PID parameter type '%s'\n", paramType);
        return 0;
    }

    // Parse float values
    float32_t parsed_values[NUM_JOINTS * NUM_JOINTS] = {0};
    char *data_start = uart_str + 2;
    char *token;
    char *rest = data_start;
    int index = 0;

    while ((token = strtok_r(rest, ",", &rest)) != NULL && index < NUM_JOINTS * NUM_JOINTS) {
        parsed_values[index++] = (float32_t)atof(token);
    }

    if (index != NUM_JOINTS * NUM_JOINTS) {
        sprintf((char *)hserial->pTxBuffer, "Error: Expected %d floats for PID %s, got %d.\n",
                NUM_JOINTS * NUM_JOINTS, paramType, index);
        return 0;
    }

    MultivariablePID_SetParameter(pid, parsed_values, chosen_param);
    sprintf((char *)hserial->pTxBuffer, "PID %s parameters updated successfully.\n", paramType);
    return 1;
}
