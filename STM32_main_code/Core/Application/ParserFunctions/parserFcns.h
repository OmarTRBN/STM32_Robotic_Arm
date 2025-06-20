/*
 * parserFcns.h
 *
 *  Created on: Jun 19, 2025
 *      Author: omarmac
 */

#ifndef APP_PARSERFUNCTIONS_PARSERFCNS_H_
#define APP_PARSERFUNCTIONS_PARSERFCNS_H_

#include "app_includes.h"

void Parse_CMD_TEST_LED(SerialComm_HandleTypeDef* hserial);
void Parse_CMD_MOTOR_STATE(SerialComm_HandleTypeDef* hserial);
void Parse_CMD_MOTOR_REF(SerialComm_HandleTypeDef* hserial);
uint8_t Parse_CMD_SET_PID(SerialComm_HandleTypeDef* hserial, MultivariablePID* pid);

#endif /* APP_PARSERFUNCTIONS_PARSERFCNS_H_ */
