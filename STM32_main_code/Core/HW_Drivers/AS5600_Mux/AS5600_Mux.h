/*
 * AS5600_Mux.h
 *
 *  Created on: May 11, 2025
 *      Author: omart
 */

#ifndef HW_DRIVERS_AS5600_MUX_AS5600_MUX_H_
#define HW_DRIVERS_AS5600_MUX_AS5600_MUX_H_

#include "AS5600_Mux_Def.h"

#include "main.h"
#include "i2c.h"

AS5600_MUX_StatusTypeDef AS5600_MUX_Init(AS5600_MUX_HandleTypeDef *, uint8_t);

AS5600_MUX_StatusTypeDef AS5600_MUX_StartStopDMA(AS5600_MUX_HandleTypeDef *, bool);
AS5600_MUX_StatusTypeDef AS5600_MUX_LoopDMA(AS5600_MUX_HandleTypeDef *);
void AS5600_MUX_MemRxCpltCallback(AS5600_MUX_HandleTypeDef *);
void AS5600_MUX_TxCpltCallback(AS5600_MUX_HandleTypeDef *);

AS5600_MUX_StatusTypeDef AS5600_MUX_ReadAllPolling(AS5600_MUX_HandleTypeDef *handle);
AS5600_MUX_StatusTypeDef AS5600_MUX_ReadChannel(AS5600_MUX_HandleTypeDef *, uint8_t);

#endif /* HW_DRIVERS_AS5600_MUX_AS5600_MUX_H_ */
