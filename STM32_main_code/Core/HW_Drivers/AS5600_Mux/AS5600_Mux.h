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

AS5600_MUX_StatusTypeDef AS5600_MUX_Init(AS5600_MUX_HandleTypeDef *handle, uint8_t num_of_sensors);

AS5600_MUX_StatusTypeDef AS5600_MUX_ReadAllPolling(AS5600_MUX_HandleTypeDef *handle);

#endif /* HW_DRIVERS_AS5600_MUX_AS5600_MUX_H_ */
