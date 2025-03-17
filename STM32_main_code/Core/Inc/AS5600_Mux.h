/*
 * AS5600_Mux.h
 *
 *  Created on: Mar 14, 2025
 *      Author: omart
 */

#ifndef INC_AS5600_MUX_H_
#define INC_AS5600_MUX_H_

#include "main.h"

#define MAX_SENSORS 8

typedef struct {
    I2C_HandleTypeDef* hi2c;     // I2C handle
    uint8_t num_sensors;         // Number of active sensors
    uint16_t angles[MAX_SENSORS]; // Array to store all raw angle readings
    HAL_StatusTypeDef sensor_status_array[MAX_SENSORS];
} AS5600_Mux_Array;

/* Function prototypes - Minimal set required */
HAL_StatusTypeDef AS5600_Mux_Init(AS5600_Mux_Array* sensors, I2C_HandleTypeDef* hi2c, uint8_t num_sensors);
HAL_StatusTypeDef AS5600_Mux_SelectChannel(AS5600_Mux_Array* sensors, uint8_t channel);
HAL_StatusTypeDef AS5600_Mux_ReadAngle(AS5600_Mux_Array* sensors, uint8_t sensor_index, uint16_t* angle);
void AS5600_Mux_ReadAllAngles(AS5600_Mux_Array* sensors);

float AS5600_Mux_ConvertToDegrees(uint16_t raw_angle);
float AS5600_Mux_ConvertToRadians(uint16_t raw_angle);

#endif /* INC_AS5600_MUX_H_ */


