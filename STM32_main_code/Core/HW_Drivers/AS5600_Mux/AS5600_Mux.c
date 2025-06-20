/*
 * AS5600_Mux.c
 *
 *  Created on: May 11, 2025
 *      Author: omart
 */

#include "AS5600_Mux.h"

AS5600_MUX_StatusTypeDef AS5600_MUX_Init(AS5600_MUX_HandleTypeDef *handle, uint8_t num_of_sensors) {
    if (!handle || num_of_sensors > AS5600_MUX_MAX_CHANNELS) return AS5600_MUX_ERROR;

    handle->is_initialized = AS5600_MUX_INITIALIZED;

    handle->num_channels = num_of_sensors;
    memset(handle->channel_states, AS5600_MUX_NONE, AS5600_MUX_MAX_CHANNELS * sizeof(AS5600_MUX_StatusTypeDef));
    memset(handle->channel_raw_values, AS5600_MUX_DEFAULT_HANDLE, AS5600_MUX_MAX_CHANNELS * sizeof(uint16_t));

    return AS5600_MUX_OK;
}

AS5600_MUX_StatusTypeDef AS5600_MUX_ReadAllPolling(AS5600_MUX_HandleTypeDef *handle) {
	if (!handle || !handle->is_initialized) return AS5600_MUX_ERROR;

	AS5600_MUX_StatusTypeDef status = AS5600_MUX_OK;
    uint8_t channel_select = 1;
    uint8_t buffer[2];

	for (int i=0; i<handle->num_channels; i++) {
		channel_select = (1 << i);

		// Switch channel
		if (HAL_I2C_Master_Transmit(AS5600_MUX_I2C, TCA9548A_ADDR, &channel_select, 1, AS5600_MUX_TIMEOUT) != HAL_OK)
		{
			status = AS5600_MUX_ERROR;
			handle->channel_states[i] = AS5600_MUX_ERROR;
			handle->channel_raw_values[i] = AS5600_MUX_DEFAULT_HANDLE;
		}
		else
		{
			// Read angle
			if (HAL_I2C_Mem_Read(AS5600_MUX_I2C, AS5600_ADDR, ANGLE_MSB_REG, I2C_MEMADD_SIZE_8BIT, buffer, 2, AS5600_MUX_TIMEOUT) != HAL_OK)
			{
				status = AS5600_MUX_ERROR;
				handle->channel_states[i] = AS5600_MUX_ERROR;
				handle->channel_raw_values[i] = AS5600_MUX_DEFAULT_HANDLE;
			}
			else
			{
				handle->channel_states[i] = AS5600_MUX_OK;
				handle->channel_raw_values[i] = ((buffer[0] << 8) | buffer[1]) & 0x0FFF;
			}
		}
	}

	return status;
}
