/*
 * AS5600_Mux.c
 *
 *  Created on: May 11, 2025
 *      Author: omart
 */

#include "AS5600_Mux.h"

static AS5600_MUX_StatusTypeDef SelectMuxAddrDMA(AS5600_MUX_HandleTypeDef *handle) {
	handle->dma_busy = 1;

	uint8_t channel_select = 1 << handle->dma_current_channel;

	if (HAL_I2C_Master_Transmit_DMA(AS5600_MUX_I2C, TCA9548A_ADDR, &channel_select, 1) != HAL_OK) {
		return AS5600_MUX_ERROR;
	}
	else {
		return AS5600_MUX_OK;
	}
}
static AS5600_MUX_StatusTypeDef ReadAS5600DMA(AS5600_MUX_HandleTypeDef *handle) {
	handle->dma_busy = 1;

	if (HAL_I2C_Mem_Read_DMA(AS5600_MUX_I2C, AS5600_ADDR, ANGLE_MSB_REG, I2C_MEMADD_SIZE_8BIT, handle->dma_buffer, 2) != HAL_OK) {
		return AS5600_MUX_ERROR;
	}
	else {
		return AS5600_MUX_OK;
	}
}
static void HandleCpltDMA(AS5600_MUX_HandleTypeDef *handle) {
	handle->dma_busy = 1;

	handle->channel_raw_values[handle->dma_current_channel] = ((handle->dma_buffer[0] << 8) | handle->dma_buffer[1]) & 0x0FFF;
	handle->dma_current_channel++;

	if (handle->dma_current_channel >= handle->num_channels) {
		if (handle->dma_mode_start_stop == AS5600_MUX_DMA_RUN) {
			handle->dma_current_channel = 0;
			handle->dma_loop_state = AS5600_MUX_DMA_READY;
		}
		else {
			handle->dma_loop_state = AS5600_MUX_DMA_IDLE;
		}
	}
	else {
		handle->dma_loop_state = AS5600_MUX_DMA_READY;
	}

	handle->dma_busy = 0;
}

AS5600_MUX_StatusTypeDef AS5600_MUX_Init(AS5600_MUX_HandleTypeDef *handle, uint8_t num_of_sensors) {
    if (!handle || num_of_sensors > AS5600_MUX_MAX_CHANNELS) return AS5600_MUX_ERROR;

    handle->is_initialized = AS5600_MUX_INITIALIZED;

    handle->dma_mode_start_stop = AS5600_MUX_DMA_STOP;
    handle->dma_loop_state = AS5600_MUX_DMA_IDLE;
    handle->dma_busy = true;

    handle->num_channels = num_of_sensors;
    memset(handle->channel_raw_values, 0, AS5600_MUX_MAX_CHANNELS * sizeof(uint16_t));
    memset(handle->channel_states, AS5600_MUX_NONE, AS5600_MUX_MAX_CHANNELS * sizeof(uint16_t));

    return AS5600_MUX_OK;
}

AS5600_MUX_StatusTypeDef AS5600_MUX_StartStopDMA(AS5600_MUX_HandleTypeDef *handle, bool dma_start) {
	if (!handle || !handle->is_initialized) return AS5600_MUX_ERROR;

	if (dma_start == AS5600_MUX_DMA_RUN) {
		handle->dma_current_channel = 0;
		handle->dma_loop_state = AS5600_MUX_DMA_READY;
		handle->dma_mode_start_stop = 1;
	    handle->dma_busy = 0;
	}
	else {
	    handle->dma_busy = 1;
		handle->dma_current_channel = 0;
		handle->dma_loop_state = AS5600_MUX_DMA_IDLE;
		handle->dma_mode_start_stop = 0;
	}

	return AS5600_MUX_OK;
}
AS5600_MUX_StatusTypeDef AS5600_MUX_LoopDMA(AS5600_MUX_HandleTypeDef *handle) {
	switch (handle->dma_loop_state) {
		case AS5600_MUX_DMA_IDLE:
			break;

		case AS5600_MUX_DMA_READY:
			if (!handle->dma_busy) handle->channel_states[handle->dma_current_channel] = SelectMuxAddrDMA(handle);
			break;

		case AS5600_MUX_READ_AS5600:
			if (!handle->dma_busy) handle->channel_states[handle->dma_current_channel] = ReadAS5600DMA(handle);
			break;

		case AS5600_MUX_DMA_DONE:
			if (!handle->dma_busy) HandleCpltDMA(handle);
			break;

		default:
			break;
	}

	return AS5600_MUX_OK;
}
void AS5600_MUX_TxCpltCallback(AS5600_MUX_HandleTypeDef *handle) {
	handle->dma_loop_state = AS5600_MUX_READ_AS5600;
	handle->dma_busy = 0;
}
void AS5600_MUX_MemRxCpltCallback(AS5600_MUX_HandleTypeDef *handle) {
	handle->dma_loop_state = AS5600_MUX_DMA_DONE;
	handle->dma_busy = 0;
}

AS5600_MUX_StatusTypeDef AS5600_MUX_ReadAllPolling(AS5600_MUX_HandleTypeDef *handle) {
	if (!handle) return AS5600_MUX_ERROR;
	if (!handle->is_initialized || handle->dma_mode_start_stop == AS5600_MUX_DMA_RUN) return AS5600_MUX_ERROR;

    uint8_t channel_select = 1;
    uint8_t buffer[2];

	for (int i=0; i<handle->num_channels; i++) {
		handle->channel_states[i] = AS5600_MUX_OK;
		channel_select = 1 << i;

		// Switch channel
		if (HAL_I2C_Master_Transmit(AS5600_MUX_I2C, TCA9548A_ADDR, &channel_select, 1, AS5600_MUX_TIMEOUT) != HAL_OK) {
			handle->channel_states[i] = AS5600_MUX_ERROR;
		}

		// Read angle
		if (HAL_I2C_Mem_Read(AS5600_MUX_I2C, AS5600_ADDR, ANGLE_MSB_REG, I2C_MEMADD_SIZE_8BIT, buffer, 2, AS5600_MUX_TIMEOUT) != HAL_OK) {
			handle->channel_states[i] = AS5600_MUX_ERROR;
		}

	    handle->channel_raw_values[i] = ((buffer[0] << 8) | buffer[1]) & 0x0FFF;
	}

	return AS5600_MUX_OK;
}
AS5600_MUX_StatusTypeDef AS5600_MUX_ReadChannel(AS5600_MUX_HandleTypeDef *handle, uint8_t channel) {
	if (!handle || channel >= handle->num_channels) return AS5600_MUX_ERROR;
	if (handle->dma_mode_start_stop == AS5600_MUX_DMA_RUN) return AS5600_MUX_ERROR;

    uint8_t channel_select = 1 << channel;
    uint8_t buffer[2];

    // Select channel on TCA9548A
    if (HAL_I2C_Master_Transmit(AS5600_MUX_I2C, TCA9548A_ADDR, &channel_select, 1, AS5600_MUX_TIMEOUT) != HAL_OK) {
        return AS5600_MUX_ERROR;
    }

    // Read full angle
    if (HAL_I2C_Mem_Read(AS5600_MUX_I2C, AS5600_ADDR, ANGLE_MSB_REG, I2C_MEMADD_SIZE_8BIT, buffer, 2, AS5600_MUX_TIMEOUT) != HAL_OK) {
    	return AS5600_MUX_ERROR;
    }

    handle->channel_raw_values[channel] = ((buffer[0] << 8) | buffer[1]) & 0x0FFF;

	return AS5600_MUX_OK;
}
