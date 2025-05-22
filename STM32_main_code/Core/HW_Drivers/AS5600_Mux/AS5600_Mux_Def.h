/*
 * AS5600_Mux_Def.h
 *
 *  Created on: May 11, 2025
 *      Author: omart
 */

#ifndef HW_DRIVERS_AS5600_MUX_AS5600_MUX_DEF_H_
#define HW_DRIVERS_AS5600_MUX_AS5600_MUX_DEF_H_

#include "main.h"

#define TCA9548A_ADDR  (0x70 << 1)
#define AS5600_ADDR    (0x36 << 1)
#define ANGLE_MSB_REG  0x0E
#define ANGLE_LSB_REG  0x0F

#define AS5600_MUX_I2C &hi2c1

#define AS5600_MUX_MAX_CHANNELS 8
#define AS5600_MUX_TIMEOUT 10

#define AS5600_MUX_DMA_RUN 1
#define AS5600_MUX_DMA_STOP  0
#define AS5600_MUX_INITIALIZED 1
#define AS5600_MUX_NOT_INITIALIZED 0

typedef enum {
	AS5600_MUX_OK,
	AS5600_MUX_ERROR,
	AS5600_MUX_NONE
} AS5600_MUX_StatusTypeDef;

typedef enum {
	AS5600_MUX_DMA_IDLE,
	AS5600_MUX_DMA_READY,
	AS5600_MUX_READ_AS5600,
	AS5600_MUX_DMA_DONE
} AS5600_MUX_DMA_LoopStateTypeDef;

typedef struct {
    bool is_initialized;

	bool dma_mode_start_stop;
	volatile bool dma_busy;
	volatile uint8_t dma_current_channel;
    uint8_t dma_buffer[2];
    AS5600_MUX_DMA_LoopStateTypeDef dma_loop_state;

	uint8_t num_channels;
    uint16_t channel_raw_values[AS5600_MUX_MAX_CHANNELS];
    AS5600_MUX_StatusTypeDef channel_states[AS5600_MUX_MAX_CHANNELS];
} AS5600_MUX_HandleTypeDef;

#endif /* HW_DRIVERS_AS5600_MUX_AS5600_MUX_DEF_H_ */
