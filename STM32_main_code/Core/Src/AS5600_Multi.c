/*
 * AS5600_Multi.c
 *
 *  Created on: Feb 23, 2025
 *      Author: omart
 */

#include "AS5600_Multi.h"

int AS5600_Init(AS5600_HandleTypeDef *as5600, I2C_HandleTypeDef *hi2c) {
    uint8_t status = 0;

    // Setup handle
    as5600->hi2c = hi2c;
    as5600->addr = AS5600_ADDR << 1; // Shift address for HAL

    // Check if magnet is detected
    if(HAL_I2C_Mem_Read(hi2c, as5600->addr, AS5600_REGISTER_STATUS, 1, &status, 1, 100) != HAL_OK) {
        return 1;
    }

    if(!(status & AS5600_MAGNET_DETECTED)) {
        return 2;  // No magnet detected
    }

    return 0;
}
int AS5600_ReadAngle(AS5600_HandleTypeDef *as5600, uint16_t *angle) {
    uint8_t data[2];

    // Read angle registers
    if(HAL_I2C_Mem_Read(as5600->hi2c, as5600->addr, AS5600_REGISTER_ANGLE_HIGH, 1, data, 2, 100) != HAL_OK) {
        return 1;
    }

    // Combine high and low bytes
    *angle = ((uint16_t)data[0] << 8) | data[1];

    return 0;
}


