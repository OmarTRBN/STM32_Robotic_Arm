/*
 * AS5600_Multi.h
 *
 *  Created on: Feb 23, 2025
 *      Author: omart
 */

#ifndef INC_AS5600_MULTI_H_
#define INC_AS5600_MULTI_H_

#include "main.h"
#include <stdint.h>

#define AS5600_ADDR 0x36
#define AS5600_REGISTER_ANGLE_HIGH 0x0E
#define AS5600_REGISTER_ANGLE_LOW 0x0F
#define AS5600_REGISTER_STATUS 0x0B
#define AS5600_MAGNET_DETECTED (1 << 5)

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;
} AS5600_HandleTypeDef;

int AS5600_Init(AS5600_HandleTypeDef *as5600, I2C_HandleTypeDef *hi2c);
int AS5600_ReadAngle(AS5600_HandleTypeDef *as5600, uint16_t *angle);

#endif /* INC_AS5600_MULTI_H_ */
