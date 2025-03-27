/*
 * AS5600_Mux.c
 *
 *  Created on: Mar 14, 2025
 *      Author: omart
 */

#include "AS5600_Mux.h"

#define TCA9548A_ADDR          0x70    // 7-bit address (0x70 = 112 decimal)
#define AS5600_ADDR            0x36    // 7-bit address (0x36 = 54 decimal)
#define I2C_TIMEOUT            100     // Timeout in ms for I2C operations

/* AS5600 Register Map - Only what we need */
#define AS5600_REG_ANGLE_H     0x0E    // Angle high byte
#define AS5600_REG_ANGLE_L     0x0F    // Angle low byte

HAL_StatusTypeDef AS5600_Mux_Init(AS5600_Mux_Array* sensors, I2C_HandleTypeDef* hi2c, uint8_t num_sensors) {
    if (hi2c == NULL || num_sensors > MAX_SENSORS) return HAL_ERROR;

    sensors->hi2c = hi2c;
    sensors->num_sensors = num_sensors;

    for (uint8_t i = 0; i < MAX_SENSORS; i++) {
        sensors->angles[i] = 0;
    }

    uint8_t disable_cmd = 0x00;
    if (HAL_I2C_Master_Transmit(sensors->hi2c, TCA9548A_ADDR << 1, &disable_cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    // Basic check if sensors are responsive
    for (uint8_t i = 0; i < num_sensors; i++) {
        // Select channel
        if (AS5600_Mux_SelectChannel(sensors, i) != HAL_OK) {
            return HAL_ERROR;
        }

        uint16_t dummy;
        if (AS5600_Mux_ReadAngle(sensors, i, &dummy) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_Mux_SelectChannel(AS5600_Mux_Array* sensors, uint8_t channel) {
    if (channel > 7) {
        return HAL_ERROR;
    }

    uint8_t channel_cmd = 1 << channel;
    if (HAL_I2C_Master_Transmit(sensors->hi2c, TCA9548A_ADDR << 1, &channel_cmd, 1, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef AS5600_Mux_ReadAngle(AS5600_Mux_Array* sensors, uint8_t sensor_index, uint16_t* angle) {
    uint8_t angle_h, angle_l;
    uint8_t reg_addr;

    if (sensor_index >= sensors->num_sensors) {
        return HAL_ERROR;
    }

    // Select the appropriate channel
    if (AS5600_Mux_SelectChannel(sensors, sensor_index) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read high byte
    reg_addr = AS5600_REG_ANGLE_H;

    if (HAL_I2C_Master_Transmit(sensors->hi2c, AS5600_ADDR << 1, &reg_addr, 1, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(sensors->hi2c, AS5600_ADDR << 1, &angle_h, 1, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    // Read low byte
    reg_addr = AS5600_REG_ANGLE_L;
    if (HAL_I2C_Master_Transmit(sensors->hi2c, AS5600_ADDR << 1, &reg_addr, 1, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(sensors->hi2c, AS5600_ADDR << 1, &angle_l, 1, I2C_TIMEOUT) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine high and low bytes
    *angle = ((uint16_t)angle_h << 8) | angle_l;

    return HAL_OK;
}

void AS5600_Mux_ReadAllAngles(AS5600_Mux_Array* sensors) {
    for (uint8_t i = 0; i < sensors->num_sensors; i++) {
        sensors->sensor_status_array[i] = AS5600_Mux_ReadAngle(sensors, i, &sensors->angles[i]);
    }
}

float AS5600_Mux_ConvertToDegrees(uint16_t raw_angle) {
    return (float)raw_angle * 360.0f / 4096.0f;
}

float AS5600_Mux_ConvertToRadians(uint16_t raw_angle) {

    return (float)raw_angle * 2.0f * 3.14159f / 4096.0f;

}
