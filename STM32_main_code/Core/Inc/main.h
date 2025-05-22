/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "arm_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define CMD_SET_TRAJ_COEFF      ( ('T'<<8) | 'C') // "TC" Trajectory Coefficients
#define CMD_BEGIN_TRAJ          ( ('B'<<8) | 'T') // "BT" Begin Trajectory



#define CMD_AS5600_DATA    ( ('A'<<8) | 'B') // "AB"

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define AS5600_MAX 4096.0        // 12-bit AS5600 range (0-4095)
#define RAD_TO_AS5600 (AS5600_MAX / 2.0 * M_PI)
#define DEG_TO_AS5600 (AS5600_MAX / 360.0)
#define	AS5600_TO_DEG (360.0 / AS5600_MAX)
#define AS5600_TO_RAD (2.0 * M_PI / AS5600_MAX)
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define NUM_JOINTS 4
#define NUM_JOINTS_TRAJ 2
#define TRAJ_POLY_TERMS 6
#define TRAJ_COEFF_LEN NUM_JOINTS_TRAJ*TRAJ_POLY_TERMS
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TEST_LED_Pin GPIO_PIN_13
#define TEST_LED_GPIO_Port GPIOC
#define M1_DIR_Pin GPIO_PIN_14
#define M1_DIR_GPIO_Port GPIOC
#define M1_EN_Pin GPIO_PIN_15
#define M1_EN_GPIO_Port GPIOC
#define M2_DIR_Pin GPIO_PIN_1
#define M2_DIR_GPIO_Port GPIOA
#define M2_EN_Pin GPIO_PIN_3
#define M2_EN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
