/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_FREQ_HZ  200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FREQ_TO_MS(freq)     (1000 / (freq))                // Hz to milliseconds
#define FREQ_TO_TICKS(freq)  pdMS_TO_TICKS(FREQ_TO_MS(freq))	// Hz to RTOS ticks
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for as5600Task */
osThreadId_t as5600TaskHandle;
const osThreadAttr_t as5600Task_attributes = {
  .name = "as5600Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for controllerTask */
osThreadId_t controllerTaskHandle;
const osThreadAttr_t controllerTask_attributes = {
  .name = "controllerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ReadAs5600Task(void *argument);
void RunControllerTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of as5600Task */
  as5600TaskHandle = osThreadNew(ReadAs5600Task, NULL, &as5600Task_attributes);

  /* creation of controllerTask */
  controllerTaskHandle = osThreadNew(RunControllerTask, NULL, &controllerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  // HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ReadAs5600Task */
/**
* @brief Function implementing the as5600Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReadAs5600Task */
void ReadAs5600Task(void *argument)
{
  /* USER CODE BEGIN ReadAs5600Task */
	TickType_t xLastWakeTime;
	const TickType_t xPeriodTicks = FREQ_TO_TICKS(APP_ENCODER_FREQ);

	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  appMuxStatus = AS5600_MUX_ReadAllPolling(&appMuxHandle);
	  for (uint8_t i = 0; i < appMuxHandle.num_channels; i++) {
		  q_meas[i] = appMuxHandle.channel_raw_values[i];
	  }

	  vTaskDelayUntil(&xLastWakeTime, xPeriodTicks);
  }
  /* USER CODE END ReadAs5600Task */
}

/* USER CODE BEGIN Header_RunControllerTask */
/**
* @brief Function implementing the controllerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunControllerTask */
void RunControllerTask(void *argument)
{
  /* USER CODE BEGIN RunControllerTask */
	TickType_t xLastWakeTime;
	const TickType_t xPeriodTicks = FREQ_TO_TICKS(APP_CONTROLLER_FREQ);
	appPidObj.dt = 1.0f / ((float) APP_CONTROLLER_FREQ);

	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  MultivariablePID_SetSetpoint(&appPidObj, q_set);
	  MultivariablePID_Compute(&appPidObj, q_meas);

	  for (int i=0; i<NUM_MOTORS; i++) {
		  q_out[i] = appPidObj.output_data[i];
	  }

	  STEPMOTOR_MultiSetSpeed(appStepMotors, q_out, NUM_MOTORS);
	  vTaskDelayUntil(&xLastWakeTime, xPeriodTicks);
  }
  /* USER CODE END RunControllerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

