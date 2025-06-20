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
#include "app_includes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FREQ_TO_MS(freq)     (1000 / (freq))                // Hz to milliseconds
#define FREQ_TO_TICKS(freq)  pdMS_TO_TICKS(FREQ_TO_MS(freq))	// Hz to RTOS ticks
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char uart_tx_buf[128];
volatile bool uartCmdPending = false;
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
/* Definitions for DataTxTask */
osThreadId_t DataTxTaskHandle;
const osThreadAttr_t DataTxTask_attributes = {
  .name = "DataTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TrajTask */
osThreadId_t TrajTaskHandle;
const osThreadAttr_t TrajTask_attributes = {
  .name = "TrajTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for as5600 */
osMutexId_t as5600Handle;
const osMutexAttr_t as5600_attributes = {
  .name = "as5600"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* Definitions for qArraysMutex */
osMutexId_t qArraysMutexHandle;
const osMutexAttr_t qArraysMutex_attributes = {
  .name = "qArraysMutex"
};
/* Definitions for uartSem */
osSemaphoreId_t uartSemHandle;
const osSemaphoreAttr_t uartSem_attributes = {
  .name = "uartSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MyProcessCommand(SerialComm_HandleTypeDef* hserial);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ReadAs5600Task(void *argument);
void RunControllerTask(void *argument);
void DataTxFcn(void *argument);
void TrajFcn(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of as5600 */
  as5600Handle = osMutexNew(&as5600_attributes);

  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* creation of qArraysMutex */
  qArraysMutexHandle = osMutexNew(&qArraysMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uartSem */
  uartSemHandle = osSemaphoreNew(1, 1, &uartSem_attributes);

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

  /* creation of DataTxTask */
  DataTxTaskHandle = osThreadNew(DataTxFcn, NULL, &DataTxTask_attributes);

  /* creation of TrajTask */
  TrajTaskHandle = osThreadNew(TrajFcn, NULL, &TrajTask_attributes);

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
	  if (appSerialHandle.responseReadyFlag == 1) {
		  MyProcessCommand(&appSerialHandle);
		  if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK) {
			  SerialComm_Transmit(&appSerialHandle);

			  osSemaphoreAcquire(uartSemHandle, osWaitForever);
			  appSerialHandle.responseReadyFlag = 0;

			  osMutexRelease(uartMutexHandle);
		  }
	  }
	  osDelay(1);
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
	  if (osMutexAcquire(as5600Handle, osWaitForever) == osOK) {
		  appMuxStatus = AS5600_MUX_ReadAllPolling(&appMuxHandle);
		  osMutexRelease(as5600Handle);
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

	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  // Set new reference angles
	  MultivariablePID_SetSetpoint(&appPidObj, q_set);

	  // Update measurements
      osMutexAcquire(as5600Handle, osWaitForever);
      for (uint8_t i = 0; i < appMuxHandle.num_channels; i++) {
          q_meas[i] = appMuxHandle.channel_raw_values[i];
      }
      osMutexRelease(as5600Handle);

      // Compute PID Controller
      MultivariablePID_Compute(&appPidObj, q_meas);
	  for (int i=0; i<NUM_JOINTS; i++) {
		  q_out[i] = appPidObj.output_data[i];
	  }

	  // Send signals to motors
	  STEPMOTOR_MultiSetSpeed(appStepMotors, q_out, NUM_JOINTS);

	  vTaskDelayUntil(&xLastWakeTime, xPeriodTicks);
  }
  /* USER CODE END RunControllerTask */
}

/* USER CODE BEGIN Header_DataTxFcn */
/**
* @brief Function implementing the DataTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataTxFcn */
void DataTxFcn(void *argument)
{
  /* USER CODE BEGIN DataTxFcn */
//	char buf[16];
//	uint8_t len;
  /* Infinite loop */
  for(;;)
  {

//	len = sprintf(buf, "<s>%d<e>", val);
//
//	if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK) {
//	  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
//	  osSemaphoreAcquire(uartSemHandle, osWaitForever);
//	  osMutexRelease(uartMutexHandle);
//	}
    osDelay(1);
  }
  /* USER CODE END DataTxFcn */
}

/* USER CODE BEGIN Header_TrajFcn */
/**
* @brief Function implementing the TrajTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TrajFcn */
void TrajFcn(void *argument)
{
  /* USER CODE BEGIN TrajFcn */
	TickType_t xLastWakeTime;
	const TickType_t xPeriodTicks = FREQ_TO_TICKS(APP_TRAJ_FREQ);
//	char buf[16];
//	uint8_t len;
//	uint32_t i = 0; // Counter for sine wave phase
////	const float PI = 3.1415926535f; // Define PI
//	const uint16_t AMPLITUDE = 2047; // Half of 4095 (peak to peak)
//	const uint16_t OFFSET = 2048;   // Center of the sine wave (0-4095 range)

  /* Infinite loop */
  for(;;)
  {
      // Calculate sine wave value
      // The sine function returns values from -1 to 1.
      // We scale it by AMPLITUDE, then add OFFSET to shift it to 0-4095.
      // i is incremented to change the phase over time.
//      uint16_t val = (uint16_t)(AMPLITUDE * sinf(i * (2 * PI / 100.0f)) + OFFSET); // 100.0f controls the speed/frequency of the sine wave
//
//	  len = sprintf(buf, "<s>%d<e>\n", val);
//
//	  if (osMutexAcquire(uartMutexHandle, osWaitForever) == osOK) {
//
//		  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, len);
//		  osSemaphoreAcquire(uartSemHandle, osWaitForever);
//
//		  osMutexRelease(uartMutexHandle);
//	  }

	vTaskDelayUntil(&xLastWakeTime, xPeriodTicks);
  }
  /* USER CODE END TrajFcn */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void MyProcessCommand(SerialComm_HandleTypeDef* hserial) {
    uint8_t* data = hserial->pRxBuffer;
    uint16_t encodedCommand = (data[0] << 8) | data[1];

    memset(hserial->pTxBuffer, 0, hserial->txBufferSize);

    switch (encodedCommand) {
        case CMD_TEST_LED:
            Parse_CMD_TEST_LED(hserial);
            break;

        case CMD_MOTOR_STATE:
            Parse_CMD_MOTOR_STATE(hserial);
            break;

        case CMD_MOTOR_REF:
            Parse_CMD_MOTOR_REF(hserial);
            break;

		case CMD_SET_PID:
            Parse_CMD_SET_PID(hserial, &appPidObj);
            break;

        default:
            sprintf((char *)hserial->pTxBuffer, "<d>Unknown command: 0x%04X ('%c%c')\n", encodedCommand, data[0], data[1]);
            break;
    }

    memset(hserial->pRxBuffer, 0, hserial->rxBufferSize);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        osSemaphoreRelease(uartSemHandle);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == SERIALCOMM_UART) {
    	SerialComm_RxCpltCallback(&appSerialHandle);
    }
}
/* USER CODE END Application */

