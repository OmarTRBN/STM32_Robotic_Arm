/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
//#include "Timing.h"
//
//#include "Trajectory.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//char response[50];

// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
//char globalDataArray[1100];
//volatile uint8_t globalControllerFlag = 0;
//volatile HAL_StatusTypeDef statusCheck = 0;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
//CommandProtocol_Handle cmdHandle;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
//StepMotor l1_motor;
//StepMotor l2_motor;
//StepMotor* motorArray[NUM_JOINTS];
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
//float32_t q_set[NUM_JOINTS] = { 2048.0, 2048.0, 2048.0, 2048.0 };
//float32_t q_meas[NUM_JOINTS] = { 2048.0, 2048.0, 2048.0, 2048.0 };
//MultivariablePID pidObj;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
//Trajectory robotTraj;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void MyProcessCommand(SerialComm_HandleTypeDef* hserial);
//void setup_motors();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	SerialComm_SetCommandCallback(MyProcessCommand);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  /* 🥊🥊🥊 Serial Communication 🥊🥊🥊 */
  appSerialStatus = SerialComm_Init(&appSerialHandle, appSerialRxArray, appSerialTxArray);
//  HAL_UART_Receive_DMA(&huart1, &appSerialRxArray[appSerialHandle.rxIndex++], 1);
  /* 🥊🥊🥊 Encoders 🥊🥊🥊 */
  appMuxStatus = AS5600_MUX_Init(&appMuxHandle, NUM_MOTORS);

  /*🥊🥊🥊 Motors 🥊🥊🥊*/
  App_InitMotors();

  /*🥊🥊🥊 PID Control 🥊🥊🥊*/
  MultivariablePID_Init(&appPidObj);
  appPidObj.dt = 1.0f / ((float) APP_CONTROLLER_FREQ);

//  Trajectory_Init(&robotTraj);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(globalControllerFlag)
//	  {
//		  globalControllerFlag = 0;
//
//		  float controller_dt = DWT_GetDeltaTime();
//		  pidObj.dt = (float32_t)controller_dt;
//
//		  Trajectory_Compute(&robotTraj, (float32_t)controller_dt);
//
//		  MultivariablePID_SetSetpoint(&pidObj, q_set);
//		  MultivariablePID_Compute(&pidObj, q_meas);
//
//		  StepMotor_SetSpeedLUT(&l1_motor, pidObj.output_data[0]);
//		  StepMotor_SetSpeedLUT(&l2_motor, pidObj.output_data[1]);
//	  }
//
//	  if (HAL_GetTick() - lastTime > interval)
//	  {
//		  AS5600_Mux_ReadAllAngles(&sensors);
//		  for (uint8_t i = 0; i < sensors.num_sensors; i++) {
//			  q_meas[i] = sensors.angles[i];
//		  }
//
//		  lastTime = HAL_GetTick();
//	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t ParsePIDParametersFromUART(MultivariablePID *pid, char *uart_str, uint16_t len) {
    if (pid == NULL || uart_str == NULL || len == 0) return 0;

    // Make sure the string is null-terminated
    if (uart_str[len-1] != '\0') {
        if (len >= SERIALCOMM_RX_BUFF_SIZE) {
            // String too long, can't safely null-terminate
            return 0;
        }
        uart_str[len] = '\0';
    }

    float32_t parsed_values[NUM_JOINTS*NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS*NUM_JOINTS; i++) {
        parsed_values[i] = 0.0f;
    }

    // Determine which parameter is being updated
    uint16_t chosen_param;
    char *data_start = NULL;

    if (strncmp(uart_str, "KP", 2) == 0) {
        chosen_param = CMD_SET_KP;
    }
    else if (strncmp(uart_str, "KI", 2) == 0) {
        chosen_param = CMD_SET_KI;
    }
    else if (strncmp(uart_str, "KD", 2) == 0) {
        chosen_param = CMD_SET_KD;
    }
    else {
        // Unrecognized parameter
        return 0;
    }
    data_start = uart_str + 2;

    // Parse the comma-separated values
    char *token;
    char *rest = data_start;
    int index = 0;

    while ((token = strtok_r(rest, ",", &rest)) != NULL && index < NUM_JOINTS*NUM_JOINTS) {
        // Convert the token to float
        parsed_values[index] = (float32_t)atof(token);
        index++;
    }

    // Check if we received the expected number of values
    if (index != NUM_JOINTS*NUM_JOINTS) {
        // Invalid number of parameters
        return 0;
    }

    // Update the PID parameters
    MultivariablePID_SetParameter(pid, parsed_values, chosen_param);

    return 1;
}
//		case CMD_SET_PARAM:
//			char paramType[3] = {hserial->pRxBuffer[2], hserial->pRxBuffer[3], '\0'};
//			char *recievedShit = (char *)&hserial->pRxBuffer[2];
//
//			if (ParsePIDParametersFromUART(&appPidObj, recievedShit, strlen(recievedShit))) {
//				sprintf((char *)hserial->pTxBuffer, "PID %s parameters updated successfully.\n", paramType);
//			}
//			else {
//				sprintf((char *)hserial->pTxBuffer, "Error: Failed to parse PID %s parameters!\n", paramType);
//			}
//			break;
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
