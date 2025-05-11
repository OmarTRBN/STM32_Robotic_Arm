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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CommandProtocol.h"
#include "StepMotor.h"
#include "Timing.h"
#include "AS5600_Mux.h"
#include "lut.h"

#include "PID_Control.h"
#include "Trajectory.h"
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
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
char globalDataArray[1100];
volatile uint8_t globalControllerFlag = 0;
volatile HAL_StatusTypeDef statusCheck = 0;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
CommandProtocol_Handle cmdHandle;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
StepMotor l1_motor;
StepMotor l2_motor;
StepMotor* motorArray[NUM_JOINTS];
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
float32_t q_set[NUM_JOINTS] = { 2048.0, 2048.0, 2048.0, 2048.0 };
float32_t q_meas[NUM_JOINTS] = { 2048.0, 2048.0, 2048.0, 2048.0 };
MultivariablePID pidObj;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
Trajectory robotTraj;
// 🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊🥊
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MyProcessCommand(CommandProtocol_Handle* handle);
void setup_motors();
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
  CommandProtocol_SetCommandProcessor(MyProcessCommand);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  statusCheck = CommandProtocol_Init(&cmdHandle, &huart1, 100);

  setup_motors();

  MultivariablePID_Init(&pidObj);

  Trajectory_Init(&robotTraj);

//  volatile uint32_t lastTime = 0; uint32_t interval = 4;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  HAL_TIM_Base_Start_IT(&htim11); // Start controller timer
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
void MyProcessCommand(CommandProtocol_Handle* handle) {
	// Combine the first two characters into a 16-bit integer
	uint16_t encodedCommand = (handle->rxBuffer[0] << 8) | handle->rxBuffer[1];
    char response[50];

    switch(encodedCommand) { // First 2 bytes are command
        case CMD_TEST_LED:
            HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
            CommandProtocol_SendResponse(handle, "LED TOGGLED!\n");
            break;

        case CMD_STEP_MOTOR_STATE:
			int index = handle->rxBuffer[2] - '0'; // Convert char to int by subtracting '0'
			char state = handle->rxBuffer[3];

			if (state == '1') {
				StepMotor_Enable(motorArray[index]);
			}
			else {
				StepMotor_Disable(motorArray[index]);
			}
			sprintf(response, "Motor %d is at state %c\n", index, state);
			CommandProtocol_SendResponse(handle, response);
			break;

        case CMD_SET_PARAM:
			char paramType[3] = {handle->rxBuffer[2], handle->rxBuffer[3], '\0'};
			char *recievedShit = (char *)&handle->rxBuffer[2];

			if (ParsePIDParametersFromUART(&pidObj, recievedShit, strlen(recievedShit))) {
				sprintf(response, "PID %s parameters updated successfully.\n", paramType);
			}
			else {
				sprintf(response, "Error: Failed to parse PID %s parameters!\n", paramType);
			}
			CommandProtocol_SendResponse(handle, response);
			break;

        case CMD_SET_TRAJ_COEFF:
        	if (Trajectory_ParseCoeffs((char*)handle->rxBuffer, &robotTraj) == HAL_OK)
        	{
				CommandProtocol_SendResponse(handle, "Trajectory coefficients received.\n");
			}
        	else
        	{
				CommandProtocol_SendResponse(handle, "Error parsing trajectory data.\n");
			}
        	break;

        case CMD_BEGIN_TRAJ:
        	Trajectory_Start(&robotTraj);
        	sprintf(response, "Trajectory started.\n");
			CommandProtocol_SendResponse(handle, response);
        	break;

//        case CMD_AS5600_DATA:
////            sprintf(response, "AS5600 Angles: %d, %d\n", sensors.angles[0], sensors.angles[1]);
//            CommandProtocol_SendResponse(handle, response);
//            break;

        default:
            sprintf(response, "Unknown command: %d\n", encodedCommand);
        	CommandProtocol_SendResponse(handle, response);
        	break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == cmdHandle.huart) {
		uint8_t receivedByte = cmdHandle.rxBuffer[cmdHandle.rxIndex];
		CommandProtocol_ProcessByte(&cmdHandle, receivedByte);
		HAL_UART_Receive_IT(huart, &cmdHandle.rxBuffer[cmdHandle.rxIndex], 1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM11)
    {
    	globalControllerFlag = 1;
    }
}

void setup_motors() {
    // Initialize individual motors as you're already doing
    statusCheck = StepMotor_Init(&l1_motor, &htim5, TIM_CHANNEL_1, M1_DIR_GPIO_Port, M1_DIR_Pin, M1_EN_GPIO_Port, M1_EN_Pin);
    statusCheck = StepMotor_Init(&l2_motor, &htim9, TIM_CHANNEL_1, M2_DIR_GPIO_Port, M2_DIR_Pin, M2_EN_GPIO_Port, M2_EN_Pin);

    // Set up the motor array
    motorArray[0] = &l1_motor;
    motorArray[1] = &l2_motor;

    StepMotor_SetSpeedLUT(&l1_motor, 0); // Set motor speed to 0 Initially
    StepMotor_SetSpeedLUT(&l2_motor, 0); // Set motor speed to 0 Initially
}
/* USER CODE END 4 */

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
