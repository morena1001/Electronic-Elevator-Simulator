/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct request {
	uint8_t requested; // 0 if a floor is not requested. 1 or more if requested 1 or more times
	uint8_t floor_number; // Stores the floor number of a given request.
	char direction; // Direction of the request. Can have values 'n' for no direction, 'u' for up, 'd' for down
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// htim2 : Controls how long an elevator has until it is deemed empty
// htim6 : Controls the movement of the elevator between floors
// htim15 : Controls the BCDI LED that shows that a destination can be chosen from inside the elevator

char msg[40]; // Used to send messages regarding elevator status back to computer to be displayed in terminal
struct request floor_requests[4]; // When a floor makes a request, the requested and direction values for a floor's corresponding element are updated
char elevator_direction; // Stores direction of the elevator. Can have values 'n' for no direction, 'u' for up, and 'd' for down
uint8_t floor_position; // Stores position of the elevator
uint8_t floor_destinations[2]; // Stores the destinations of the elevator (max is 2). Can have values '0' for no floor, or '1', '2', or '3' for the floor of the destination
struct request* requests[4]; // Stores the incoming requests (like a queue). Can have values '0' for no requests, or '1', '2', or '3' for the floor of the request
uint8_t pin_num; // Used to differentiate between OI2U_Pin and OI3D_Pin in the EXTI_9_5_IRQ
struct request* serving; // Stores the floor number that the elevator is currently serving. Can have values '0' for no floor, or '1', '2', or '3' for the floors that are currently being served
uint8_t start_timer; // Indicates that the htim2 has fully ran once
uint8_t traveling; // Boolean variable that used for case 2 and 4 (Where the first floor that requests the elevator is another floor)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Program_Start_Light_Up(void); // An LED light up sequence that makes sure all the LED in the system work
void Elevator_Simulator_Init(void); // Initializes all the variables to their default values
void Choose_Floor_To_Service(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  Program_Start_Light_Up();
  Elevator_Simulator_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if (requests[0] != 0 && (!serving || (serving && floor_position == requests[0]))) {
//		  serving = requests[0];
//		  for (int i = 0; i < 2; i++) {
//			  requests[i] = requests[i + 1];
//		  }
//		  requests[2] = 0;
////		  elevator_direction = floor_position < 1 ? 'u' : 'd';
//
//
//		  // CASE 1
//		  if (floor_position == serving/*requests[0]*/ /*&& requests[1] == 0*/) {
//			  switch(serving) {
//			  case 1:
//				  elevator_direction = 'u';
//				  HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin, GPIO_PIN_RESET); // Assign a value to elevator_direction based on the .direction value
//				  break;
//			  case 2:
//				  elevator_direction = floor_requests[1].direction;
//				  HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_RESET);
//				  break;
//			  case 3:
//				  elevator_direction = 'd';
//				  HAL_GPIO_WritePin(FR3_GPIO_Port, FR3_Pin, GPIO_PIN_RESET);
//				  break;
//			  }
//			  HAL_TIM_Base_Start_IT(&htim15);
//			  sprintf(msg, "%c\r\n", elevator_direction);
//			  HAL_UART_Transmit(&huart2, (uint8_t*) msg, 3, 100);
//		  }
//
//		  // CASE 2
//		  else if (!traveling && floor_position != serving) {
//			  traveling = 1;
//			  elevator_direction = floor_position > serving ? 'd' : 'u';
//			  HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_SET);
//			  HAL_TIM_Base_Start_IT(&htim6);
//		  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM15;
  PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9995;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 3999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 111;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PF1_Pin|PF2_Pin|PF3_Pin|DF1_Pin
                          |DF2_Pin|DF3_Pin|DU_Pin|DD_Pin
                          |FR1_Pin|FR2_Pin|FR3_Pin|BCDI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF1_Pin PF2_Pin PF3_Pin DF1_Pin
                           DF2_Pin DF3_Pin DU_Pin DD_Pin
                           FR1_Pin FR2_Pin FR3_Pin BCDI_Pin */
  GPIO_InitStruct.Pin = PF1_Pin|PF2_Pin|PF3_Pin|DF1_Pin
                          |DF2_Pin|DF3_Pin|DU_Pin|DD_Pin
                          |FR1_Pin|FR2_Pin|FR3_Pin|BCDI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : II1_Pin II2_Pin II3_Pin OI1U_Pin
                           OI2D_Pin OI2U_Pin OI3D_Pin */
  GPIO_InitStruct.Pin = II1_Pin|II2_Pin|II3_Pin|OI1U_Pin
                          |OI2D_Pin|OI2U_Pin|OI3D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM2) {
////		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//		for (int i = 0; i < 20; i++) {
//			msg[i] = '\0';
//		}
//		sprintf(msg, "YIPPEE\r\n");
//		HAL_UART_Transmit(&huart2, (uint8_t*) msg, 10, 100);
//	}
//}

void Elevator_Simulator_Init(void) {
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

	// Floor request corresponding to floor 1 in an upward direction
	floor_requests[0].requested = 0;
	floor_requests[0].floor_number = 1;
	floor_requests[0].direction = 'u';

	// Floor request corresponding to floor 2 in a downward direction
	floor_requests[1].requested = 0;
	floor_requests[1].floor_number = 2;
	floor_requests[1].direction = 'd';

	// Floor request corresponding to floor 2 in an upward direction
	floor_requests[2].requested = 0;
	floor_requests[2].floor_number = 2;
	floor_requests[2].direction = 'u';

	// Floor request corresponding to floor 3 in a downward direction
	floor_requests[3].requested = 0;
	floor_requests[3].floor_number = 3;
	floor_requests[3].direction = 'd';

	elevator_direction = 'n';
	floor_position = 1; // The elevator always begins in the first floor

	for (int i = 0; i < 2; i++) {
		floor_destinations[i] = 0;
	}

	for (int i =  0; i < 3; i++) {
			requests[i] = NULL;
		}

	pin_num = 0;
	serving = NULL;
	start_timer = 0;
	traveling = 0;

	HAL_GPIO_WritePin(PF1_GPIO_Port, PF1_Pin, GPIO_PIN_SET);
}

void Program_Start_Light_Up(void) {
	HAL_GPIO_TogglePin(PF1_GPIO_Port, PF1_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(PF2_GPIO_Port, PF2_Pin);
	HAL_GPIO_TogglePin(PF1_GPIO_Port, PF1_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(PF3_GPIO_Port, PF3_Pin);
	HAL_GPIO_TogglePin(PF2_GPIO_Port, PF2_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(DF1_GPIO_Port, DF1_Pin);
	HAL_GPIO_TogglePin(PF3_GPIO_Port, PF3_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(DF2_GPIO_Port, DF2_Pin);
	HAL_GPIO_TogglePin(DF1_GPIO_Port, DF1_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(DF3_GPIO_Port, DF3_Pin);
	HAL_GPIO_TogglePin(DF2_GPIO_Port, DF2_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(DU_GPIO_Port, DU_Pin);
	HAL_GPIO_TogglePin(DF3_GPIO_Port, DF3_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(DD_GPIO_Port, DD_Pin);
	HAL_GPIO_TogglePin(DU_GPIO_Port, DU_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(FR1_GPIO_Port, FR1_Pin);
	HAL_GPIO_TogglePin(DD_GPIO_Port, DD_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(FR2_GPIO_Port, FR2_Pin);
	HAL_GPIO_TogglePin(FR1_GPIO_Port, FR1_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(FR3_GPIO_Port, FR3_Pin);
	HAL_GPIO_TogglePin(FR2_GPIO_Port, FR2_Pin);
	HAL_Delay(250);

	HAL_GPIO_TogglePin(FR3_GPIO_Port, FR3_Pin);
	HAL_Delay(1000);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == OI2U_Pin) {
		pin_num = 2;
	} else if (GPIO_Pin == OI3D_Pin) {
		pin_num = 3;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	// IF htim2 has fully run once, it will check if there are any destination
	if (htim->Instance == TIM2 && start_timer == 1) {
		// IF no destinations, the elevator has finished serving the floor in the serving variable
		if (!floor_destinations[0]) {
			// IF there are no other requests, stop moving the elevator
//			if (!requests[0] && !serving) {
			if (!requests[0]) {
				sprintf(msg, "NOTHING TO DO\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, 15, 100);
			}

			// reset the serving variable and the LED corresponding to the elevator_direction variable
			serving->requested = 0;
			serving = NULL;
			Choose_Floor_To_Service();
			HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_RESET);


		} // ELSE IF only one destination, begin moving elevator
		  else if (floor_destinations[1] == 0) {
		    elevator_direction = floor_destinations[0] < floor_position ? 'd' : 'u';
		    HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DD_Pin : DU_Pin, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_SET);
		    HAL_TIM_Base_Start_IT(&htim6);

		} // ELSE (two destinations), choose closest one, and move to that floor
		  else {
			HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_SET);
			// IF second destination chosen is closer to elevator, swap values to let the closest value be the first destination
			if (floor_destinations[0] - floor_position > floor_destinations[1] - floor_position) {
				uint8_t temp = floor_destinations[0];
				floor_destinations[0] = floor_destinations[1];
				floor_destinations[1] = temp;
			}
			elevator_direction = floor_destinations[0] < floor_position ? 'd' : 'u';
			HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DD_Pin : DU_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_SET);
			HAL_TIM_Base_Start_IT(&htim6);
		}
	}
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
