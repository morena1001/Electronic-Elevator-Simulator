/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
struct request {
	uint8_t requested; // Boolean that is 1 if request received, else 0
	char direction; // Direction of the request
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Reset_Timer(int num, ...);
void Choose_Floor_To_Service(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
/* USER CODE BEGIN EV */
// ALL THE EXTERN VARIABLES are first declared and are initialized in the 'main.c' file
extern struct request floor_requests[3];
extern char elevator_direction;
extern uint8_t floor_position;
extern uint8_t floor_destinations[2];
extern uint8_t requests[3];
extern uint8_t pin_num;
extern uint8_t serving;
extern uint8_t start_timer;
extern uint8_t traveling;

uint8_t count = 0; // Controls the blinking pattern of the BCDI LED for htim15
uint8_t elevator_moving_counter = 0; // Controls the elevator movement for htim6
char msgs[50]; // Used to send messages regarding elevator status back to computer to be displayed in terminal
extern UART_HandleTypeDef huart2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	// FUNCTION SETS FLOOR 1 AS A DESTINATION

	// Simple software debouncing
	for (int i = 0; i < 65535; i++);
	// IF button for choosing floor 1 as a destination is pressed
	if(HAL_GPIO_ReadPin(II1_GPIO_Port, II1_Pin)) {
		// IF the current floor_position is not 1 and elevator is not empty (currently serving an elevator) and elevator not traveling to a floor to serve it
		if (floor_position != 1 && serving && !traveling) {
			// IF no destination has been chosen yet, put the corresponding value in the first element of the array
			if (!floor_destinations[0]) {
				floor_destinations[0] = 1;

				HAL_GPIO_WritePin(DF1_GPIO_Port, DF1_Pin, GPIO_PIN_SET);

				// IF there are no more floor requests, begin timer to countdown time left to choose destinations
				if (!requests[0])	Reset_Timer(1, &htim2); // htim2 is reset every time a the first destination is set to give more time to choose another destination
			} // ELSE IF the previously chosen destination is not the same as this new destination
			  // (There are only at most 2 possible floors to set as a destination, overwriting can never occur)
			  else if (floor_destinations[0] != 1 /* !floor_destinations[1] */) {
				floor_destinations[1] = 1;

				HAL_GPIO_WritePin(DF1_GPIO_Port, DF1_Pin, GPIO_PIN_SET);
			} // ELSE (the chosen floor could not be set as a destination)
//			  else {
//				return;
//			}
		}
	}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(II1_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	// FUNCTION SETS FLOOR 2 AS A DESTINATION

	// Simple software debouncing
	for (int i = 0; i < 65535; i++);
	// IF button for choosing floor 2 as a destination is pressed
	if(HAL_GPIO_ReadPin(II2_GPIO_Port, II2_Pin)) {
		// IF the current floor_position is not 2 and elevator is not empty (currently serving an elevator) and elevator not traveling to a floor to serve it
		if (floor_position != 2 && serving && !traveling) {
			// IF no destination has been chosen yet, put the corresponding value in the first element of the array
			if (!floor_destinations[0]) {
				floor_destinations[0] = 2;

				HAL_GPIO_WritePin(DF2_GPIO_Port, DF2_Pin, GPIO_PIN_SET);

				// IF there are no more floor requests, begin timer to countdown time left to choose destinations
				if (!requests[0])	Reset_Timer(1, &htim2); // htim2 is reset every time a the first destination is set to give more time to choose another destination
			} // ELSE IF the previously chosen destination is not the same as this new destination
			  else if (floor_destinations[0] != 2){
				floor_destinations[1] = 2;

				HAL_GPIO_WritePin(DF2_GPIO_Port, DF2_Pin, GPIO_PIN_SET);
			} // ELSE (the chosen floor could not be set as a destination)
//			  else {
//				  return;
//			  }
		}
	}
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(II2_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 and Touch Sense controller.
  */
void EXTI2_TSC_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_TSC_IRQn 0 */
	// FUNCTION SET FLOOR 3 AS A DESTINATION

	// Simple software debouncing
	for (int i = 0; i < 65535; i++);
	// IF button for choosing floor 3 as a destination is pressed
	if(HAL_GPIO_ReadPin(II3_GPIO_Port, II3_Pin)) {
		// IF the current floor_position is not 2 and elevator is not empty (currently serving an elevator) and elevator not traveling to a floor to serve it
		if (floor_position != 3 && serving && !traveling) {
			// IF no destination has been chosen yet, put the corresponding value in the first element of the array
			if (!floor_destinations[0]) {
				floor_destinations[0] = 3;

				HAL_GPIO_WritePin(DF3_GPIO_Port, DF3_Pin, GPIO_PIN_SET);

				// IF there are no more floor requests, begin timer to countdown time left to choose destinations
				if (!requests[0])	Reset_Timer(1, &htim2); // htim2 is reset every time a the first destination is set to give more time to choose another destination
			} // ELSE IF the previously chosen destination is not the same as this new destination
			  else if (floor_destinations[0] != 3) {
				floor_destinations[1] = 3;

				HAL_GPIO_WritePin(DF3_GPIO_Port, DF3_Pin, GPIO_PIN_SET);
			} // ELSE (the chosen floor could not be set as a destination)
//			  else {
//				  return;
//			  }
		}
	}
  /* USER CODE END EXTI2_TSC_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(II3_Pin);
  /* USER CODE BEGIN EXTI2_TSC_IRQn 1 */

  /* USER CODE END EXTI2_TSC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	// FUNCTION REQUESTS FLOOR 1

	for (int i = 0; i < 65535; i++);
	// IF button for choosing floor 1 as a request is pressed
	if(HAL_GPIO_ReadPin(OI1U_GPIO_Port, OI1U_Pin)) {
		uint8_t i = 0;
		while (requests[i]) i++;

		// IF the floor being serviced is the same the floor being requested
		if (i == 0 && serving == 1) {
			if (floor_position != serving) {
				floor_requests[0].requested = 1;
				floor_requests[0].direction = 'u';
				requests[i] = 1;
				Reset_Timer(1, &htim2);
				HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin, GPIO_PIN_SET);
			}
		} // ELSE IF the floor is not currently being requested or if it is being served, AND if the most recent requested floor is not the same as this floor
		  else if ((!floor_requests[0].requested || serving == 1) && (!(i == 0 && serving == 1) && !(i > 0 && requests[i - 1] == 1))) {
			floor_requests[0].requested = 1;
			floor_requests[0].direction = 'u';
			requests[i] = 1;

			// IF the elevator is already serving a floor but no destination has been set yet, start the timer so that the new floor request can have a chance to be serviced
			if (serving && !floor_destinations[0]) {
				Reset_Timer(1, &htim2);
			}

			HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin, GPIO_PIN_SET);
			Choose_Floor_To_Service();
		}
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(OI1U_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	// FUNCTION REQUESTS FLOOR 2

	for (int i = 0; i < 65535; i++);
	// IF button for choosing floor 2 as a request is pressed
	if(HAL_GPIO_ReadPin(OI2D_GPIO_Port, OI2D_Pin)) {
		uint8_t i = 0;
		while (requests[i]) i++;

		// IF the floor being serviced is the same the floor being requested
		if (i == 0 && serving == 2) {
			if (floor_position != serving) {
				floor_requests[1].requested = 1;
				floor_requests[1].direction = 'd';
				requests[i] = 2;
				Reset_Timer(1, &htim2);
				HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_SET);
			}
		} // ELSE IF the floor is not currently being requested or if it is being served, AND if the most recent requested floor is not the same as this floor
		  else if ((!floor_requests[1].requested || serving == 2) && (!(i == 0 && serving == 2) && !(i > 0 && requests[i - 1] == 2))) {
			floor_requests[1].requested = 1;
			floor_requests[1].direction = 'd';
			requests[i] = 2;

			// IF the elevator is already serving a floor but no destination has been set yet, start the timer so that the new floor request can have a chance to be serviced
			if (serving && !floor_destinations[0]) {
				Reset_Timer(1, &htim2);
			}

			HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_SET);
			Choose_Floor_To_Service();
		}
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(OI2D_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(OI2U_Pin);
  HAL_GPIO_EXTI_IRQHandler(OI3D_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  // FUNCTION REQUESTS FLOOR 2 AND FLOOR 3

  // IF button for choosing floor 2 is pressed
    if(pin_num == 2) {
	    uint8_t i = 0;
		while (requests[i]) i++;

		// IF the floor being serviced is the same the floor being requested
		if (i == 0 && serving == 2) {
			if (floor_position != serving) {
				floor_requests[1].requested = 1;
				floor_requests[1].direction = 'u';
				requests[i] = 2;
				Reset_Timer(1, &htim2);
				HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_SET);
			}
		} // ELSE IF the floor is not currently being requested or if it is being served, AND if the most recent requested floor is not the same as this floor
		  else if ((!floor_requests[1].requested || serving == 2) && (!(i == 0 && serving == 2) && !(i > 0 && requests[i - 1] == 2))) {
			floor_requests[1].requested = 1;
			floor_requests[1].direction = 'u';
			requests[i] = 2;

			// IF the elevator is already serving a floor but no destination has been set yet, start the timer so that the new floor request can have a chance to be serviced
			if (serving && !floor_destinations[0]) {
				Reset_Timer(1, &htim2);
			}

			HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_SET);
			Choose_Floor_To_Service();
		}
	} // ELSE IF button for choosing floor 3 is pressed
      else if (pin_num == 3) {
		uint8_t i = 0;
		while (requests[i]) i++;

		// IF the floor being serviced is the same the floor being requested
		if (i == 0 && serving == 3) {
			if (floor_position != serving) {
				floor_requests[2].requested = 1;
				floor_requests[2].direction = 'u';
				requests[i] = 3;
				Reset_Timer(1, &htim2);
				HAL_GPIO_WritePin(FR3_GPIO_Port, FR3_Pin, GPIO_PIN_SET);
			}
		} // ELSE IF the floor is not currently being requested or if it is being served, AND if the most recent requested floor is not the same as this floor
		if ((!floor_requests[2].requested || serving == 3) && (!(i == 0 && serving == 3) && !(i > 0 && requests[i - 1] == 3))) {
			floor_requests[2].requested = 1;
			floor_requests[2].direction = 'u';
			requests[i] = 3;

			// IF the elevator is already serving a floor but no destination has been set yet, start the timer so that the new floor request can have a chance to be serviced
			if (serving && !floor_destinations[0]) {
				Reset_Timer(1, &htim2);
			}

			HAL_GPIO_WritePin(FR3_GPIO_Port, FR3_Pin, GPIO_PIN_SET);
			Choose_Floor_To_Service();
		}
	}
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break and TIM15 interrupts.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */
	count++;
	if (count == 15 || (count < 10 && count % 2 == 1)) {
		HAL_GPIO_TogglePin(BCDI_GPIO_Port, BCDI_Pin);
	}

	if (count == 18) 	count = 0;
  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  if (start_timer) {
  		start_timer = 0;
  		HAL_TIM_Base_Stop_IT(&htim2);
  		HAL_TIM_Base_Stop_IT(&htim15);
  		HAL_GPIO_WritePin(BCDI_GPIO_Port, BCDI_Pin, GPIO_PIN_RESET);
  	} else {
  		start_timer = 1;
  		sprintf(msgs, "HURRY UP TIMER STARTING\r\n");
  		HAL_UART_Transmit(&huart2, (uint8_t*) msgs, 25, 100);
  	}
  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
//	for (int i = 0; i < 65535; i++);
//	if (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
//		on = 1;
//	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	if (elevator_moving_counter < 3) {
//		if (elevator_direction == 'u') {
//			switch(floor_position + 1) {
//			case 2:
//				HAL_GPIO_TogglePin(PF2_GPIO_Port, PF2_Pin);
//				break;
//			case 3:
//				HAL_GPIO_TogglePin(PF3_GPIO_Port, PF3_Pin);
//				break;
//			}
//		} else {
//			switch(floor_position - 1) {
//			case 1:
//				HAL_GPIO_TogglePin(PF1_GPIO_Port, PF1_Pin);
//				break;
//			case 2:
//				HAL_GPIO_TogglePin(PF1_GPIO_Port, PF1_Pin);
//				break;
//			}
//		}
		sprintf(msgs, "MOVING TO FLOOR %d FROM FLOOR %d\r\n", elevator_direction == 'u' ? floor_position + 1 : floor_position - 1, floor_position);
		HAL_UART_Transmit(&huart2, (uint8_t*) msgs, 32, 100);
		elevator_moving_counter++;
	} else {
		switch (floor_position) {
		case 1:
			HAL_GPIO_WritePin(PF1_GPIO_Port, PF1_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(PF2_GPIO_Port, PF2_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(PF3_GPIO_Port, PF3_Pin, GPIO_PIN_RESET);
			break;
		}
		if (elevator_direction == 'd') {
			floor_position--;
		} else {
			floor_position++;
		}
		switch (floor_position) {
		case 1:
			HAL_GPIO_WritePin(PF1_GPIO_Port, PF1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(PF2_GPIO_Port, PF2_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(PF3_GPIO_Port, PF3_Pin, GPIO_PIN_SET);
			break;
		}


		elevator_moving_counter = 0;
		sprintf(msgs, "ELEVATOR AT FLOOR %d\r\n", floor_position);
		HAL_UART_Transmit(&huart2, (uint8_t*) msgs, 21, 100);

		if (floor_position == floor_destinations[0]) { // IF ELEVATOR HAS REACHED FIRST DESTINATION
			sprintf(msgs, "DESTINATION AT FLOOR %d REACHED\r\n", floor_position);
			HAL_UART_Transmit(&huart2, (uint8_t*) msgs, 32, 100);

			switch(floor_position) {
			case 1:
				HAL_GPIO_WritePin(DF1_GPIO_Port, DF1_Pin, GPIO_PIN_RESET);
				break;
			case 2:
				HAL_GPIO_WritePin(DF2_GPIO_Port, DF2_Pin, GPIO_PIN_RESET);
				break;
			case 3:
				HAL_GPIO_WritePin(DF3_GPIO_Port, DF3_Pin, GPIO_PIN_RESET);
				break;
			}

			HAL_GPIO_WritePin(elevator_direction == 'u' ? DU_GPIO_Port : DD_GPIO_Port, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_RESET);
			if (floor_destinations[1] == 0) { // IF THERE ARE NO OTHER FLOOR DESTINATION REQUESTS
				floor_destinations[0] = 0;

			} else { // IF THERE IS ANOTHER FLOOR DESTINATION REQUEST
				floor_destinations[0] = floor_destinations[1];
				floor_destinations[1] = 0;

				// CHECK FOR ANY INPUTS INSIDE ELEVATOR, ONLY ALLOW TO GO IN THE CURRENT DIRECTION
			}
//			HAL_TIM_Base_Stop_IT(&htim2);
//			start_timer = 0;
//			if (requests[0] || floor_destinations[0])		HAL_TIM_Base_Start_IT(&htim2);
			if (requests[0] || floor_destinations[0])	Reset_Timer(1, &htim2);
			count = 0;
			HAL_TIM_Base_Start_IT(&htim15);
			HAL_TIM_Base_Stop_IT(&htim6);

		} else if (floor_position == serving && traveling) {
			sprintf(msgs, "REQUEST AT FLOOR %d REACHED\r\n", floor_position);
			HAL_UART_Transmit(&huart2, (uint8_t*) msgs, 28, 100);

			HAL_GPIO_WritePin(elevator_direction == 'u' ? DU_GPIO_Port : DD_GPIO_Port, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_RESET);
			traveling = 0;
			switch(serving) {
			  case 1:
				  elevator_direction = 'u';
				  HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin, GPIO_PIN_RESET);
				  break;
			  case 2:
				  elevator_direction = floor_requests[1].direction;
				  HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_RESET);
				  break;
			  case 3:
				  elevator_direction = 'd';
				  HAL_GPIO_WritePin(FR3_GPIO_Port, FR3_Pin, GPIO_PIN_RESET);
				  break;
			  }
			HAL_TIM_Base_Stop_IT(&htim2);
			start_timer = 0;
			if (requests[0])		HAL_TIM_Base_Start_IT(&htim2);
			count = 0;
			HAL_TIM_Base_Start_IT(&htim15);
			HAL_TIM_Base_Stop_IT(&htim6);
		} else {
			// CHECK IF THERE ARE ANY REQUESTS AT THAT FLOOR GOING IN THE SAME DIRECTION AS THE ELEVATOR, AND SERVICE THEM IF SO
			uint8_t floor_requested = 0;
			switch(floor_position) {
			case 1:
				if (floor_requests[0].requested) {
					HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin, GPIO_PIN_RESET);
					floor_requested = 1;
				}
				break;
			case 2:
				if (floor_requests[1].requested && floor_requests[1].direction == elevator_direction) {
					sprintf(msgs, "%c %c\r\n", elevator_direction, floor_requests[1].direction);
								HAL_UART_Transmit(&huart2, (uint8_t*) msgs, 5, 100);
					HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_RESET);
					floor_requested = 2;
				}
				break;
			case 3:
				if (floor_requests[2].requested) {
					HAL_GPIO_WritePin(FR3_GPIO_Port, FR3_Pin, GPIO_PIN_RESET);
					floor_requested = 3;
				}
				break;
			}

			if (floor_requested) {
				for (int i = 0; i < 3; i++) {
					if (requests[i] == floor_requested) {
						for (int k = i; k < 2; k++) {
							requests[k] = requests[k + 1];
						}
						requests[3] = 0;
						break;
					}
				}
			}

			if (traveling) {

			} else {
				count = 0;
				HAL_TIM_Base_Stop_IT(&htim2);
				start_timer = 0;
				HAL_TIM_Base_Start_IT(&htim2);
				HAL_TIM_Base_Start_IT(&htim15);
				HAL_TIM_Base_Stop_IT(&htim6);
			}
		}
	}
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM2) {
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	}
//}

void Reset_Timer(int num, ...) {
	va_list arg_list;
	va_start(arg_list, num);

	for (int i = 0; i < num; i++) {
		if (va_arg(arg_list, TIM_HandleTypeDef*)->Instance == TIM2) {
			HAL_TIM_Base_Stop_IT(&htim2);
			start_timer = 0;
			HAL_TIM_Base_Start_IT(&htim2);
		} else if (va_arg(arg_list, TIM_HandleTypeDef*)->Instance == TIM15) {
			HAL_TIM_Base_Stop_IT(&htim15);
			count = 0;
			HAL_TIM_Base_Start_IT(&htim15);
		}
	}
}

void Choose_Floor_To_Service(void) {
	// IF there is at least one floor requests and if we are not serving
	if (requests[0] && !serving) {
		serving = requests[0];
		for (int i = 0; i < 2; i++) {
			requests[i] = requests[i + 1];
		}
		requests[2] = 0;

		// CASE 1
		// IF the elevator's floor position matches with the floor that we are serving
		if (floor_position == serving) {
			switch(serving) {
			case 1:
				elevator_direction = 'u';
				HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin, GPIO_PIN_RESET);
				break;
			case 2:
				elevator_direction = floor_requests[1].direction;
				HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin, GPIO_PIN_RESET);
				break;
			case 3:
				elevator_direction = 'd';
				HAL_GPIO_WritePin(FR3_GPIO_Port, FR3_Pin, GPIO_PIN_RESET);
				break;
			}

			HAL_TIM_Base_Start_IT(&htim15);
		} // CASE 2
		  // IF the elevator's floor position does not match the floor that we are serving
		  else if (!traveling && floor_position != serving) {
			  traveling = 1;
			  elevator_direction = floor_position > serving ? 'd' : 'u';
			  HAL_GPIO_WritePin(GPIOC, elevator_direction == 'u' ? DU_Pin : DD_Pin, GPIO_PIN_SET);
			  HAL_TIM_Base_Start_IT(&htim6);
		  }
	}

//	// IF there is at least 1 floor request and if the elevator is either not serving or it is serving and the current floor position ma
//	if (requests[0] != 0 && (!serving || (serving && floor_position == requests[0]))) {
}
/* USER CODE END 1 */
