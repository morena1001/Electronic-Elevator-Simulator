/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define PF1_Pin GPIO_PIN_0
#define PF1_GPIO_Port GPIOC
#define PF2_Pin GPIO_PIN_1
#define PF2_GPIO_Port GPIOC
#define PF3_Pin GPIO_PIN_2
#define PF3_GPIO_Port GPIOC
#define DF1_Pin GPIO_PIN_3
#define DF1_GPIO_Port GPIOC
#define DF2_Pin GPIO_PIN_4
#define DF2_GPIO_Port GPIOC
#define DF3_Pin GPIO_PIN_5
#define DF3_GPIO_Port GPIOC
#define II1_Pin GPIO_PIN_0
#define II1_GPIO_Port GPIOB
#define II1_EXTI_IRQn EXTI0_IRQn
#define II2_Pin GPIO_PIN_1
#define II2_GPIO_Port GPIOB
#define II2_EXTI_IRQn EXTI1_IRQn
#define II3_Pin GPIO_PIN_2
#define II3_GPIO_Port GPIOB
#define II3_EXTI_IRQn EXTI2_TSC_IRQn
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define DU_Pin GPIO_PIN_6
#define DU_GPIO_Port GPIOC
#define DD_Pin GPIO_PIN_7
#define DD_GPIO_Port GPIOC
#define FR1_Pin GPIO_PIN_8
#define FR1_GPIO_Port GPIOC
#define FR2_Pin GPIO_PIN_9
#define FR2_GPIO_Port GPIOC
#define FR3_Pin GPIO_PIN_10
#define FR3_GPIO_Port GPIOC
#define BCDI_Pin GPIO_PIN_11
#define BCDI_GPIO_Port GPIOC
#define OI1U_Pin GPIO_PIN_3
#define OI1U_GPIO_Port GPIOB
#define OI1U_EXTI_IRQn EXTI3_IRQn
#define OI2D_Pin GPIO_PIN_4
#define OI2D_GPIO_Port GPIOB
#define OI2D_EXTI_IRQn EXTI4_IRQn
#define OI2U_Pin GPIO_PIN_5
#define OI2U_GPIO_Port GPIOB
#define OI2U_EXTI_IRQn EXTI9_5_IRQn
#define OI3D_Pin GPIO_PIN_6
#define OI3D_GPIO_Port GPIOB
#define OI3D_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
