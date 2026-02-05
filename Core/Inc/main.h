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
#include "stm32f0xx_hal.h"

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
#define ENABLE_Pin GPIO_PIN_0
#define ENABLE_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_1
#define DIR_GPIO_Port GPIOA
#define STEP_Pin GPIO_PIN_3
#define STEP_GPIO_Port GPIOA
#define SW3_Pin GPIO_PIN_5
#define SW3_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_6
#define SW2_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_7
#define SW1_GPIO_Port GPIOA
#define BT1_Pin GPIO_PIN_0
#define BT1_GPIO_Port GPIOB
#define LMT1_Pin GPIO_PIN_1
#define LMT1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define LMT2_Pin GPIO_PIN_6
#define LMT2_GPIO_Port GPIOB
#define BT2_Pin GPIO_PIN_7
#define BT2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
