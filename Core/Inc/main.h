/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define led_Pin GPIO_PIN_13
#define led_GPIO_Port GPIOC
#define EC1_Pin GPIO_PIN_0
#define EC1_GPIO_Port GPIOA
#define EC2_Pin GPIO_PIN_1
#define EC2_GPIO_Port GPIOA
#define PWMC_Pin GPIO_PIN_2
#define PWMC_GPIO_Port GPIOA
#define PWMD_Pin GPIO_PIN_3
#define PWMD_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOA
#define EA__Pin GPIO_PIN_6
#define EA__GPIO_Port GPIOA
#define EA2_Pin GPIO_PIN_7
#define EA2_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_4
#define AIN2_GPIO_Port GPIOC
#define DIN__Pin GPIO_PIN_14
#define DIN__GPIO_Port GPIOE
#define DIN2_Pin GPIO_PIN_15
#define DIN2_GPIO_Port GPIOE
#define EB1_Pin GPIO_PIN_12
#define EB1_GPIO_Port GPIOD
#define EB2_Pin GPIO_PIN_13
#define EB2_GPIO_Port GPIOD
#define ED1_Pin GPIO_PIN_6
#define ED1_GPIO_Port GPIOC
#define ED2_Pin GPIO_PIN_7
#define ED2_GPIO_Port GPIOC
#define PWMA_Pin GPIO_PIN_15
#define PWMA_GPIO_Port GPIOA
#define CIN1_Pin GPIO_PIN_10
#define CIN1_GPIO_Port GPIOC
#define CIN2_Pin GPIO_PIN_11
#define CIN2_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_3
#define PWMB_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_7
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_8
#define BIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
