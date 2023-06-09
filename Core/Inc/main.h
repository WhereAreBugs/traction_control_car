/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define AIN_2_Pin GPIO_PIN_13
#define AIN_2_GPIO_Port GPIOC
#define AIN_1_Pin GPIO_PIN_14
#define AIN_1_GPIO_Port GPIOC
#define BIN_1_Pin GPIO_PIN_15
#define BIN_1_GPIO_Port GPIOC
#define pwmA_Pin GPIO_PIN_0
#define pwmA_GPIO_Port GPIOA
#define pwmB_Pin GPIO_PIN_1
#define pwmB_GPIO_Port GPIOA
#define Input_L_Pin GPIO_PIN_2
#define Input_L_GPIO_Port GPIOA
#define Input_LM_Pin GPIO_PIN_3
#define Input_LM_GPIO_Port GPIOA
#define Input_RM_Pin GPIO_PIN_4
#define Input_RM_GPIO_Port GPIOA
#define Input_R_Pin GPIO_PIN_5
#define Input_R_GPIO_Port GPIOA
#define Input_M_Pin GPIO_PIN_6
#define Input_M_GPIO_Port GPIOA
#define BIN_2_Pin GPIO_PIN_7
#define BIN_2_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_0
#define SERVO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
