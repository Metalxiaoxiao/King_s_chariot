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
#define RF_PWM_TIM2_Pin GPIO_PIN_0
#define RF_PWM_TIM2_GPIO_Port GPIOA
#define LF_PWM_TIM2_Pin GPIO_PIN_1
#define LF_PWM_TIM2_GPIO_Port GPIOA
#define RB_2_AIN2_Pin GPIO_PIN_2
#define RB_2_AIN2_GPIO_Port GPIOA
#define RB_2_AIN1_Pin GPIO_PIN_3
#define RB_2_AIN1_GPIO_Port GPIOA
#define LB_2_BIN1_Pin GPIO_PIN_6
#define LB_2_BIN1_GPIO_Port GPIOA
#define LB_2_BIN2_Pin GPIO_PIN_7
#define LB_2_BIN2_GPIO_Port GPIOA
#define SS_Pin GPIO_PIN_12
#define SS_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_14
#define MOSI_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_15
#define MISO_GPIO_Port GPIOB
#define DJ2_TIM1_Pin GPIO_PIN_8
#define DJ2_TIM1_GPIO_Port GPIOA
#define DJ1_TIM1_Pin GPIO_PIN_9
#define DJ1_TIM1_GPIO_Port GPIOA
#define RB_PWM_TIM1_Pin GPIO_PIN_10
#define RB_PWM_TIM1_GPIO_Port GPIOA
#define LB_PWM_TIM1_Pin GPIO_PIN_11
#define LB_PWM_TIM1_GPIO_Port GPIOA
#define BOOST_Pin GPIO_PIN_12
#define BOOST_GPIO_Port GPIOA
#define LF_1_AIN1_Pin GPIO_PIN_15
#define LF_1_AIN1_GPIO_Port GPIOA
#define LF_1_AIN2_Pin GPIO_PIN_3
#define LF_1_AIN2_GPIO_Port GPIOB
#define RF_1_BIN1_Pin GPIO_PIN_8
#define RF_1_BIN1_GPIO_Port GPIOB
#define RF_1_BIN2_Pin GPIO_PIN_9
#define RF_1_BIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
