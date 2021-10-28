/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM4_PRESCALER 8
#define BUTT_UP_Pin GPIO_PIN_6
#define BUTT_UP_GPIO_Port GPIOC
#define BUTT_UP_EXTI_IRQn EXTI9_5_IRQn
#define BUTT_DOWN_Pin GPIO_PIN_8
#define BUTT_DOWN_GPIO_Port GPIOC
#define BUTT_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define BUTT_LEFT_Pin GPIO_PIN_9
#define BUTT_LEFT_GPIO_Port GPIOC
#define BUTT_LEFT_EXTI_IRQn EXTI9_5_IRQn
#define BUTT_OK_Pin GPIO_PIN_15
#define BUTT_OK_GPIO_Port GPIOA
#define BUTT_OK_EXTI_IRQn EXTI15_10_IRQn
#define BUTT_RIGHT_Pin GPIO_PIN_11
#define BUTT_RIGHT_GPIO_Port GPIOC
#define BUTT_RIGHT_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
