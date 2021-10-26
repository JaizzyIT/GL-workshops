/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DELAY_STEP_DEFAULT 7
#define DELAY_STEP_MIN 1
#define DELAY_STEP_MAX 15
#define DELAY_MAX 100
#define DELAY_MIN 1
#define TRUE 1
#define FALSE 0
#define NUM_OF_LEDS 4
#define NUM_OF_MODES 3
#define FIRST_MODE 1
#define RANDOM_MODE_DELAY 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;

/* USER CODE BEGIN PV */
uint16_t ledSelect[NUM_OF_LEDS] = { LED_GREEN_Pin, LED_ORANGE_Pin, LED_RED_Pin,	LED_BLUE_Pin };
int8_t currentDelay;
uint8_t delayStep = DELAY_STEP_DEFAULT;
uint8_t currentLED;
uint8_t mode = 1;
uint32_t randomNumber;
_Bool sleep;
_Bool increaseFlag;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* @brief GPIO EXTI PIN Handler
 * @param GPIO PIN caused an interruption
 * @retval None
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	switch (GPIO_Pin) {
	case BUTT_UP_Pin:
		if(delayStep < DELAY_STEP_MAX) delayStep++;
		break;
	case BUTT_DOWN_Pin:
		if(delayStep > DELAY_STEP_MIN) delayStep--;
		break;
	case BUTT_LEFT_Pin:
		if (mode <= FIRST_MODE) mode = NUM_OF_MODES;
		else mode--;
		break;
	case BUTT_RIGHT_Pin:
		if (mode >= NUM_OF_MODES) mode = FIRST_MODE;
		else mode++;
		break;
	case BUTT_OK_Pin:
		if (sleep){
			HAL_ResumeTick();
			sleep = FALSE;
		}
		else sleep = TRUE;
		break;
	}

}

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
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (!sleep){
	    	switch (mode) {
	    	case 1:
	    		/* consecutive light up one LED with cyclically changing delay*/
	    		for (uint8_t i = 0; i < NUM_OF_LEDS; i++) {
	    			if (i != currentLED)HAL_GPIO_WritePin(GPIOD, ledSelect[i], GPIO_PIN_RESET);
	    			else HAL_GPIO_WritePin(GPIOD, ledSelect[i], GPIO_PIN_SET);
	    		}
	    		break;
	    	case 2:
	    		/* consecutive switch off one LED with cyclically changing delay */
	    		for (uint8_t i = 0; i < NUM_OF_LEDS; i++) {
	    			if (i == currentLED) HAL_GPIO_WritePin(GPIOD, ledSelect[i], GPIO_PIN_RESET);
	    			else HAL_GPIO_WritePin(GPIOD, ledSelect[i], GPIO_PIN_SET);
	    		}
	    		break;
	    	case 3:
	    		/* randomly lighting LEDs with constant delay */
	    		HAL_RNG_GenerateRandomNumber(&hrng, &randomNumber);
				HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

	    		for (uint8_t i = 0; i < 4; i++){
	    			if (randomNumber & (1 << i)) HAL_GPIO_WritePin(GPIOD, ledSelect[i], GPIO_PIN_SET);
	    		}
	    		HAL_Delay(RANDOM_MODE_DELAY * delayStep);
	    		continue;
	    		break;
	    	}

	    	/* moving to next LED */
	    	if (currentLED < NUM_OF_LEDS)
	    		currentLED++;
	    	else currentLED = 0;

	    	/* defining delay */
	    	if (increaseFlag == TRUE) {
	    		if (currentDelay < DELAY_MAX){
	    			currentDelay += delayStep;
	    			if (currentDelay > DELAY_MAX)currentDelay = DELAY_MAX;
	    		}
	    		else {
	    			currentDelay = DELAY_MAX;
	    			increaseFlag = FALSE;
	    			}
	    		}
	    	else {
	    		if (currentDelay > DELAY_MIN){

	    			currentDelay -= delayStep;
	    			if (currentDelay < DELAY_MIN) currentDelay = DELAY_MIN;
	    		}
	    		else {
	    			currentDelay = DELAY_MIN;
	    			increaseFlag = TRUE;
	    			}
	    		}
	    	HAL_Delay(currentDelay);
	    }
		else {
			HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_ORANGE_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTT_UP_Pin BUTT_DOWN_Pin BUTT_LEFT_Pin BUTT_RIGHT_Pin */
  GPIO_InitStruct.Pin = BUTT_UP_Pin|BUTT_DOWN_Pin|BUTT_LEFT_Pin|BUTT_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTT_OK_Pin */
  GPIO_InitStruct.Pin = BUTT_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTT_OK_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
	while (1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
