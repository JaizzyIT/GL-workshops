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
#include "pca9685.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TX_BUF_SIZE 64
#define RX_BUF_SIZE 16
#define UART_TIMEOUT 10

#define TRUE 1
#define FALSE 0

#define DEL_SYMBOL 0x7f 		//127 - ascii code for del symbol (work as backspase as well)
#define ALL_LEDS 0xff
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
pcaLED_HandleTypeDef hpca1;

char ReciveBuf[RX_BUF_SIZE] = {0};
char uartReceiveChar;
char transmitBuf[TX_BUF_SIZE] = {0};

char *rxBuffPtr = ReciveBuf;

_Bool action = FALSE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
static void PCA_LED_Init(void);
void parseNewCommand(void);
void printChar(char* buf);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char* ptr, int len) {
	strncpy(transmitBuf, ptr, len);
	HAL_UART_Transmit(&huart3, (uint8_t*)transmitBuf, len, UART_TIMEOUT);
	return len;
}

/* @brief function for send one char via uart
 * @param *buf pointer to (char) value
 * @retval None
 * */
void printChar(char* buf){
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, 1, UART_TIMEOUT);
}

/* @brief DMA UART3 receive complete callback
 * @param *huart pointer to uart structure
 * @retval None
 * */
void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart){
	if (uartReceiveChar == '\r') {
		*rxBuffPtr = '\0';
		rxBuffPtr = ReciveBuf;
		action = TRUE;
		return;
	}

	if ((uartReceiveChar == DEL_SYMBOL) && (rxBuffPtr > ReciveBuf)){
		rxBuffPtr--;
		printChar(&uartReceiveChar);
		return;
	}

	if ((isalnum(uartReceiveChar) || uartReceiveChar == ' ') == 0)	return;

	if (rxBuffPtr < (ReciveBuf + RX_BUF_SIZE)){
		*rxBuffPtr = uartReceiveChar;
		rxBuffPtr++;
		printChar(&uartReceiveChar);
	}
	else {
		rxBuffPtr = ReciveBuf;
		printf("\r\n--! The command is too long, try again !--\r\n");
	}
}

/* @brief parsing new string to get command from uart
 * @param none
 * @retval none
 * */
void parseNewCommand(void){
	int scanStatus;
	int ledNum = 0, ledBright = 0;
	char str1[3] = {0}, str2[3] = {0};

	scanStatus = sscanf(ReciveBuf, "%s %u %u", str1, &ledNum, &ledBright);
	if (scanStatus != 3 || (strcmp(str1, "led") != 0)){
		scanStatus = sscanf(ReciveBuf, "%s %s %u", str1, str2, &ledBright);
		ledNum = ALL_LEDS;
		if (scanStatus != 3 || (strcmp(str1, "led") != 0) || (strcmp(str2, "all") != 0)){
			printf("\r\n>>! Wrong command or format. Format: led <number> <brightness> !<<\r\n");
			return;
		}
	}
	if (ledBright > MAX_BRIGHTNESS) {
		printf("\r\n>>! Too high brightness value, will be set to 100%% !<<\r\n");
		ledBright = MAX_BRIGHTNESS;
	}
	else if (ledBright < 0){
		printf("\r\n>>! Wrong brightness value, will be set to 0%% !<<\r\n");
		ledBright = 0;
	}

	if (ledNum == ALL_LEDS) {
		if (ledBright > 0) printf("\r\n-- Light up ALL LEDs with %d%% brightness --\r\n", ledBright);
		else printf("\r\n-- Switching off ALL LEDs --\r\n");
		pcaLED_setPWM(&hpca1, LED_PIN_ALL, ledBright, DEFAULT_OFFSET);
		return;
	}
	else if (ledNum > 0 && ledNum <= MAX_NUM_OF_LEDS) {
		if (ledBright > 0) printf("\r\n-- Light up LED number %d with %d%% brightness --\r\n", ledNum, ledBright);
		else printf("\r\n-- Switching off LED number %d --\r\n", ledNum);
		pcaLED_setPWM(&hpca1, ledNum-1, ledBright, DEFAULT_OFFSET);
	}
	else printf("\r\n>>! Wrong LED number. Use 1 to 16 LED number !<<\r\n");
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  PCA_LED_Init();
  UART_Start_Receive_DMA (&huart3,(uint8_t*)&uartReceiveChar, 1);

  printf("\r\n- To control LEDs use next format:\r\n"
		  	 "- led <number>(1-16/all) <brightness>(0-100)\r\n"
		  	 "- Example: led 5 100\r\n"
		  	 "- To confirm press -Enter-\r\n"
		  	 "- To correct input press -Backspace-\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (action){
		  parseNewCommand();

		  action = FALSE;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_OE_GPIO_Port, LED_OE_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : LED_OE_Pin */
  GPIO_InitStruct.Pin = LED_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_OE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static void PCA_LED_Init(void)
{
	hpca1.address = 0x40;// << 1);
	hpca1.invLogic = NON_INV_OUT;
	hpca1.outChg = CHANGE_ON_STOP;
	hpca1.outDrv = TOTEM_POLE;
	hpca1.outNE = OUT_NE_0;
	hpca1.hi2c = &hi2c1;
	hpca1.OEport = LED_OE_GPIO_Port;
	hpca1.OEpin = LED_OE_Pin;
	pcaLED_Config(&hpca1);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
