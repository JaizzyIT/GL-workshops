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
#include <stdio.h>
#include <string.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0

#define READ 0x03
#define SECTOR_4KB_ERASE 0x20
#define CHIP_ERASE 0x60
#define BYTE_PROGRAM 0x02
#define AAI_WORD_PROGRAM 0xad
#define READ_STATUS_REG 0x05
#define EN_WR_STATUS_REG 0x50
#define WRITE_STATUS_REG 0x01
#define WRITE_ENABLE 0x06
#define WRIGHT_DISABLE 0x04
#define READ_ID 0x90
#define EBSY 0x70					//Enable SO to output RY/BY# status during AAI programming
#define DBSY 0x80					//Disable SO to output RY/BY# status during AAI programming

#define START_ADDERSS 0x00
#define SECTOR_MAX_ADDRESS 0x1ff
#define SECTOR_DATA_MAX_ADDRESS 0xfff

#define TX_UART_BUF_SIZE 64
#define RX_UART_BUF_SIZE 5012
#define UART_TIMEOUT 10

#define TX_SPI_BUF_SIZE 8
#define RX_SPI_BUF_SIZE 5012
#define SPI_TIMEOT 100

#define BYTES_IN_SECTOR 4095
#define EMPTY_BYTE 0xff
#define ENABLE_WRITE_TO_CHIP 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define BIT_IS_SET(var, bit) ((var) & (1<<(bit)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
char reciveUARTbuf[RX_UART_BUF_SIZE] = {0};
char receiveUARTchar;
char transmitUARTbuf[TX_UART_BUF_SIZE] = {0};

uint8_t txSPIbuffer[TX_SPI_BUF_SIZE] = {0};
uint8_t rxSPIbuffer[RX_SPI_BUF_SIZE] = {0};

_Bool statusReading = FALSE;

const char *time_capsule[] = {
		"From: Yuriy Krutchenko, yuriy.elius@gmail.com\r",
		"Mentor: Denys Kondratenko, denys.kondratenko@globallogic.com\r",
		"Date: 26.11.2021\r",
		"TIME CAPSULE\r",
		">>> Richard Dawkins <<<\r",
		"We are going to die, and that makes us the lucky ones\r",
		"Most people are never going to die because they are never going to be born\r",
		"The potential people who could have been here in my place\r",
		"But who will in fact never see the light of day outnumber the sand grains of Sahara\r",
		"Certainly those unborn ghosts include greater poets than Keats, scientists greater than Newton\r",
		"We know this because the set of possible people allowed by our DNA\r",
		"So massively exceeds the set of actual people\r",
		"In the teeth of those stupefying odds it is you and I, in our ordinariness, that are here\r",
		"We privileged few, who won the lottery of birth against all odds\r",
		"How dare we whine at our inevitable return to that prior state\r",
		"From which the vast majority have never stirred?\r",
		"There is grandeur in this view of life, with its several powers\r",
		"Having been originally breathed into a few forms or into one\r",
		"And that whilst this planet has gone cycling on according to the fixed law of gravity\r",
		"From so simple a beginning endless forms most beautiful and most wonderful have been\r",
		"And are being, evolved.\r"
};

uint16_t sizeOfTimeCapsule = (sizeof(time_capsule)/sizeof(time_capsule[0]));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void flashSPI_RxTx_DMA(uint8_t *tx, uint8_t *rx, uint16_t size);
void flashSPI_RxTx_Blocking(uint8_t *tx, uint8_t *rx, uint16_t size);
uint8_t flashReadStatus(void);
void flashWriteStatus(uint8_t command);
//void flashWriteData(uint16_t sector_addr, int16_t byte_addr, const char* data);
void flashWriteByte(uint16_t sector_addr, int16_t byte_addr, uint8_t byte);
void flashChipErase(void);
void flashChipEraseSector(uint16_t sector_addr);
void flashReadData(uint16_t sector_addr_from, uint16_t sector_addr_to, int16_t byte_addr, uint16_t size);
void flashWriteDataToSector(uint16_t sector_addr, int16_t byte_addr, const char* data);
void flashWriteTimeCapsule(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char* ptr, int len) {
	strncpy(transmitUARTbuf, ptr, len);
	HAL_UART_Transmit(&huart3, (uint8_t*)transmitUARTbuf, len, UART_TIMEOUT);
	return len;
}
/* @brief function for send one char via uart
 * @param *buf pointer to (char) value
 * @retval None
 * */
void printChar(char* buf){
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, 1, UART_TIMEOUT);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case BUTT_UP_Pin:
		flashChipErase();													//erase whole chip
		break;
	case BUTT_DOWN_Pin:
		printf("\r\nReading the Time Capsule data\r\n");
		flashReadData(0, (sizeOfTimeCapsule-1), 0, 100);					//read time capsule
		printf("......Done!\r\n");
		break;
	case BUTT_LEFT_Pin:

		break;
	case BUTT_RIGHT_Pin:
		break;
	case BUTT_OK_Pin:
		flashWriteTimeCapsule();											//write time capsule
		break;
	}
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef * huart){					//placeholder for update to project..
/*	if (receiveUARTchar == '\r') {
		*rxBuffPtr = '\0';
		rxBuffPtr = reciveUARTbuf;function
		action = TRUE;
		return;
	status = flashReadStatus();
	if (BIT_IS_SET(status, 0)){
		printf("Error. Devise is busy");
		return;
	}
	}

	if ((receiveUARTchar == DEL_SYMBOL) && (rxBuffPtr > reciveUARTbuf)){
		rxBuffPtr--;
		printChar(&receiveUARTchar);
		return;
	}

	if ((isalnum(receiveUARTchar) || receiveUARTchar == ' ') == 0)	return;

	if (rxBuffPtr < (reciveUARTbuf + RX_UART_BUF_SIZE)){
		*rxBuffPtr = receiveUARTchar;
		rxBuffPtr++;
		printChar(&receiveUARTchar);
	}
	else {
		rxBuffPtr = reciveUARTbuf;
		printf("\r\n--! The command is too long, try again !--\r\n");
	}*/
}

void flashSPI_RxTx_Blocking(uint8_t *tx, uint8_t *rx, uint16_t size){
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_Pin, FALSE);
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, size, SPI_TIMEOT);
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_Pin, TRUE);
}

uint8_t flashReadStatus(void){
	txSPIbuffer[0] = READ_STATUS_REG;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 2);
	return rxSPIbuffer[1];
	printf("Status register value: %x\r\n", rxSPIbuffer[1]);
}

void flashWriteStatus(uint8_t command){
	txSPIbuffer[0] = EN_WR_STATUS_REG;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 1);
	txSPIbuffer[0] = WRITE_STATUS_REG;
	txSPIbuffer[1] = command;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 2);
}

void flashReadData(uint16_t sector_addr_from, uint16_t sector_addr_to, int16_t byte_addr, uint16_t size){
	uint16_t sector_addr_curr = sector_addr_from;

	uint8_t status = flashReadStatus();

	if (BIT_IS_SET(status, 0)){
		printf("Error. Devise is busy\r\n");
		return;
	}
	if (sector_addr_to > SECTOR_MAX_ADDRESS){
		printf("Error. The end of memory\r\n");
		return;
	}

	while(sector_addr_curr <= sector_addr_to){
		txSPIbuffer[0] = READ;
		txSPIbuffer[1] = (uint8_t)(sector_addr_curr >> 4);
		txSPIbuffer[2] = (uint8_t)((sector_addr_curr << 4) | (byte_addr >> 8));
		txSPIbuffer[3] = (uint8_t)byte_addr;

		flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, size);

		if(size == 5 && sector_addr_to == START_ADDERSS) return;

		printf("Line %d: ", sector_addr_curr);						//let's name it for user "line" instead of "sector", that's more convenient ;)
		sector_addr_curr++;

		if (rxSPIbuffer[4] == EMPTY_BYTE){
			printf("empty\r\n");
			continue;
		}
		else{
			printf("%s", (rxSPIbuffer+4));
		}
	}
}

void flashWriteByte(uint16_t sector_addr, int16_t byte_addr, uint8_t byte){
	txSPIbuffer[0] = WRITE_ENABLE;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 1);

	txSPIbuffer[0] = BYTE_PROGRAM;
	txSPIbuffer[1] = (uint8_t)(sector_addr >> 4);
	txSPIbuffer[2] = (uint8_t)((sector_addr << 4) | (byte_addr >> 8));
	txSPIbuffer[3] = (uint8_t)byte_addr;
	txSPIbuffer[4] = byte;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 5);
}

void flashWriteDataToSector(uint16_t sector_addr, int16_t byte_addr, const char* data){
	const char *endPtr = data + (BYTES_IN_SECTOR - 3);

	uint8_t status = flashReadStatus();
	if (BIT_IS_SET(status, 0)){
		printf("Error. Devise is busy\r\n");
		return;
	}

	if (sector_addr > SECTOR_MAX_ADDRESS){
		printf("Error. The end of memory\r\n");
		return;
	}

	if(BYTES_IN_SECTOR < sizeof(data)){
		printf("The data is too long\r\n");
		return;
	}

	while(data < endPtr && *data != '\r'){
		flashWriteByte(sector_addr, byte_addr, (uint8_t) *data);
		data++;
		byte_addr++;
		}
	flashWriteByte(sector_addr, byte_addr, '\r');
	byte_addr++;
	flashWriteByte(sector_addr, byte_addr, '\n');
	byte_addr++;
	flashWriteByte(sector_addr, byte_addr, '\0');

	printf("Line %d is written\r\n", sector_addr);
}

void flashChipErase(void){
	txSPIbuffer[0] = WRITE_ENABLE;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 1);

	txSPIbuffer[0] = CHIP_ERASE;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 1);
	printf("\r\nErasing whole chip\r\n");
}

void flashChipEraseSector(uint16_t sector_addr){
	txSPIbuffer[0] = WRITE_ENABLE;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 1);

	txSPIbuffer[0] = SECTOR_4KB_ERASE;
	txSPIbuffer[1] = (uint8_t)(sector_addr >> 4);
	txSPIbuffer[2] = (uint8_t)(sector_addr << 4);
	txSPIbuffer[3] = 0;
	flashSPI_RxTx_Blocking(txSPIbuffer, rxSPIbuffer, 4);
	printf("Erasing line: %d\r\n", sector_addr);
}

_Bool flashIsSectorEmpty(uint16_t sector_addr){
	flashReadData(START_ADDERSS, START_ADDERSS, START_ADDERSS, 5);
	if (rxSPIbuffer[4] == EMPTY_BYTE) return TRUE;
	else return FALSE;
}

void flashWriteTimeCapsule(void){
	if(flashIsSectorEmpty(START_ADDERSS)){
		printf("\r\nWriting the Time Capsule data\r\n");
		for(uint16_t i = 0; i < sizeOfTimeCapsule; i++){
			flashWriteDataToSector(i, START_ADDERSS, time_capsule[i]);
			}
		printf("......Done!\r\n");
		}
	else printf("The memory is not empty. To write Time Capsule please erase chip first\r\n");
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA (&huart3,(uint8_t*)&receiveUARTchar, 1);

  flashWriteStatus(ENABLE_WRITE_TO_CHIP);

  printf("\r\n- To read data from flash memory: press <DOWN> button -\r\n"
		  	 "- To erase whole flash memory: press <CENTER/OK> button -\r\n"
		  	 "- To write Time Capsule to flash memory: press <CENTER/OK> button -\r\n");
/*		  	 "- placeholder -\r\n"
		  	 "- placeholder -\r\n");*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

/*	  flashReadStatus();

	  HAL_Delay(1000);*/


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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_GREEN_Pin|LED_ORANGE_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

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
