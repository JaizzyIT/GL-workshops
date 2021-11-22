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
//#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LED_GREEN_PWM (htim4.Instance->CCR1)
#define LED_ORANGE_PWM (htim4.Instance->CCR2)
#define LED_RED_PWM (htim4.Instance->CCR3)
#define LED_BLUE_PWM (htim4.Instance->CCR4)
#define LED_RED_WARN_PERIOD (htim10.Instance->ARR)
#define TIM10_UPDATE (htim10.Instance->EGR)
#define WARN_BLINK_1_FREQ 10						// Hz*10
#define WARN_BLINK_2_FREQ 25						// Hz*10
#define WARN_BLINK_3_FREQ 50						// Hz*10

#define ADC_RESOLUTION 4095
#define VREF 3000
#define AVR_BUFF_SIZE 8
#define V_EXT_T_SENS_0 2020							//mV - external sensor voltage at 0 degrees Celsius
#define V_EXT_T_SENS_100 20							//mV - external sensor voltage at 100 degrees Celsius
#define V25 760										//mV - datasheet value for 25° C (internal temperature sensor)
#define AVG_SLOPE 2.5								//mV/°C - average slope (internal temperature sensor)


#define TRUE 1
#define FALSE 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
volatile uint8_t newADCData;

uint32_t carrierFreq;

uint16_t adcData[3];
uint16_t potADCAvrBuff[AVR_BUFF_SIZE], ExtTempAvrBuff[AVR_BUFF_SIZE], intTempAvrBuff[AVR_BUFF_SIZE];
uint16_t potADCVal, extTempADCVal, intTempADCVal;
uint16_t potVolts, extTempDegree, intTempDegree;
uint16_t extTempVolts, intTempVolts;

const uint16_t potPosMax = VREF, potPosMin = 0;			//mV
const uint16_t extTempMax = 100, extTempMin = 0;		//°C
const uint16_t intTempMax = 100, intTempMin = 0;		//°C

const uint16_t potPosWarnHI = 1000;						// mV
const uint16_t extTempWarnHI = 30;						// %
const uint16_t intTempWarnHI = 35;						// %

const uint16_t potHysteresis = 50;						//mV
const uint16_t tempHysteresis = 1;						//°C

enum {
	POTENTIOMETER,
	EXT_TEMP_SENSOR,
	INT_TEMP_SENSOR
};

enum {
	WARNING_LVL_1 = 1,
	WARNING_LVL_2 = 2,
	WARNING_LVL_3 = 3
}warningLevel;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void updateAverageValue(void);
void updatePWM(void);
void adcDataConvert(void);
void alarmBlink(void);
_Bool hysteresisCheck(uint16_t, _Bool, uint16_t, uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* @brief TIM10 counter elapsed callback
 * @param *htim pointer to timer structure
 * @retval None
 * */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOD, LED_RED_Pin);
}

/*
int _write(int file, char* ptr, int len) {
	int i = 0;
	for (i = 0; i<len; i++) ITM_SendChar(*ptr++);
	return len;
}*/

/* @brief ADC group conversion complete
 * @param hadc pointer to ADC structure
 * @retval None
 * */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
		newADCData = TRUE;
}

/* @brief reading new ADC data from DMA buffer and averaging it
 * @param none
 * @retval None
 * */
void updateAverageValue(void){
	static uint8_t currentAvrElement = 0;

	potADCAvrBuff[currentAvrElement] = adcData[POTENTIOMETER];
	ExtTempAvrBuff[currentAvrElement] = adcData[EXT_TEMP_SENSOR];
	intTempAvrBuff[currentAvrElement] = adcData[INT_TEMP_SENSOR];

	currentAvrElement = (currentAvrElement >= (AVR_BUFF_SIZE - 1)) ? 0 : (currentAvrElement + 1);

	potADCVal = 0;
	extTempADCVal = 0;
	intTempADCVal = 0;

	for (uint8_t i = 0; i < AVR_BUFF_SIZE ; i++){
		potADCVal += potADCAvrBuff[i];
		extTempADCVal += ExtTempAvrBuff[i];
		intTempADCVal += intTempAvrBuff[i];
	}

	potADCVal = potADCVal >> 3;
	extTempADCVal = extTempADCVal >> 3;
	intTempADCVal = intTempADCVal >> 3;
}

/* @brief checking if value limits reached
 * @param constant pointers to (uint16_t) value, max, min
 * @retval (uint16_t) result of limits check
 * */
uint16_t limitsCheck(const uint16_t* const value, const uint16_t* const max, const uint16_t* const min){
	if (*value > *max) return *max;
	if (*value < *min) return *min;
	else return *value;
}

/* @brief update PWM values to TIM4 channels configured for PWM
 * @param none
 * @retval none
 * */
void updatePWM(void){
	LED_BLUE_PWM = potVolts * PWM_PERIOD / VREF;
	LED_GREEN_PWM = limitsCheck(&extTempDegree, &extTempMax, &extTempMin) * PWM_PERIOD / extTempMax;
	LED_ORANGE_PWM = limitsCheck(&intTempDegree, &intTempMax, &intTempMin)  * PWM_PERIOD / intTempMax;
}

/* @brief converting raw ADC data to volts and degrees
 * @param none
 * @retval none
 * */
void adcDataConvert(void){
	/*Potentiometer: discrete to volts*/
	potVolts = VREF * potADCVal / ADC_RESOLUTION;
	/*External temperature sensor: discrete to volts and to degrees*/
	extTempVolts = VREF * extTempADCVal / ADC_RESOLUTION;
	extTempDegree = extTempMax - ((extTempVolts - V_EXT_T_SENS_100) * extTempMax / (V_EXT_T_SENS_0 - V_EXT_T_SENS_100)) + extTempMin;
	/*Internal temperature sensor: discrete to volts and to degrees*/
	intTempVolts = VREF * intTempADCVal / ADC_RESOLUTION;
	intTempDegree = ((intTempVolts - V25) / AVG_SLOPE) + 25;		//formula from datasheet

//	 printf("%d %d %d\n", potVolts, extTempDegree, intTempDegree);
}

/* @brief checking if alarm is occurred and applying hysteresis
 * @param (uint16_t) value, warnHI, hyst, (bool) warnPrev
 * @retval bool value
 * */
_Bool hysteresisCheck(uint16_t value, _Bool warnPrev, uint16_t warnHI, uint16_t hyst){
	if (warnPrev) return (value > (warnHI - hyst));
	else return (value > warnHI);
}

/* @brief analysing if alarm blink is needed
 * @param none
 * @retval none
 * */
void alarmBlink(void){
uint8_t warning = 0;
static uint8_t warnPrev = 0;
static _Bool potPosWarnHiPrev, extTempWarnHiPrev, intTempWarnHiPrev;

	if (hysteresisCheck(potVolts, potPosWarnHiPrev, potPosWarnHI, potHysteresis)) {
		warning++;
		potPosWarnHiPrev = TRUE;
	}
	else potPosWarnHiPrev = FALSE;

	if (hysteresisCheck(extTempDegree, extTempWarnHiPrev, extTempWarnHI, tempHysteresis)) {
		warning++;
		extTempWarnHiPrev = TRUE;
	}
	else extTempWarnHiPrev = FALSE;

	if (hysteresisCheck(intTempDegree, intTempWarnHiPrev, intTempWarnHI, tempHysteresis)) {
		warning++;
		intTempWarnHiPrev = TRUE;
	}
	else intTempWarnHiPrev = FALSE;

	if (warning){
		switch (warning){
		case WARNING_LVL_1:
			LED_RED_WARN_PERIOD = ((carrierFreq * 10)/(WARN_BLINK_1_FREQ * 2)) - 1;
			break;
		case WARNING_LVL_2:
			LED_RED_WARN_PERIOD = ((carrierFreq * 10)/(WARN_BLINK_2_FREQ * 2)) - 1;
			break;
		case WARNING_LVL_3:
			LED_RED_WARN_PERIOD = ((carrierFreq * 10)/(WARN_BLINK_3_FREQ * 2)) - 1;
			break;
		}
	}
	else {
		LED_RED_WARN_PERIOD = 0;
		HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_RESET);
	}
	if (warning != warnPrev) {
		TIM10_UPDATE = TRUE;
		warnPrev = warning;
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
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcData, 3);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim10);
  //LED_RED_WARN_PERIOD = 0;
  carrierFreq = (uint32_t)(HAL_RCC_GetPCLK2Freq()/(TIM10_PRESCALER));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (newADCData) {
		  updateAverageValue();

		  adcDataConvert();

		  updatePWM();

		  alarmBlink();

		  newADCData = FALSE;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = TIM4_PRESCALER-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PWM_PERIOD-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = TIM10_PRESCALER-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

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
