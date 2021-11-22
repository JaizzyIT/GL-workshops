/*
 * pca9685.c
 *
 *  Created on: Nov 20, 2021
 *      Author: Yuriy Krutchenko
 */

#include "stm32f4xx_hal.h"
#include "pca9685.h"
#include "stm32f4xx_hal_i2c.h"

#define I2C_TX_DELAY 1000
#define MULTIPLIER  40.96		//4095/100

#define MODE1_REG 0						//MODE 1 register address
#define MODE2_REG 1						//MODE 1 register address

#define SUBADR1_REG 2					//SUB ADDRESS 1 register address
#define SUBADR2_REG 3					//SUB ADDRESS 2 register address
#define SUBADR3_REG 4					//SUB ADDRESS 3 register address

#define ALLCALLADR_REG 5				//ALL CALL ADDRESS register address

#define LED_ON_START_REG 6				//LED0 ON first register address
#define LED_OFF_START_REG 8				//LED0 OFF first register address

#define ALL_LED_ON_REG 250				//ALL LEDs ON first register address
#define ALL_LED_OFF_REG 252				//ALL LEDs ON first register address

#define PRE_SCALE_REG 254				//PRESCALER for PWM register address
#define TEST_MODE 255

#define MODE1_RESTART_BIT	7
#define MODE1_EXTCLK_BIT	6
#define MODE1_AI_BIT		5
#define MODE1_SLEEP_BIT		4
#define MODE1_SUB1_BIT		3
#define MODE1_SUB2_BIT		2
#define MODE1_SUB3_BIT		1
#define MODE1_ALLCALL_BIT	0

#define MODE2_INVRT_BIT		4
#define MODE2_OCH_BIT		3
#define MODE2_OUTDRV_BIT	2
#define MODE2_OUTNE1_BIT	1
#define MODE2_OUTNE0_BIT	0

/* @brief function for transmit data via I2C interface
 * @param (I2C_HandleTypeDef) *hi2c - I2C pointer to handle structure;
 * @param (uint8_t) address - address of I2C slave;
 * @param const (uint8_t) *data - pointer to transmitting data;
 * @param (uuint16_t) size - every PWM cycle delay before switching on;
 * @param (uint16_t) delay - timeout for transmitting operation;
 * @retval (int) transmit state
 * */
static int pcaLED_I2C_Master_Transimt(I2C_HandleTypeDef *hi2c, uint8_t address, const uint8_t *data, uint16_t size, uint16_t delay){
	HAL_StatusTypeDef I2C_txStatus = HAL_I2C_Master_Transmit(hi2c, address, (uint8_t *) data, size, delay);
	if (I2C_txStatus == HAL_OK) return 0;
	else return -1;
}

/* @brief setting up a PWM values to pca9685
 * @param (pcaLED_HandleTypeDef) *hpca - pca9685 pointer to structure;
 * @param (uint16_t) ledPin (1-17)- LED number(1-16), all LEDs - 17;
 * @param uint16_t ledDutyCycle (0-100) - duty cycle for chosen LEDs, %;
 * @param uint16_t ledOffset - every PWM cycle delay before switching on;
 * @retval none
 * */
void pcaLED_setPWM(pcaLED_HandleTypeDef *hpca, uint16_t ledPin, uint16_t ledDutyCycle, uint16_t ledOffset){
	uint8_t txDataBuf[5] = {0};
	uint16_t *ptr = NULL;

	if (ledOffset > MAX_BRIGHTNESS) ledOffset = MAX_BRIGHTNESS;
	if (ledDutyCycle > MAX_BRIGHTNESS) ledDutyCycle = MAX_BRIGHTNESS;

	ledOffset = (uint16_t)(ledOffset * MULTIPLIER);
	ledDutyCycle = (uint16_t)(ledDutyCycle * MULTIPLIER);

	if (ledPin == LED_PIN_ALL) txDataBuf[0] = ALL_LED_ON_REG;
	else txDataBuf[0] = (ledPin * 4) + LED_ON_START_REG;

	ptr = (uint16_t*)(txDataBuf+1);
	*ptr = ledOffset;
	ptr = (uint16_t*)(txDataBuf+3);
	*ptr = ((ledOffset + ledDutyCycle) > MAX_RESOLUTION) ? MAX_RESOLUTION : (ledOffset + ledDutyCycle);

	pcaLED_I2C_Master_Transimt(hpca->hi2c, hpca->address, txDataBuf, sizeof(txDataBuf), I2C_TX_DELAY);
}

/* @brief
 * @param
 * @retval none
 * */
void pcaLED_Config(pcaLED_HandleTypeDef *hpca){
	uint8_t txDataBuf[3] = {0};

	HAL_GPIO_WritePin(hpca->OEport, hpca->OEpin, GPIO_PIN_SET);

	txDataBuf[0] = MODE1_REG;
	txDataBuf[1] |= (TRUE << MODE1_AI_BIT);

	if (hpca->invLogic) txDataBuf[2] |= (INV_OUT << MODE2_INVRT_BIT);
	else txDataBuf[2] |= (NON_INV_OUT << MODE2_INVRT_BIT);

	if (hpca->outChg) txDataBuf[2] |= (CHANGE_ON_ACK << MODE2_OCH_BIT);
	else txDataBuf[2] |= (CHANGE_ON_STOP << MODE2_OCH_BIT);

	if (hpca->outDrv) txDataBuf[2] |= (TOTEM_POLE << MODE2_OUTDRV_BIT);
	else txDataBuf[2] |= (OPEN_DRAIN << MODE2_OUTDRV_BIT);

	switch(hpca->outNE){
	case OUT_NE_0:
		txDataBuf[2] |= (FALSE << MODE2_OUTNE0_BIT);
		txDataBuf[2] |= (FALSE << MODE2_OUTNE1_BIT);
	break;
	case OUT_NE_1:
		txDataBuf[2] |= (TRUE << MODE2_OUTNE0_BIT);
		txDataBuf[2] |= (FALSE << MODE2_OUTNE1_BIT);
	break;
	case OUT_NE_HI:
		txDataBuf[2] |= (FALSE << MODE2_OUTNE0_BIT);
		txDataBuf[2] |= (TRUE << MODE2_OUTNE1_BIT);
	break;
	default:
		txDataBuf[2] |= (FALSE << MODE2_OUTNE0_BIT);
		txDataBuf[2] |= (FALSE << MODE2_OUTNE1_BIT);
	}

	hpca->address = (hpca->address << 1);

	pcaLED_I2C_Master_Transimt(hpca->hi2c, hpca->address, txDataBuf, sizeof(txDataBuf), I2C_TX_DELAY);
	pcaLED_setPWM(hpca, LED_PIN_ALL, 0, DEFAULT_OFFSET);

	HAL_GPIO_WritePin(hpca->OEport, hpca->OEpin, GPIO_PIN_RESET);
}
