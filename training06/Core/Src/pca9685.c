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
#define MULTIPLIER  40.95		//4095/100

/* @brief function for transmit data via I2C interface
 * @param (I2C_HandleTypeDef) *hi2c - I2C pointer to handle structure;
 * @param (uint8_t) address - address of I2C slave;
 * @param const (uint8_t) *data - pointer to transmitting data;
 * @param (uuint16_t) size - every PWM cycle delay before switching on;
 * @param (uint16_t) delay - timeout for transmitting operation;
 * @retval (int) transmit state
 * */
int I2C_Master_Transimt(I2C_HandleTypeDef *hi2c, uint8_t address, const uint8_t *data, uint16_t size, uint16_t delay){
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
void setLED_PWM(pcaLED_HandleTypeDef *hpca, uint16_t ledPin, uint16_t ledDutyCycle, uint16_t ledOffset){
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

	I2C_Master_Transimt(hpca->hi2c, hpca->address, txDataBuf, sizeof(txDataBuf), I2C_TX_DELAY);
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

	I2C_Master_Transimt(hpca->hi2c, hpca->address, txDataBuf, sizeof(txDataBuf), I2C_TX_DELAY);
	setLED_PWM(hpca, LED_PIN_ALL, 0, DEFAULT_OFFSET);

	HAL_GPIO_WritePin(hpca->OEport, hpca->OEpin, GPIO_PIN_RESET);
}
