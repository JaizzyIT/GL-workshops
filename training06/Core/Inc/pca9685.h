/*
 * pca9685.h
 *
 *  Created on: Nov 20, 2021
 *      Author: Yuriy Krutchenko
 */

#ifndef INC_PCA9685_PWM_H_
#define INC_PCA9685_PWM_H_

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

#define TRUE 1
#define FALSE 0

#define CHANGE_ON_STOP 0				//states for MODE2_OCH_BIT
#define CHANGE_ON_ACK 1

#define NON_INV_OUT 0					//states for MODE2_INVRT_BIT
#define INV_OUT 1

#define OPEN_DRAIN 0					//states for MODE2_OUTDRV_BIT
#define TOTEM_POLE 1

#define OUT_NE_0 0						//states for MODE2_OUTNE1_BIT, MODE2_OUTNE0_BIT
#define OUT_NE_1 1
#define OUT_NE_HI 2

#define DEFAULT_OFFSET 0

#define MAX_RESOLUTION 4095				//MAX resolution of PWM
#define MAX_NUM_OF_LEDS 16
#define MAX_BRIGHTNESS 100

typedef struct{
	uint8_t address;
	uint8_t invLogic;
	uint8_t outChg;
	uint8_t outDrv;
	uint8_t outNE;
	I2C_HandleTypeDef *hi2c;
	GPIO_TypeDef *OEport;
	uint16_t OEpin;
}pcaLED_HandleTypeDef;

typedef enum {
	LED_PIN_0,
	LED_PIN_1,
	LED_PIN_2,
	LED_PIN_3,
	LED_PIN_4,
	LED_PIN_5,
	LED_PIN_6,
	LED_PIN_7,
	LED_PIN_8,
	LED_PIN_9,
	LED_PIN_10,
	LED_PIN_11,
	LED_PIN_12,
	LED_PIN_13,
	LED_PIN_14,
	LED_PIN_15,
	LED_PIN_ALL,
}pcaLED_Pin;

/* @brief configuring pca9685
 * @param (pcaLED_HandleTypeDef) *hpca - pca9685 pointer to structure;
 * @retval none
 * */
void pcaLED_Config(pcaLED_HandleTypeDef *hpca);

/* @brief function for transmit data via I2C interface
 * @param (I2C_HandleTypeDef) *hi2c - I2C pointer to handle structure;
 * @param (uint8_t) address - address of I2C slave;
 * @param const (uint8_t) *data - pointer to transmitting data;
 * @param (uuint16_t) size - every PWM cycle delay before switching on;
 * @param (uint16_t) delay - timeout for transmitting operation;
 * @retval (int) transmit state
 * */
int I2C_Master_Transimt(I2C_HandleTypeDef *hi2c, uint8_t address, const uint8_t *data, uint16_t size, uint16_t delay);

/* @brief setting up a PWM values to pca9685
 * @param (pcaLED_HandleTypeDef) *hpca - pca9685 pointer to structure;
 * @param (uint16_t) ledPin (1-17)- LED number(1-16), all LEDs - 17;
 * @param uint16_t ledDutyCycle (0-100) - duty cycle for chosen LEDs, %;
 * @param uint16_t ledOffset - every PWM cycle delay before switching on;
 * @retval none
 * */
void setLED_PWM(pcaLED_HandleTypeDef *hpca, uint16_t ledPin, uint16_t ledDutyCycle, uint16_t ledOffset);

#endif /* INC_PCA9685_PWM_H_ */
