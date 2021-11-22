/*
 * pca9685.h
 *
 *  Created on: Nov 20, 2021
 *      Author: Yuriy Krutchenko
 */

#ifndef INC_PCA9685_PWM_H_
#define INC_PCA9685_PWM_H_

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

#define MAX_RESOLUTION 4095				//MAX resolution of PWM
#define MAX_NUM_OF_LEDS 16
#define MAX_BRIGHTNESS 100

#define DEFAULT_OFFSET 0

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

/* @brief setting up a PWM values to pca9685
 * @param (pcaLED_HandleTypeDef) *hpca - pca9685 pointer to structure;
 * @param (uint16_t) ledPin (1-17)- LED number(1-16), all LEDs - 17;
 * @param uint16_t ledDutyCycle (0-100) - duty cycle for chosen LEDs, %;
 * @param uint16_t ledOffset - every PWM cycle delay before switching on;
 * @retval none
 * */
void pcaLED_setPWM(pcaLED_HandleTypeDef *hpca, uint16_t ledPin, uint16_t ledDutyCycle, uint16_t ledOffset);

#endif /* INC_PCA9685_PWM_H_ */
