/*
 * 7_segment_display.c
 *
 *  Created on: Aug 31, 2024
 *      Author: karol
 */

#include <seven_segment_display.h>
#include "stm32f407xx.h"
#include <stdint.h>


/* Bit positions of all leds in special code, look at "digitCodes"
 *   __0_
 *  |	 |
 * 5|	 |1
 *  |__6_|
 *  |	 |
 * 4|	 |2
 *  |____|
 *     3
 *
 *  */

static const uint8_t digitCodes[10]=
{
	0x7F, // 0
	0x6,  // 1
	0x5B, // 2
	0x4F, // 3
	0x66, // 4
	0x6D, // 5
	0xFD, // 6
	0x7,  // 7
	0xFF, // 8
	0xEF, // 9
};

static uint8_t currentDigitOnDisplay=0;

void initDisplay()
{
	GPIO_Handle_t gpioLEDDHandler={0};
	gpioLEDDHandler=(GPIO_Handle_t){
			.pGPIOx=GPIOA,
			.GPIO_PinConfig=(GPIO_PinConfig_t){
				.GPIO_PinNumber=GPIO_PIN_NUM_0,
				.GPIO_PinMode=GPIO_MODE_OUTPUT,
				.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP,
				.GPIO_PinOutputSpeed = GPIO_OUTPUT_SPEED_VERY_HIGH,
			}
	};

	GPIO_Init(&gpioLEDDHandler);

	gpioLEDDHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_1;
	GPIO_Init(&gpioLEDDHandler);

	gpioLEDDHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_2;
	GPIO_Init(&gpioLEDDHandler);

	gpioLEDDHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_3;
	GPIO_Init(&gpioLEDDHandler);

	gpioLEDDHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_4;
	GPIO_Init(&gpioLEDDHandler);

	gpioLEDDHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	GPIO_Init(&gpioLEDDHandler);

	gpioLEDDHandler.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_6;
	GPIO_Init(&gpioLEDDHandler);
}


void setDigitOnDisplay(uint8_t digit)
{
	uint8_t digitCode = digitCodes[digit];

	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_0, (digitCode & (1U << 0) ) ? GPIO_HIGH : GPIO_LOW);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_1, (digitCode & (1U << 1) ) ? GPIO_HIGH : GPIO_LOW);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_2, (digitCode & (1U << 2) ) ? GPIO_HIGH : GPIO_LOW);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_3, (digitCode & (1U << 3) ) ? GPIO_HIGH : GPIO_LOW);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_4, (digitCode & (1U << 4) ) ? GPIO_HIGH : GPIO_LOW);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_5, (digitCode & (1U << 5) ) ? GPIO_HIGH : GPIO_LOW);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NUM_6, (digitCode & (1U << 6) ) ? GPIO_HIGH : GPIO_LOW);

	currentDigitOnDisplay=digit;
}

uint8_t getCurrentDigitOnDisplay()
{
	return currentDigitOnDisplay;
}
