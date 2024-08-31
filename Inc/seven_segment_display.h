/*
 * 7_segment_display.h
 *
 *  Created on: Aug 31, 2024
 *      Author: karol
 */

#ifndef SEVEN_SEGMENT_DISPLAY_H_
#define SEVEN_SEGMENT_DISPLAY_H_

#include <stdint.h>

void initDisplay();

void setDigitOnDisplay(uint8_t digit);

uint8_t getCurrentDigitOnDisplay();

#endif /* SEVEN_SEGMENT_DISPLAY_H_ */
