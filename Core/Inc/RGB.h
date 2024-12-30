/*
 * RGB.h
 *
 *  Created on: Dec 11, 2024
 *      Author: oguzh
 */

#ifndef INC_RGB_H_
#define INC_RGB_H_

#include "stm32f4xx_hal.h"

typedef struct {

	TIM_HandleTypeDef *timer;
	uint32_t red_channel;
	uint32_t green_channel;
	uint32_t blue_channel;

}RGB_LED;

// RGB LED Init
void RGB_LED_Init(RGB_LED* led, TIM_HandleTypeDef *htim, uint32_t red_channel, uint32_t green_channel, uint32_t blue_channel);

// RGB LED renk ayarı
void RGB_LED_Set_Color(RGB_LED* led, uint8_t red, uint8_t green, uint8_t blue);

// RGB LED söndür
void RGB_LED_Off(RGB_LED* led);

#endif /* INC_RGB_H_ */
