/*
 * RGB.c
 *
 *  Created on: Dec 11, 2024
 *      Author: oguzh
 */

#include "RGB.h"

void RGB_LED_Init(RGB_LED *led, TIM_HandleTypeDef *htim, uint32_t red_channel, uint32_t green_channel, uint32_t blue_channel)
{
	led->timer = htim;
	led->red_channel = red_channel;
	led->green_channel = green_channel;
	led->blue_channel = blue_channel;

	// PWM'leri baslat
	HAL_TIM_PWM_Start(led->timer,led->red_channel);
	HAL_TIM_PWM_Start(led->timer,led->green_channel);
	HAL_TIM_PWM_Start(led->timer,led->blue_channel);
}

void RGB_LED_Set_Color(RGB_LED *led, uint8_t red, uint8_t green, uint8_t blue)
{
    uint32_t red_val = (uint32_t)((led->timer->Init.Period + 1) * red / 255);
    uint32_t green_val = (uint32_t)((led->timer->Init.Period + 1) * green / 255);
    uint32_t blue_val = (uint32_t)((led->timer->Init.Period + 1) * blue / 255);

    // PWM'lerin duty degerini set et
    __HAL_TIM_SET_COMPARE(led->timer, led->red_channel, red_val);
    __HAL_TIM_SET_COMPARE(led->timer, led->green_channel, green_val);
    __HAL_TIM_SET_COMPARE(led->timer, led->blue_channel, blue_val);
}

void RGB_LED_Off(RGB_LED* led)
{
	RGB_LED_Set_Color(led,0,0,0);
}


