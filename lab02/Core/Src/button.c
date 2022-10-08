/*
 * button.c
 *
 *  Created on: Oct 8, 2022
 *      Author: ivanc
 */

#include "button.h"

bit_fied_TypeDef btn = {0, 0, 0};
uint32_t btn_time = 0;

Button_Status get_button_status(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin)
{
	btn.state = !HAL_GPIO_ReadPin(GPIO_port, GPIO_pin);

	if (btn.state && !btn.short_state && ((HAL_GetTick() - btn_time) > BUTTON_SHORT_PRESS_DELAY)) {
		btn.short_state = 1;
		btn.long_state = 0;
		btn_time = HAL_GetTick();
	}
	else if (btn.state && !btn.long_state && (HAL_GetTick() - btn_time) > BUTTON_LONG_PRESS_DELAY) {
		btn.long_state = 1;
		return Long_Press;
	}
	else if (!btn.state && btn.short_state && (HAL_GetTick() - btn_time) > BUTTON_SHORT_PRESS_DELAY) {
		btn.short_state = 0;
		btn_time = HAL_GetTick();
		if(!btn.long_state) {
			return Short_Press;
		}
	}
	return False_Press;
}
