/*
 * button.h
 *
 *  Created on: Oct 8, 2022
 *      Author: ivanc
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"

#define BUTTON_SHORT_PRESS_DELAY 	100
#define BUTTON_LONG_PRESS_DELAY 	1000

typedef struct {
	uint8_t state : 1;
	uint8_t short_state : 1;
	uint8_t long_state : 1;
} bit_fied_TypeDef;

typedef enum {
	False_Press,
	Short_Press,
	Long_Press,
}Button_Status;


/*
 * @brief Read and recognizes long and short presses encoder button
 * @retval Current encoder button status
 */
Button_Status get_button_status(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);

#endif /* INC_BUTTON_H_ */
