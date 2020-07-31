#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"
#include "stm32l4xx_hal.h"

void ledPin(_Bool red, _Bool green, _Bool blue);
void led(int col);
void led_flash(int col);

#endif /* INC_LED_H_ */
