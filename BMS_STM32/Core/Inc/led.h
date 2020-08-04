#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"
#include "stm32l4xx_hal.h"

typedef struct colour {
	_Bool r;
	_Bool g;
	_Bool b;
} Colour;

typedef enum {
	Blink,
	Hold
} LEDMode;

void LedOn(State_t CurrentState);

#endif /* INC_LED_H_ */
