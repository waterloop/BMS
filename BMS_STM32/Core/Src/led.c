#include "led.h"

#define ON GPIO_PIN_RESET
#define OFF GPIO_PIN_SET

void ledPin(_Bool red, _Bool green, _Bool blue) {
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, !(red));
	HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, !(green));
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, !(blue));
}

void led(int col) {
	switch(col)
	{
		case 1: // red
			ledPin(1,0,0);
			break;
		case 2: // yellow
			ledPin(1,1,0);
			break;
		case 3: // green
			ledPin(0,1,0);
			break;
		case 4: // turqoise
			ledPin(0,1,1);
			break;
		case 5: // blue
			ledPin(0,0,1);
			break;
		case 6: // magenta
			ledPin(1,0,1);
			break;
		case 7: // white
			ledPin(1,0,1);
			break;
		default: // off
			ledPin(0,0,0);
	}
}

void led_flash(int col) {
	led(col);
	HAL_Delay(500);
	led(0);
	HAL_Delay(500);
}
