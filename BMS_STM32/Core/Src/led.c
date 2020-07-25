#include "led.h"

#define Port GPIOC
#define RedPin RedLED_Pin
#define GreenPin GreenLED_Pin
#define BluePin BlueLED_Pin

void ledOn(char color[3]) {
	if (color == "RED") {
		HAL_GPIO_WritePin(Port, RedPin, 1);
		HAL_GPIO_WritePin(Port, GreenPin, 0);
		HAL_GPIO_WritePin(Port, BluePin, 0);
	} else if (color == "YLW") {
		HAL_GPIO_WritePin(Port, RedPin, 1);
		HAL_GPIO_WritePin(Port, GreenPin, 1);
		HAL_GPIO_WritePin(Port, BluePin, 0);
	} else if (color == "GRN") {
		HAL_GPIO_WritePin(Port, RedPin, 0);
		HAL_GPIO_WritePin(Port, GreenPin, 1);
		HAL_GPIO_WritePin(Port, BluePin, 0);
	} else if (color == "BLU") {
		HAL_GPIO_WritePin(Port, RedPin, 0);
		HAL_GPIO_WritePin(Port, GreenPin, 0);
		HAL_GPIO_WritePin(Port, BluePin, 1);
	} else if (color == "PRP") {
		HAL_GPIO_WritePin(Port, RedPin, 1);
		HAL_GPIO_WritePin(Port, GreenPin, 0);
		HAL_GPIO_WritePin(Port, BluePin, 1);
	}
}

void led(char color[3]) {
	ledOn(color);
}

void led_flash(char color[3]) {
	ledOn(color);

	HAL_Delay(1000);

	HAL_GPIO_WritePin(Port, RedPin, 0);
	HAL_GPIO_WritePin(Port, GreenPin, 0);
	HAL_GPIO_WritePin(Port, BluePin, 0);

	HAL_Delay(1000);
}
