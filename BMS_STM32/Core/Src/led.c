#include "led.h"

#define ON 0 // Depends on your RGB; my RGB turns off the channel when GPIO output is true
#define OFF 1

Colour Red = {ON, OFF, OFF};
Colour Yellow = {ON, ON, OFF};
Colour Green = {OFF, ON, OFF};
Colour Turquoise = {OFF, ON, ON};
Colour Blue = {OFF, OFF, ON};
Colour Magenta = {ON, OFF, ON};
Colour White = {ON, ON, ON};

_Bool BlinkOn = 1;

void Led(LEDMode LEDMode, Colour LedColour) {
	if (LEDMode == Hold) {
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, LedColour.r);
		HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, LedColour.g);
		HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, LedColour.b);
	} else if (LEDMode == Blink) {
		if (BlinkOn == 1) {
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, LedColour.r);
			HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, LedColour.g);
			HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, LedColour.b);
			BlinkOn = 0;
		} else {
			HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, OFF);
			HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, OFF);
			HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, OFF);
			BlinkOn = 1;
		}
	}
}

void LedOn(State_t CurrentState) {
	switch (CurrentState)
	{
		case Initialize:
			Led(Hold, White);
			break;
		case Idle:
			Led(Hold, Green);
			break;
		case Precharging:
			Led(Blink, Magenta);
			break;
		case Run:
			Led(Hold, Magenta);
			break;
		case Stop:
			Led(Blink, Blue);
			break;
		case Sleep:
			Led(Hold, Blue);
			break;
		case NormalDangerFault:
			Led(Hold, Red);
			break;
		case SevereDangerFault:
			Led(Blink, Red);
			break;
		case Charging:
			Led(Hold, Yellow);
			break;
		case Charged:
			Led(Blink, Green);
			break;
		case Balancing:
			Led(Blink, Yellow);
			break;
		default:
			Led(Blink, White);
			break;
	}
}
