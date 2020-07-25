#include "contactor.h"

void contactor(int OnOff) {
	HAL_GPIO_WritePin(GPIOA, ContactorPin, OnOff);
}
