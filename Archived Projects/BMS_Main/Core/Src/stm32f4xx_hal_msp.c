#include "main.h"

void HAL_MspInit(void) {
	HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}
