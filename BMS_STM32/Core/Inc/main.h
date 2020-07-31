/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef void(*pointer)();
typedef pointer(*State)();

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
State Initialize();
State Idle();
State Precharging();
State Run();
State Stop();
State Sleep();
State Charging();
State Balancing();
State Normal_Danger_Fault();
State Severe_Danger_Fault();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Red_Pin GPIO_PIN_1
#define LED_Red_GPIO_Port GPIOB
#define Contactor_Pin GPIO_PIN_8
#define Contactor_GPIO_Port GPIOA
#define Reset_Pin GPIO_PIN_11
#define Reset_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define Start_Pin GPIO_PIN_4
#define Start_GPIO_Port GPIOB
#define Stop_Pin GPIO_PIN_5
#define Stop_GPIO_Port GPIOB
#define LED_Green_Pin GPIO_PIN_6
#define LED_Green_GPIO_Port GPIOB
#define LED_Blue_Pin GPIO_PIN_7
#define LED_Blue_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
