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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* Operation Mode and Definitions */
// #define LED_ENABLED
#define TESTING_MODE
#define numCells 14

/* Private types */
typedef enum {
  Initialize,
  Idle,
  Precharging,
  Run,
  Stop,
  Sleep,
  NormalDangerFault,
  SevereDangerFault,
  Charging,
  Charged,
  Balancing
} State_t;

typedef struct {
	uint16_t voltage;
	uint8_t temperature;
} Cell;

typedef struct Battery {
	uint16_t voltage;
	uint16_t current;
	uint8_t temperature;
	Cell cells[numCells];
} Battery;

typedef State_t (*pfEvent)(void);

typedef struct {
	State_t State;
	pfEvent Event;
} StateMachine;

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
State_t InitializeEvent(void);
State_t IdleEvent(void);
State_t PrechargingEvent(void);
State_t RunEvent(void);
State_t StopEvent(void);
State_t SleepEvent(void);
State_t NormalDangerFaultEvent(void);
State_t SevereDangerFaultEvent(void);
State_t ChargingEvent(void);
State_t ChargedEvent(void);
State_t BalancingEvent(void);

void BatteryInit(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RED_Pin GPIO_PIN_0
#define RED_GPIO_Port GPIOA
#define LED_CTRL_Pin GPIO_PIN_0
#define LED_CTRL_GPIO_Port GPIOB
#define Contactor_Pin GPIO_PIN_8
#define Contactor_GPIO_Port GPIOA
#define Stop_Pin GPIO_PIN_9
#define Stop_GPIO_Port GPIOA
#define Reset_Pin GPIO_PIN_10
#define Reset_GPIO_Port GPIOA
#define Charge_Pin GPIO_PIN_3
#define Charge_GPIO_Port GPIOB
#define Start_Pin GPIO_PIN_4
#define Start_GPIO_Port GPIOB
#define BLUE_Pin GPIO_PIN_5
#define BLUE_GPIO_Port GPIOB
#define GREEN_Pin GPIO_PIN_7
#define GREEN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
// Arbitrary values, get values from the Electrical team.
#define SevereDangerVoltage 55000
#define NormalDangerVoltage 53000
#define SevereDangerCurrent 30000
#define NormalDangerCurrent 25000
#define SevereDangerTemperature 60
#define NormalDangerTemperature 50

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
