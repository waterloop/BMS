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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
uint16_t Get_Voltage(void);
uint16_t Get_Current(void);
uint8_t Get_Temperature(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SAI1_MCK_Pin GPIO_PIN_2
#define SAI1_MCK_GPIO_Port GPIOE
#define AUDIO_RST_Pin GPIO_PIN_3
#define AUDIO_RST_GPIO_Port GPIOE
#define SAI1_FS_Pin GPIO_PIN_4
#define SAI1_FS_GPIO_Port GPIOE
#define SAI1_SCK_Pin GPIO_PIN_5
#define SAI1_SCK_GPIO_Port GPIOE
#define SAI1_SD_Pin GPIO_PIN_6
#define SAI1_SD_GPIO_Port GPIOE
#define MFX_IRQ_OUT_Pin GPIO_PIN_13
#define MFX_IRQ_OUT_GPIO_Port GPIOC
#define MAG_INT_Pin GPIO_PIN_1
#define MAG_INT_GPIO_Port GPIOC
#define MAG_DRDY_Pin GPIO_PIN_2
#define MAG_DRDY_GPIO_Port GPIOC
#define VLCD_Pin GPIO_PIN_3
#define VLCD_GPIO_Port GPIOC
#define JOY_CENTER_Pin GPIO_PIN_0
#define JOY_CENTER_GPIO_Port GPIOA
#define JOY_LEFT_Pin GPIO_PIN_1
#define JOY_LEFT_GPIO_Port GPIOA
#define JOY_RIGHT_Pin GPIO_PIN_2
#define JOY_RIGHT_GPIO_Port GPIOA
#define JOY_UP_Pin GPIO_PIN_3
#define JOY_UP_GPIO_Port GPIOA
#define JOY_DOWN_Pin GPIO_PIN_5
#define JOY_DOWN_GPIO_Port GPIOA
#define TEMPERATURE_Pin GPIO_PIN_4
#define TEMPERATURE_GPIO_Port GPIOC
#define CURRENT_Pin GPIO_PIN_5
#define CURRENT_GPIO_Port GPIOC
#define VOLTAGE_Pin GPIO_PIN_0
#define VOLTAGE_GPIO_Port GPIOB
#define POT_VCC_Pin GPIO_PIN_1
#define POT_VCC_GPIO_Port GPIOB
#define LD_R_Pin GPIO_PIN_2
#define LD_R_GPIO_Port GPIOB
#define AUDIO_DIN_Pin GPIO_PIN_7
#define AUDIO_DIN_GPIO_Port GPIOE
#define LD_G_Pin GPIO_PIN_8
#define LD_G_GPIO_Port GPIOE
#define AUDIO_CLK_Pin GPIO_PIN_9
#define AUDIO_CLK_GPIO_Port GPIOE
#define QSPI_CLK_Pin GPIO_PIN_10
#define QSPI_CLK_GPIO_Port GPIOE
#define QSPI_CS_Pin GPIO_PIN_11
#define QSPI_CS_GPIO_Port GPIOE
#define QSPI_D0_Pin GPIO_PIN_12
#define QSPI_D0_GPIO_Port GPIOE
#define QSPI_D1_Pin GPIO_PIN_13
#define QSPI_D1_GPIO_Port GPIOE
#define QSPI_D2_Pin GPIO_PIN_14
#define QSPI_D2_GPIO_Port GPIOE
#define QSPI_D3_Pin GPIO_PIN_15
#define QSPI_D3_GPIO_Port GPIOE
#define MFX_I2C_SLC_Pin GPIO_PIN_10
#define MFX_I2C_SLC_GPIO_Port GPIOB
#define MFX_I2C_SDA_Pin GPIO_PIN_11
#define MFX_I2C_SDA_GPIO_Port GPIOB
#define Contactor_Pin GPIO_PIN_12
#define Contactor_GPIO_Port GPIOB
#define Charge_Pin GPIO_PIN_13
#define Charge_GPIO_Port GPIOB
#define Start_Pin GPIO_PIN_14
#define Start_GPIO_Port GPIOB
#define Stop_Pin GPIO_PIN_15
#define Stop_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOD
#define Reset_Pin GPIO_PIN_9
#define Reset_GPIO_Port GPIOD
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_9
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define OTG_FS_OverCurrent_Pin GPIO_PIN_10
#define OTG_FS_OverCurrent_GPIO_Port GPIOC
#define OTG_FS_VBUS_Pin GPIO_PIN_11
#define OTG_FS_VBUS_GPIO_Port GPIOC
#define EXT_RST_Pin GPIO_PIN_0
#define EXT_RST_GPIO_Port GPIOD
#define MEMS_SCK_Pin GPIO_PIN_1
#define MEMS_SCK_GPIO_Port GPIOD
#define GYRO_INT1_Pin GPIO_PIN_2
#define GYRO_INT1_GPIO_Port GPIOD
#define MEMS_MISO_Pin GPIO_PIN_3
#define MEMS_MISO_GPIO_Port GPIOD
#define MEMS_MOSI_Pin GPIO_PIN_4
#define MEMS_MOSI_GPIO_Port GPIOD
#define USART_TX_Pin GPIO_PIN_5
#define USART_TX_GPIO_Port GPIOD
#define USART_RX_Pin GPIO_PIN_6
#define USART_RX_GPIO_Port GPIOD
#define GYRO_CS_Pin GPIO_PIN_7
#define GYRO_CS_GPIO_Port GPIOD
#define M3V3_REG_ON_Pin GPIO_PIN_3
#define M3V3_REG_ON_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define GYRO_INT2_Pin GPIO_PIN_8
#define GYRO_INT2_GPIO_Port GPIOB
#define XL_CS_Pin GPIO_PIN_0
#define XL_CS_GPIO_Port GPIOE
#define XL_INT_Pin GPIO_PIN_1
#define XL_INT_GPIO_Port GPIOE
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
