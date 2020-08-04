/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for Measurements */
osThreadId_t MeasurementsHandle;
const osThreadAttr_t Measurements_attributes = {
  .name = "Measurements",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* Definitions for StateMachine */
osThreadId_t StateMachineHandle;
const osThreadAttr_t StateMachine_attributes = {
  .name = "StateMachine",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Led */
osThreadId_t LedHandle;
const osThreadAttr_t Led_attributes = {
  .name = "Led",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for BinSem */
osSemaphoreId_t BinSemHandle;
const osSemaphoreAttr_t BinSem_attributes = {
  .name = "BinSem"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartMeasurements(void *argument);
void StartStateMachine(void *argument);
void StartLed(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}

Battery BatteryPack;

void BatteryInit(void) {
	BatteryPack.voltage = 0;
	BatteryPack.current = 0;
	BatteryPack.temperature = 0;
	for (int i = 0; i < numCells; i++) {
		Cell cell = {
			.voltage = 0,
			.temperature = 0
		};
		BatteryPack.cells[i] = cell;
	}
}

const char* StateNames[] = {
  "Initialize",
  "Idle",
  "Precharging",
  "Run",
  "Stop",
  "Sleep",
  "NormalDangerFault",
  "SevereDangerFault",
  "Charging",
  "Charged",
  "Balancing"
};

State_t CurrentState = Initialize;
State_t OldState = Sleep;

StateMachine SM[11] = {
    {Initialize, InitializeEvent},
	{Idle, IdleEvent},
	{Precharging, PrechargingEvent},
	{Run, RunEvent},
	{Stop, StopEvent},
	{Sleep, SleepEvent},
	{NormalDangerFault, NormalDangerFaultEvent},
	{SevereDangerFault, SevereDangerFaultEvent},
	{Charging, ChargingEvent},
	{Charged, ChargedEvent},
	{Balancing, BalancingEvent}
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  BatteryInit();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BinSem */
  BinSemHandle = osSemaphoreNew(1, 1, &BinSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Measurements */
  MeasurementsHandle = osThreadNew(StartMeasurements, NULL, &Measurements_attributes);

  /* creation of StateMachine */
  StateMachineHandle = osThreadNew(StartStateMachine, NULL, &StateMachine_attributes);

  /* creation of Led */
  LedHandle = osThreadNew(StartLed, NULL, &Led_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Red_Pin|LED_Pin|LED_Green_Pin|LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Red_Pin LED_Pin LED_Green_Pin LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Red_Pin|LED_Pin|LED_Green_Pin|LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Contactor_Pin */
  GPIO_InitStruct.Pin = Contactor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Contactor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Charge_Pin Reset_Pin */
  GPIO_InitStruct.Pin = Charge_Pin|Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Start_Pin Stop_Pin */
  GPIO_InitStruct.Pin = Start_Pin|Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
State_t InitializeEvent(void) {
	osDelay(3000);
	return Idle;
}

State_t IdleEvent(void) {
	osThreadResume(MeasurementsHandle);
	HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, 0);
	if (HAL_GPIO_ReadPin(Start_GPIO_Port, Start_Pin)) {
		return Precharging;
	} else if (HAL_GPIO_ReadPin(Charge_GPIO_Port, Charge_Pin)) {
		return Charging;
	} else if (HAL_GPIO_ReadPin(Stop_GPIO_Port, Stop_Pin)) {
		return Sleep;
	} else {
		return Idle;
	}
}

State_t PrechargingEvent(void) {
	osDelay(3000);
	return Run;
}

State_t RunEvent(void) {
	HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, 1);
	if (HAL_GPIO_ReadPin(Stop_GPIO_Port, Stop_Pin)) {
		return Stop;
	} else {
		return Run;
	}
}

State_t StopEvent(void) {
	HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, 0);
	if (HAL_GPIO_ReadPin(Reset_GPIO_Port, Reset_Pin)) {
		return Idle;
	} else {
		return Stop;
	}
}

State_t SleepEvent(void) {
	osThreadSuspend(MeasurementsHandle);
	if (HAL_GPIO_ReadPin(Reset_GPIO_Port, Reset_Pin)) {
		return Idle;
	} else {
		return Sleep;
	}
}

State_t NormalDangerFaultEvent(void) {
	HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, 0);
	if (HAL_GPIO_ReadPin(Reset_GPIO_Port, Reset_Pin)) {
		return Idle;
	} else {
		return NormalDangerFault;
	}
}

State_t SevereDangerFaultEvent(void) {
	HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, 0);
	return SevereDangerFault;
}

State_t ChargingEvent(void) {
	HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, 1);
	if (BatteryPack.voltage > 51600) {
		return Charged;
	} else {
		return Charging;
	}
}

State_t ChargedEvent(void) {
	HAL_GPIO_WritePin(Contactor_GPIO_Port, Contactor_Pin, 0);
	if (HAL_GPIO_ReadPin(Reset_GPIO_Port, Reset_Pin)) {
		return Idle;
	} else {
		return Charged;
	}
}

State_t BalancingEvent(void) {
	return Balancing;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMeasurements */
/**
  * @brief  Function implementing the Measurements thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartMeasurements */
void StartMeasurements(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	/* Measure Battery Pack voltage, current, temperature every 1000 milliseconds */
	BatteryPack.voltage = 51800; // mV
	BatteryPack.current = 20000; // mA
	BatteryPack.temperature = 30; // ˚C
	char dataM[100];
	sprintf(dataM, "Voltage: %dmV,  Current: %dmA,  Temperature: %d˚C\r\n", BatteryPack.voltage, BatteryPack.current, BatteryPack.temperature);
	HAL_UART_Transmit(&huart2, (uint8_t*)dataM, strlen(dataM), 500);
	if (BatteryPack.voltage > SevereDangerVoltage || BatteryPack.current > SevereDangerCurrent || BatteryPack.temperature > SevereDangerTemperature) {
		CurrentState = SevereDangerFault;
	} else if (BatteryPack.voltage > NormalDangerVoltage || BatteryPack.current > NormalDangerCurrent || BatteryPack.temperature > NormalDangerTemperature) {
		CurrentState = NormalDangerFault;
	}
    osDelay(1000);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartStateMachine */
/**
* @brief Function implementing the StateMachine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStateMachine */
void StartStateMachine(void *argument)
{
  /* USER CODE BEGIN StartStateMachine */
  /* Infinite loop */
  for(;;)
  {
	if (OldState != CurrentState) {
		char dataState[100];
		sprintf(dataState, "Current State: %s\r\n", StateNames[CurrentState]);
		HAL_UART_Transmit(&huart2, (uint8_t*)dataState, strlen(dataState), 500);
	}
	OldState = CurrentState;
	CurrentState = (*SM[CurrentState].Event)();
	osDelay(100);
  }
  /* USER CODE END StartStateMachine */
}

/* USER CODE BEGIN Header_StartLed */
/**
* @brief Function implementing the Led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLed */
void StartLed(void *argument)
{
  /* USER CODE BEGIN StartLed */
  /* Infinite loop */
  for(;;)
  {
    LedOn(CurrentState);
    osDelay(500);
  }
  /* USER CODE END StartLed */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
