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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "aux-data-module.hpp"
#include "subsystem-data-module.hpp"
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
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
AUX_MESSAGE_0_DATA_PACKET auxPacket;

static bool newCruiseInput = false;

static bool recharge = false;

static bool newCANPacket = false;
static bool cruiseActive = false;
static bool chargeTripActive = false;

static int tickPrecharge = 0;
static int tickCoil = 0;
static int tickRegen = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void AuxReceive(SUBSYSTEM_DATA_MODULE*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
//  MX_CAN_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  recharge = true;
  HAL_TIM_Base_Start_IT(&htim2);

  AUX_MESSAGE_0 aux0;
  aux0.SetupReceive(AuxReceive);

  SUBSYSTEM_DATA_MODULE::StartCAN(&hcan);

  uint8_t initializeDAC[3] = {0x70, 0x00, 0x00};	// set supply (5V) as reference

  HAL_GPIO_WritePin(RegenSS_GPIO_Port, RegenSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &initializeDAC[0], 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(RegenSS_GPIO_Port, RegenSS_Pin, GPIO_PIN_SET);
  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &initializeDAC[0], 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_SET);
  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

  uint8_t turnOnDAC[3] = {0x30, 0xFF, 0xF0};
  uint8_t turnOffDAC[3] = {0x40, 0x00, 0x00};

  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &turnOnDAC[0], 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_SET);
  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if(chargeTripActive)
//	  {
//		  HAL_GPIO_WritePin(GPIOA, MC_COIL_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOA, MPPTPRE_Pin, GPIO_PIN_RESET);
//		  HAL_TIM_Base_Stop_IT(&htim2);
//		  __HAL_TIM_SET_COUNTER(&htim2, 0);
//
//		  tickPrecharge = 0;
//		  tickCoil = 0;
//	  }
//	  else if(!chargeTripActive && recharge)
//	  {
//		  HAL_GPIO_WritePin(GPIOA, MPPTPRE_Pin, GPIO_PIN_SET);
//		  HAL_TIM_Base_Start_IT(&htim2);
//	  }
//	  if(newCANPacket && !aux0.isFifoEmpty())
//	  {
//		  newCANPacket = false;
//		  auxPacket = aux0.GetOldestDataPacket(nullptr);
//	  }
//	  // test
//	  if(newCruiseInput)
//	  {
//		  newCruiseInput = false;
//		  if(cruiseActive)
//		  {
//			  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_RESET);
//			  HAL_SPI_Transmit(&hspi2, &turnOnDAC[0], 3, 10);
//			  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_SET);
//		  }
//		  else
//		  {
//			  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_RESET);
//			  HAL_SPI_Transmit(&hspi2, &turnOffDAC[0], 3, 10);
//			  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_SET);
//		  }
//	  }

	  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_Transmit(&hspi2, turnOnDAC, sizeof(turnOnDAC), HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(CruiseSS_GPIO_Port, CruiseSS_Pin, GPIO_PIN_SET);
	  while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MCPRE_Pin|MPPTPRE_Pin|MC_COIL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MPPT_COIL_GPIO_Port, MPPT_COIL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RegenBrk_Pin|RegenSS_Pin|CruiseSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MCPRE_Pin MPPTPRE_Pin MC_COIL_Pin */
  GPIO_InitStruct.Pin = MCPRE_Pin|MPPTPRE_Pin|MC_COIL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MPPT_COIL_Pin */
  GPIO_InitStruct.Pin = MPPT_COIL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MPPT_COIL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGE_TRIP_Pin */
  GPIO_InitStruct.Pin = CHARGE_TRIP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CHARGE_TRIP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RegenBrk_Pin RegenSS_Pin CruiseSS_Pin */
  GPIO_InitStruct.Pin = RegenBrk_Pin|RegenSS_Pin|CruiseSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CRUISE_Pin */
  GPIO_InitStruct.Pin = CRUISE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CRUISE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == CRUISE_Pin)
	{
		cruiseActive = HAL_GPIO_ReadPin(CRUISE_GPIO_Port, CRUISE_Pin);
		newCruiseInput = true;
	}
	else if(GPIO_Pin == CHARGE_TRIP_Pin)
	{
		chargeTripActive = !HAL_GPIO_ReadPin(CHARGE_TRIP_GPIO_Port, CHARGE_TRIP_Pin); // charge trip is low true
		if(chargeTripActive) recharge = false;
		else recharge = true;
	}
	else __NOP();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(recharge)
		{
			if(tickPrecharge >= 50)
			{
				recharge = false;
				tickPrecharge = 0;
				HAL_GPIO_WritePin(GPIOA, MCPRE_Pin|MPPTPRE_Pin, GPIO_PIN_RESET);
			}
			else tickPrecharge++;
			if(tickCoil >= 48)
			{
				tickCoil = 0;
				HAL_GPIO_WritePin(GPIOA, MC_COIL_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC, MPPT_COIL_Pin, GPIO_PIN_SET);
			}
			else tickCoil++;
		}
		if(cruiseActive)
		{
			if(tickRegen >= 10)
			{

			}
			else tickRegen++;
		}
	}
	else if(htim->Instance == TIM3)
	{
		HAL_TIM_Base_Stop_IT(&htim3);
	}
	else __NOP();
}
void AuxReceive(SUBSYSTEM_DATA_MODULE*)
{
	newCANPacket = true;
}
/* USER CODE END 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
