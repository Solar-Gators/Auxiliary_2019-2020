/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
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
#include <stdint.h>
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
TIM_HandleTypeDef htim2;
volatile bool btnSignal_flag = false;
volatile bool swtchSignal_flag = false;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//buttons
	if (GPIO_Pin == CPlus_in_Pin || GPIO_Pin == CMinus_in_Pin ||
		GPIO_Pin ==  Hazards_in_Pin || GPIO_Pin ==  Horn_in_Pin){
		HAL_TIM_Base_Start_IT(&htim2);
	}
	//switches
	else if (GPIO_Pin == LT_in_Pin || GPIO_Pin == RT_in_Pin ||
			GPIO_Pin == Headlights_in_Pin || GPIO_Pin == Regen_in_Pin){
		swtchSignal_flag = true;
	}
}

//For timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_TIM_Base_Stop_IT(&htim2);
	if (HAL_GPIO_ReadPin(CPlus_in_GPIO_Port, CPlus_in_Pin) == GPIO_PIN_SET){
		btnSignal_flag = true;
	}
	else if (HAL_GPIO_ReadPin(CMinus_in_GPIO_Port, CMinus_in_Pin) == GPIO_PIN_SET){
		btnSignal_flag = true;
	}
	else if (HAL_GPIO_ReadPin(Hazards_in_GPIO_Port, Hazards_in_Pin) == GPIO_PIN_SET){
		btnSignal_flag = true;
	}
	else if (HAL_GPIO_ReadPin(Horn_in_GPIO_Port, Horn_in_Pin) == GPIO_PIN_SET){
		btnSignal_flag = true;
	}

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t cruiseToggle = 0;
	uint8_t cplusToggle = 0;
	uint8_t cminusToggle = 0;
	uint8_t hornToggle = 0;
	uint8_t hazardsToggle = 0;
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
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  AUX_MESSAGE_0 auxHandler0;
  auxHandler0.SetupReceive(nullptr);
  SUBSYSTEM_DATA_MODULE::StartCAN(); //removed &hcan in parameter?

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  //=====[ BUTTONS ]===== (toggle)
	  if (btnSignal_flag){
		  btnSignal_flag = false;
		//_____[Cruise in]_____
		if (HAL_GPIO_ReadPin(Cruise_in_GPIO_Port, Cruise_in_Pin) == GPIO_PIN_SET){
			
			//Need to figure out where to send cruise data******************************************
			//auxHandler0.txData.???????? = cruiseToggle;     
			cruiseToggle = !cruiseToggle;
			HAL_GPIO_ReadPin(CruiseLED_out_GPIO_Port, CruiseLED_out_Pin, cruiseToggle);
		}
		//_____[Cruise +]_____
		if (HAL_GPIO_ReadPin(CPlus_in_GPIO_Port, CPlus_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.cplusOn = cplusToggle;
			cplusToggle = !cplusToggle;
		}
		//_____[Cruise -]_____
		else if (HAL_GPIO_ReadPin(CMinus_in_GPIO_Port, CMinus_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.cminusOn = cminusToggle;
			cminusToggle = !cminusToggle;
		}
		//_____[Hazards]_____
		else if (HAL_GPIO_ReadPin(Hazards_in_GPIO_Port, Hazards_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.hazardsOn = hazardsToggle;
			hazardsToggle = !hazardsToggle;
			HAL_GPIO_ReadPin(Hazards_out_GPIO_Port, Hazards_out_Pin, hazardsToggle);
		}
		//_____[Horn]_____
		else if (HAL_GPIO_ReadPin(Horn_in_GPIO_Port, Horn_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.hornOn = hornToggle;
			hornToggle = !hornToggle;
		}
	  }
	//=====[ SWITCHES ]===== (check on or off)
	if(swtchSignal_flag){ //triggered on change of a signal
		//_____[Left Turn]_____
		if (HAL_GPIO_ReadPin(LT_in_GPIO_Port, LT_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.leftOn = true;
			HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, 1);
		}
		else{
			auxHandler0.txData.leftOn = false;
			HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, 0);
		}
		//_____[Right Turn]_____
		if (HAL_GPIO_ReadPin(RT_in_GPIO_Port, RT_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.rightOn = true;
			HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, 1);
		}
		else{
			auxHandler0.txData.rightOn = false;
			HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, 0);
		}
		//_____[Headlights]_____
		if (HAL_GPIO_ReadPin(Headlights_in_GPIO_Port, Headlights_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.headlightsOn = true;
		}
		else{
			auxHandler0.txData.headlightsOn = false;
		}
		//_____[Regen]_____
		if (HAL_GPIO_ReadPin(Regen_in_GPIO_Port, Regen_in_Pin) == GPIO_PIN_SET){
			auxHandler0.txData.regenOn = true;
			HAL_GPIO_WritePin(Regen_out_GPIO_Port, Regen_out_Pin, 1);
		}
		else{
			auxHandler0.txData.regenOn = false;
			HAL_GPIO_WritePin(Regen_out_GPIO_Port, Regen_out_Pin, 0);
		}
	  }
		auxHandler0.txData.headlightsOn = true;
		auxHandler0.txData.rightOn = true;
	  auxHandler0.SendData();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  htim2.Init.Prescaler = 48000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
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
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(CruiseLED_out_GPIO_Port, CruiseLED_out_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : CruiseLED_out_Pin */
	  GPIO_InitStruct.Pin = CruiseLED_out_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(CruiseLED_out_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : Horn_in_Pin Reserved1_Pin */
	  GPIO_InitStruct.Pin = Horn_in_Pin|Reserved1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : Reserved0_Pin */
	  GPIO_InitStruct.Pin = Reserved0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Reserved0_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : Regen_in_Pin */
	  GPIO_InitStruct.Pin = Regen_in_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Regen_in_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : Headlights_in_Pin */
	  GPIO_InitStruct.Pin = Headlights_in_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(Headlights_in_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : Hazards_in_Pin CMinus_in_Pin CPlus_in_Pin RT_in_Pin
	                           LT_in_Pin */
	  GPIO_InitStruct.Pin = Hazards_in_Pin|CMinus_in_Pin|CPlus_in_Pin|RT_in_Pin
	                          |LT_in_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

	  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

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
