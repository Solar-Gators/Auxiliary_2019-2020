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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define hazardsBm 0x01
#define hazardsBp 0
#define headlightsBm 0x02
#define headlightsBp 1
#define leftBm 0x04
#define leftBp 2
#define rightBm 0x08
#define rightBp 3
#define cplusBm 0x10
#define cplusBp 4
#define cminusBm 0x20
#define cminusBp 5
#define hornBm 0x40
#define hornBp 6
#define regenBm 0x80
#define regenBp 7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
/* USER CODE BEGIN PV */
volatile uint8_t btnSignal_flag = 0x00;
volatile uint8_t swtchSignal_flag = 0x00;
volatile uint8_t CANupdateFlag = 0x00;
volatile uint8_t dataArray[8];
volatile uint8_t timerRunning = 0x00;
CAN_TxHeaderTypeDef pHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t TxMailbox;
uint8_t outData = 0x00;
uint8_t inData = 0x00;
CAN_FilterTypeDef sFilterConfig;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//buttons
	if (GPIO_Pin == CPlus_in_Pin || GPIO_Pin == CMinus_in_Pin ||
		GPIO_Pin ==  Hazards_in_Pin || GPIO_Pin ==  Horn_in_Pin || GPIO_Pin == Regen_in_Pin){
		HAL_TIM_Base_Start_IT(&htim2);
	}
	//switches
	else if (GPIO_Pin == LT_in_Pin || GPIO_Pin == RT_in_Pin ||
			GPIO_Pin == Headlights_in_Pin){
		swtchSignal_flag = 0x01;
	}
}

//For timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim == &htim2) {

		// stop the timer
		HAL_TIM_Base_Stop_IT(&htim2);

		if (HAL_GPIO_ReadPin(CPlus_in_GPIO_Port, CPlus_in_Pin) == GPIO_PIN_SET){
			btnSignal_flag = 0x01;
			dataArray[cplusBp] = 0x01;
		}
		else if (HAL_GPIO_ReadPin(CMinus_in_GPIO_Port, CMinus_in_Pin) == GPIO_PIN_SET){
			btnSignal_flag = 0x01;
			dataArray[cminusBp] = 0x01;
		}
		else if (HAL_GPIO_ReadPin(Hazards_in_GPIO_Port, Hazards_in_Pin) == GPIO_PIN_SET){
			btnSignal_flag = 0x01;
			dataArray[hazardsBp] = !dataArray[hazardsBp];
			HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_TogglePin(Hazards_out_GPIO_Port, Hazards_out_Pin);
		}
		else if (HAL_GPIO_ReadPin(Horn_in_GPIO_Port, Horn_in_Pin) == GPIO_PIN_SET){
			dataArray[hornBp] = 0x01;
			btnSignal_flag = 0x01;
		}
		else if (HAL_GPIO_ReadPin(Regen_in_GPIO_Port, Regen_in_Pin) == GPIO_PIN_SET){
			btnSignal_flag = 0x01;
			dataArray[regenBp] = !dataArray[regenBp];
			HAL_GPIO_TogglePin(Regen_out_GPIO_Port, Regen_out_Pin);
		}
		else if (HAL_GPIO_ReadPin(Cruise_in_GPIO_Port, Cruise_in_Pin) == GPIO_PIN_SET) {
			btnSignal_flag = 0x01;
			HAL_GPIO_TogglePin(CruiseLED_out_GPIO_Port, CruiseLED_out_Pin);
		}

		if (HAL_GPIO_ReadPin(Horn_in_GPIO_Port, Horn_in_Pin) == GPIO_PIN_RESET) {
			dataArray[hornBp] = 0x00;
			btnSignal_flag = 0x01;
		}

	} else if (htim == &htim3) {

		// turn signal timer, timer 3
		if (dataArray[hazardsBp]) {

			// hazards
			HAL_GPIO_TogglePin(LT_out_GPIO_Port, LT_out_Pin);
			HAL_GPIO_TogglePin(RT_out_GPIO_Port, RT_out_Pin);

		} else if (dataArray[leftBp]) {

			// left turn
			HAL_GPIO_TogglePin(LT_out_GPIO_Port, LT_out_Pin);

		} else if (dataArray[rightBp]) {

			// right turn
			HAL_GPIO_TogglePin(RT_out_GPIO_Port, RT_out_Pin);

		}

	}

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_init(void) {

	// configure the outgoing message
	pHeader.DLC = 1;
	  pHeader.IDE = CAN_ID_STD;
	  pHeader.RTR = CAN_RTR_DATA;

	// this is the Aux team's CAN ID
	  pHeader.StdId = 0x3FF;

	  sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;

	  // The CAN filter is set to only receive from CAN messages of identifier 0x3FF, or 1023.
	  // This is the auxiliary CAN identifier for Sunbreaker.
	  sFilterConfig.FilterIdHigh=0x3FF;
	  sFilterConfig.FilterIdLow=0x3FF;
	  sFilterConfig.FilterMaskIdHigh=0;
	  sFilterConfig.FilterMaskIdLow=0;
	  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterActivation=ENABLE;
	  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

	  HAL_CAN_Start(&hcan);
	  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

}
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
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  CAN_init();
  btnSignal_flag = 0x01;
  swtchSignal_flag = 0x01;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // TODO: HANDLE CPLUS, CMINUS, AND HORN WHEN THEY ARE TRUE
	  if (btnSignal_flag || swtchSignal_flag) {

		  //update switches in memory
		  dataArray[leftBp] = HAL_GPIO_ReadPin(LT_in_GPIO_Port, LT_in_Pin);
		  dataArray[rightBp] = HAL_GPIO_ReadPin(RT_in_GPIO_Port, RT_in_Pin);
		  dataArray[headlightsBp] = HAL_GPIO_ReadPin(Headlights_in_GPIO_Port, Headlights_in_Pin);

		  // mark the CAN flag
		  CANupdateFlag = 0x01;

		  // clear the other flags
		  btnSignal_flag = 0x00;
		  swtchSignal_flag = 0x00;

		  outData = 0x00;

		  // update outData
		  if (dataArray[hazardsBp]) {
			  outData |= hazardsBm;
		  }
		  if (dataArray[headlightsBp]) {
			  outData |= headlightsBm;
		  }
		  if (dataArray[leftBp]) {
			  outData |= leftBm;
		  }
		  if (dataArray[rightBp]) {
			  outData |= rightBm;
		  }
		  if (dataArray[cplusBp]) {
			  outData |= cplusBm;
		  }
		  if (dataArray[cminusBp]) {
			  outData |= cminusBm;
		  }
		  if (dataArray[hornBp]) {
			  outData |= hornBm;
		  }
		  if (dataArray[regenBp]) {
			  outData |= regenBm;
		  }

//		  outData |= (hazardsBm & dataArray[hazardsBp]);
//		  outData |= (headlightsBm & dataArray[headlightsBp]);
//		  outData |= (leftBm & dataArray[leftBp]);
//		  outData |= (rightBm & dataArray[rightBp]);
//		  outData |= (cplusBm & dataArray[cplusBp]);
//		  outData |= (cminusBm & dataArray[cminusBp]);
//		  outData |= (hornBm & dataArray[hornBp]);
//		  outData |= (regenBm & dataArray[regenBp]);

	  }

	  // Only send a new TX message if the outgoing mailbox is empty
	  if ((HAL_CAN_IsTxMessagePending(&hcan,TxMailbox) == 0) && (CANupdateFlag)) {

		  // turn off the flag
		  CANupdateFlag = 0x00;

		  HAL_CAN_AddTxMessage(&hcan, &pHeader, &outData, &TxMailbox);

		  // clear buttons that should only send data once, aka cplus and cminus
		  dataArray[cplusBp] = 0x00;
		  dataArray[cminusBp] = 0x00;
		  outData &= ~cplusBm;
		  outData &= ~cminusBm;

	  }

	  // START A TIMER FOR HAZARDS OR TURN SIGNAL
	  	if (dataArray[hazardsBp]) {

	  		if (!timerRunning) {

	  			// write pin states
				HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_SET);
				HAL_TIM_Base_Start_IT(&htim3);
				timerRunning = 0x01;

	  		}

		} else if (dataArray[leftBp]) {

			HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_RESET);

			if (!timerRunning) {

				// left turn
				HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_SET);
				HAL_TIM_Base_Start_IT(&htim3);
				timerRunning = 0x01;

			}

		} else if (dataArray[rightBp]) {

			HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_RESET);

			if (!timerRunning) {

				// right turn
				HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_SET);
				HAL_TIM_Base_Start_IT(&htim3);
				timerRunning = 0x01;

			}

		} else {

		  // stop timer 3
		  timerRunning = 0x00;
		  HAL_TIM_Base_Stop_IT(&htim3);
		  // clear LEDs
		  HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_RESET);

		}

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
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
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
  htim3.Init.Prescaler = 1024;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 26000;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CruiseLED_out_Pin|Regen_out_Pin|LT_out_Pin|RT_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RST_Pin|LCD_DC_Pin|Hazards_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CruiseLED_out_Pin Regen_out_Pin LT_out_Pin RT_out_Pin */
  GPIO_InitStruct.Pin = CruiseLED_out_Pin|Regen_out_Pin|LT_out_Pin|RT_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Cruise_in_Pin Reserved0_Pin Regen_in_Pin */
  GPIO_InitStruct.Pin = Cruise_in_Pin|Reserved0_Pin|Regen_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_MISO_Pin LCD_MOSI_Pin */
  GPIO_InitStruct.Pin = LCD_MISO_Pin|LCD_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_DC_Pin Hazards_out_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_DC_Pin|Hazards_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Throttle_in_Pin */
  GPIO_InitStruct.Pin = Throttle_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Throttle_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_SCK_Pin */
  GPIO_InitStruct.Pin = LCD_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(LCD_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Reserved1_Pin */
  GPIO_InitStruct.Pin = Reserved1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Reserved1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Horn_in_Pin */
  GPIO_InitStruct.Pin = Horn_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Horn_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Headlights_in_Pin */
  GPIO_InitStruct.Pin = Headlights_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Headlights_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Hazards_in_Pin CMinus_in_Pin CPlus_in_Pin */
  GPIO_InitStruct.Pin = Hazards_in_Pin|CMinus_in_Pin|CPlus_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RT_in_Pin LT_in_Pin */
  GPIO_InitStruct.Pin = RT_in_Pin|LT_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
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
