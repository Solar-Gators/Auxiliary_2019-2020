/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
TIM_HandleTypeDef htim3;

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef pHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t TxMailbox;
uint8_t outData = 0x00;
uint8_t inData = 0x00;

CAN_FilterTypeDef sFilterConfig;
uint8_t canFlag = 0x00;

uint8_t hazardsOn = 0x00;
uint8_t headlightsOn = 0x00;
uint8_t leftOn = 0x00;
uint8_t rightOn = 0x00;
uint8_t timerRunning = 0x00;
uint8_t mechBrakesOn = 0x00;
uint8_t regenOn = 0x00;
uint32_t value_adc;
//uint32_t *resultRegister = &ADC_DR_DATA;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void CAN_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc2) {

	value_adc = HAL_ADC_GetValue(&hadc);

}

void CAN_init(void) {

	// configure the outgoing message
	pHeader.DLC = 1;
	  pHeader.IDE = CAN_ID_STD;
	  pHeader.RTR = CAN_RTR_DATA;

	// this is the Aux team's CAN ID
	  pHeader.StdId = 0x3FF;

//	  pRxHeader.DLC = 1;
//	  	  pRxHeader.IDE = CAN_ID_STD;
//	  	  pRxHeader.RTR = CAN_RTR_DATA;
//	  	pRxHeader.StdId = 0x3FF;

	  sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;

	  // The CAN filter is set to only receive from CAN messages of identifier 0x3FF, or 1023.
	  // This is the auxiliary CAN identifier for Sunbreaker.
	  sFilterConfig.FilterIdHigh=0;
	  sFilterConfig.FilterIdLow=0x3FF;
	  sFilterConfig.FilterMaskIdHigh=0x0;
	  sFilterConfig.FilterMaskIdLow=0x0;
	  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterActivation=ENABLE;
	  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

	  HAL_CAN_Start(&hcan);
	  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
//  /* Prevent unused argument(s) compilation warning */
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, inData2);
//	canFlag = 0x01;
//
//  /* NOTE : This function Should not be modified, when the callback is needed,
//            the HAL_CAN_RxFifo0MsgPendingCallback could be implemented in the
//            user file
//   */
//}

//For timer interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if (htim == &htim2) {

		// turn signal timer, timer 2
		if (hazardsOn) {

			// hazards
			HAL_GPIO_TogglePin(LT_out_GPIO_Port, LT_out_Pin);
			HAL_GPIO_TogglePin(RT_out_GPIO_Port, RT_out_Pin);

		} else if (leftOn) {

			// left turn
			HAL_GPIO_TogglePin(LT_out_GPIO_Port, LT_out_Pin);

		} else if (rightOn) {

			// right turn
			HAL_GPIO_TogglePin(RT_out_GPIO_Port, RT_out_Pin);

		}

	} else {

		HAL_ADC_Start_IT(&hadc);

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  CAN_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	//MX_DMA_Init();
	MX_CAN_Init();
	MX_TIM2_Init();
	MX_ADC_Init();
	MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  // clear all outputs at startup
  HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Headlights_out_GPIO_Port, Headlights_out_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Brake_out_GPIO_Port, Brake_out_Pin, GPIO_PIN_RESET);

  // start timer 3
  HAL_TIM_Base_Start_IT(&htim3);

  // link DMA
  //HAL_ADC_Start_DMA(&hadc, (uint32_t*)&value_adc,10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // trigger ADC
	  //HAL_ADC_Start(&hadc);
	  //HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	  //raw = HAL_ADC_GetValue(&hadc);
	  // polling
	  // while (!HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0));

	  // read the ADC value from brake sensor
	  if (value_adc >= 20) {

		  mechBrakesOn = 0x01;
		  HAL_GPIO_WritePin(Brake_out_GPIO_Port, Brake_out_Pin, GPIO_PIN_SET);


	  } else {

		  mechBrakesOn = 0x00;

	  }

	  if (canFlag) {

		  // clear the flag
		  canFlag = 0x00;

		  // update variables
		  hazardsOn = inData & hazardsBm;
		  headlightsOn = inData & headlightsBm;
		  leftOn = inData & leftBm;
		  rightOn = inData & rightBm;
		  regenOn = inData & regenBm;

		  // turn on the brake lights if regen braking is enabled
		  if (regenOn) {

			  HAL_GPIO_WritePin(Brake_out_GPIO_Port, Brake_out_Pin, GPIO_PIN_SET);

		  }

		  if (hazardsOn) {

			  // check to see if the timer is running
			  if (!timerRunning) {

				  // clear turn signals
				  HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_RESET);

				  timerRunning = 0x01;
				  // start the timer
				  HAL_TIM_Base_Start_IT(&htim2);

			  }

		  } else if (leftOn | rightOn) {

			  if (!timerRunning) {

				  // clear turn signals
				  HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_RESET);

				  timerRunning = 0x01;
				  // start the timer
				  HAL_TIM_Base_Start_IT(&htim2);

			  }

		  } else {

			  // stop the timer
			  HAL_TIM_Base_Stop_IT(&htim2);
			  timerRunning = 0x00;

			  // clear turn signals
			  HAL_GPIO_WritePin(RT_out_GPIO_Port, RT_out_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(LT_out_GPIO_Port, LT_out_Pin, GPIO_PIN_RESET);

		  }

		  // update headlights
		  if (headlightsOn) {

			  HAL_GPIO_WritePin(Headlights_out_GPIO_Port, Headlights_out_Pin, GPIO_PIN_SET);

		  } else {

			  HAL_GPIO_WritePin(Headlights_out_GPIO_Port, Headlights_out_Pin, GPIO_PIN_RESET);

		  }

	  }

	  // check to see if brakes are released
	  if (!mechBrakesOn && !regenOn) {

		  HAL_GPIO_WritePin(Brake_out_GPIO_Port, Brake_out_Pin, GPIO_PIN_RESET);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_6B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hcan.Init.AutoWakeUp = ENABLE;
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
  htim2.Init.Prescaler = 1024;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 26000;
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
  htim3.Init.Prescaler = 48000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Brake_out_Pin|Headlights_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LT_out_Pin|RT_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Brake_out_Pin Headlights_out_Pin */
  GPIO_InitStruct.Pin = Brake_out_Pin|Headlights_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LT_out_Pin RT_out_Pin */
  GPIO_InitStruct.Pin = LT_out_Pin|RT_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
