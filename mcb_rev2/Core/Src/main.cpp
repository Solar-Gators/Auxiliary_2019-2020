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
#include "subsystem-data-module.hpp"
#include "aux-data-module.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum slave
{
	cruiseDAC = 0,
	regenDAC = 1,
} slave_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

// ------------------------------
// CAN-related objects
static AUX_MESSAGE_0_DATA_PACKET aux0Packet;
static AUX_MESSAGE_0 aux0;
// ------------------------------

// ------------------------------
// ISR-affected booleans and other variables
static bool active_CT = false;
static bool active_Cruise = false;

static bool newInput_CT	= false;
static bool newInput_Cruise = false;

static bool sysPrecharge = false;
static bool sysMCCoilActive = false;
static bool sysMPPTCoilActive = false;

static int tickPrecharge = 0;
static int tickCoil = 0;

static bool newInput_CAN = false;
// ------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

// ----------------------------------
// --- INTERRUPT PROTOTYPE(S) -------
// ----------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void AUX_MotherReceive_Callback(SUBSYSTEM_DATA_MODULE*);
// ----------------------------------

// ---------------------------------
// --- FUNCTION PROTOTYPE(S) -------
// ---------------------------------
int DAC_Write(slave_t slave, uint8_t *data);
// ---------------------------------

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
  uint8_t DAC_Init[3] = {0x70, 0x00, 0x00};		// Set external voltage reference (5V)
  uint8_t DAC_PowerOn[3] = {0x30, 0xFF, 0xF0};	// Power on DAC with max output
  uint8_t DAC_PowerOff[3] = {0x40, 0x00, 0x00};	// Power off DAC
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  aux0.SetupReceive(AUX_MotherReceive_Callback);
  SUBSYSTEM_DATA_MODULE::StartCAN();

  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  DAC_Write(cruiseDAC, DAC_Init);
  DAC_Write(regenDAC, DAC_Init);	// when is regen DAC used
  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(newInput_CT)
	{
		if(active_CT)
		{
			// Turn off MPPT coil
			HAL_GPIO_WritePin(MPPT_COIL_GPIO_Port, MC_COIL_Pin, GPIO_PIN_RESET);
			// Ensure MPPT precharge is off
			HAL_GPIO_WritePin(MPPT_PRE_GPIO_Port, MC_PRE_Pin, GPIO_PIN_RESET);

			// Reset timer
			HAL_TIM_Base_Stop_IT(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2, 0);

			// Reset ticks
			tickPrecharge = 0;
			tickCoil = 0;

			// Set MPPT Coil active bool
			sysMPPTCoilActive = false;
		}
		else if(!active_CT && sysPrecharge && !sysMPPTCoilActive) // maybe get rid of sysMPPTCoilActive
		{
			HAL_GPIO_WritePin(MPPT_PRE_GPIO_Port, MPPT_PRE_Pin, GPIO_PIN_SET);
			HAL_TIM_Base_Start_IT(&htim2);
		}
		newInput_CT = false;
	}
	if(newInput_CAN)
	{
		if(aux0Packet.regenOn && active_Cruise)
		{
			// Turn on regen brake
			HAL_GPIO_WritePin(REGEN_BRK_GPIO_Port, REGEN_BRK_Pin, GPIO_PIN_SET);
		}
		else if(aux0Packet.regenOn)
		{
			DAC_Write(regenDAC, DAC_PowerOn);
		}
		else
		{
			// might need to separate these two functionalities somehow
			// need to ask Stephen
			// Turn off regen brake
			HAL_GPIO_WritePin(REGEN_BRK_GPIO_Port, REGEN_BRK_Pin, GPIO_PIN_RESET);
			// turn off regen DAC
			DAC_Write(regenDAC, DAC_PowerOff);
		}
		newInput_CAN = false;
	}
	if(newInput_Cruise)
	{
		if(active_Cruise)
		{
			DAC_Write(cruiseDAC, DAC_PowerOn);
		}
		else
		{
			DAC_Write(cruiseDAC, DAC_PowerOff);
		}
		newInput_Cruise = false;
	}
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
  HAL_GPIO_WritePin(GPIOA, MC_PRE_Pin|MPPT_PRE_Pin|MC_COIL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MPPT_COIL_GPIO_Port, MPPT_COIL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, REGEN_BRK_Pin|SS_REGEN_Pin|SS_CRUISE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MC_PRE_Pin MPPT_PRE_Pin MC_COIL_Pin */
  GPIO_InitStruct.Pin = MC_PRE_Pin|MPPT_PRE_Pin|MC_COIL_Pin;
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

  /*Configure GPIO pins : REGEN_BRK_Pin SS_REGEN_Pin SS_CRUISE_Pin */
  GPIO_InitStruct.Pin = REGEN_BRK_Pin|SS_REGEN_Pin|SS_CRUISE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CRUISE_IN_Pin */
  GPIO_InitStruct.Pin = CRUISE_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CRUISE_IN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	// read pins and do relevant operations
	// before setting newInput
	if(GPIO_PIN == CHARGE_TRIP_Pin)
	{
		active_CT = !HAL_GPIO_ReadPin(CHARGE_TRIP_GPIO_Port, CHARGE_TRIP_Pin);
		if(active_CT) sysPrecharge = false;
		else sysPrecharge = true;
		newInput_CT = true;
	}
	else if(GPIO_PIN == CRUISE_IN_Pin)
	{
		active_Cruise = HAL_GPIO_ReadPin(CRUISE_IN_GPIO_Port, CRUISE_IN_Pin);
		newInput_Cruise = true;
	}
	else __NOP();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(sysPrecharge)
		{
			if(tickPrecharge >= 50)	// value is adjustable
			{
				tickPrecharge = 0;	// reset tick count
				// turn off precharge
				HAL_GPIO_WritePin(MC_PRE_GPIO_Port, MC_PRE_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MPPT_PRE_GPIO_Port, MPPT_PRE_Pin, GPIO_PIN_RESET);
				sysPrecharge = false;
			}
			else ++tickPrecharge;
		}
		else if(!sysMCCoilActive || !sysMPPTCoilActive)
		{
			if(tickCoil >= 48) // value is adjustable
			{
				tickCoil = 0;
				// turn on coils
				HAL_GPIO_WritePin(MC_COIL_GPIO_Port, MC_COIL_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MPPT_COIL_GPIO_Port, MPPT_COIL_Pin, GPIO_PIN_SET);
				sysMCCoilActive = true;
				sysMPPTCoilActive = true;
			}
			else ++tickCoil;
		}
	}
	else __NOP();
}
void AUX_MotherReceive_Callback(SUBSYSTEM_DATA_MODULE*)
{
	if(!aux0.isFifoEmpty())
		aux0Packet = aux0.GetOldestDataPacket(&newInput_CAN);
}

// ----------------------------------
// --- FUNCTION DEFINITION(S) -------
// ----------------------------------
int DAC_Write(slave_t slave, uint8_t *data)
{
	GPIO_TypeDef* currentPort;
	uint16_t currentPin;
	if(slave == cruiseDAC)
	{
		currentPort = SS_CRUISE_GPIO_Port;
		currentPin = SS_CRUISE_Pin;
	}
	else if(slave == regenDAC)
	{
		currentPort = SS_REGEN_GPIO_Port;
		currentPin = SS_REGEN_Pin;
	}
	else return -1;

	HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, data, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(currentPort, currentPin, GPIO_PIN_SET);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	return 0;
}
// ----------------------------------

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
