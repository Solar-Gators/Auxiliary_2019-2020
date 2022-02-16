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
  *
  * SOLAR GATORS CAN EXAMPLE
  * Motor Controller TX board. Demonstrates the implementation of sending MC
  * requests to the Mitsuba.
  * Mitsuba Request TX messages will be sent every 1 millisecond to obtain the
  * Mitsuba status (RPM, throttle position, etc).
  *
  * SUMMARY
  * This board sends a message to Mitsuba to say "give me info!"
  * The mitsuba sends a message back with the info asked for.
  * This board handles the reception of the message.
  *
  * Crucial information will be indicated by comments starting with "SG"
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/**
*	SG
*	Includes the MC_TX and RX messages and packet class. AUX_MESSAGE_0 is
*	a child of the base class, SUBSYSTEM_DATA_MODULE
*/
#include "subsystem-can-driver/subsystem-data-module.hpp"
#include "subsystem-can-driver/mitsuba-driver-data-module.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * SG
 *
 * Defines for the specific CAN Address codes needed to communicate with Mitsuba
 * When declaring a MITSUBA_DRIVER Frame (Rx frames 0-2 or Tx), input these IDs to the
 * object constructor.
 */
#define MC_REQUEST_ID 		0x08F89540
#define MC_RX_FRAME0_ID		0x08850225
#define MC_RX_FRAME1_ID		0x08950225
#define MC_RX_FRAME2_ID		0x08A50225


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


/**
 * SG
 * Flags to alert the main code when a new CAN message has been received
 */
static bool mc_0_flag = 0;
static bool mc_1_flag = 0;
static bool mc_2_flag = 0;
static bool newInput_CAN = 0;

/* USER CODE BEGIN PV */

/**
 * SG
 * Initialize the Mitsuba Tx message. This is the "request" message that asks the Mitsuba
 * to send information back to us. Note the ID parameter that we pass into the constructor.
 * This ID is REQUIRED for the Mitsuba to acknowledge that we requested a message.
 * We will select what data we want from the Mitsuba later.
 */
static MITSUBA_DRIVER_TX_RL_MESSAGE mcRequest(MC_REQUEST_ID);

/**
 * SG
 * Initialize the Mitsuba Rx placeholder messages. These are objects to hold the messages that the Mitsuba
 * sends back to us. The message will be stored and decoded into the Data_Packet, and we
 * can read the contents later.
 * The receiving of the Mitsuba messages is seen later in the code (MC_Receive_callback).
 *
 * Since there is so much data from Mitsuba, there data is split into 3 data frames.
 * Here we'll create the 3 placeholder messages, one for each frame because their
 * structure differs.
 */
static MITSUBA_DRIVER_RX_FRAME_0 mcFrame0(MC_RX_FRAME0_ID);
static MITSUBA_DRIVER_RX_FRAME_0_DATA_PACKET mcFrame0Packet;
static MITSUBA_DRIVER_RX_FRAME_1 mcFrame1(MC_RX_FRAME1_ID);
static MITSUBA_DRIVER_RX_FRAME_1_DATA_PACKET mcFrame1Packet;
static MITSUBA_DRIVER_RX_FRAME_2 mcFrame2(MC_RX_FRAME2_ID);
static MITSUBA_DRIVER_RX_FRAME_2_DATA_PACKET mcFrame2Packet;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

// ----------------------------
// --- ISR PROTOTYPE(S) -------
// ----------------------------
/**
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/**
 *
 */

/**
 * SG
 * Prototypes for the callback functions.
 */
void MC_0_Receive_Callback(SUBSYSTEM_DATA_MODULE*);

void MC_1_Receive_Callback(SUBSYSTEM_DATA_MODULE*);

void MC_2_Receive_Callback(SUBSYSTEM_DATA_MODULE*);

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
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /**
   * SG
   * Select which information we want from Mitsuba. Since there's so much data available,
   * the data is split up in 3 frames. The details of what data is available in which
   * frame is described in "subsystem-can-driver/mitsuba-driver-data.cpp"
   *
   * For this example, we will request all 3 frames.
   */
  mcRequest.txData.requestFrame0 = true;
  mcRequest.txData.requestFrame1 = true;
  mcRequest.txData.requestFrame2 = true;


  /**
   * SG
   * Configure/enable the callback functions that will run when we receive the corresponding
   * dataframe from the Mitsuba. Since each of the 3 dataframes are unique, we need 3 different
   * callback functions.
   *
   * Also, don't forget to start CAN!
   */
  mcFrame0.SetupReceive(MC_0_Receive_Callback);
  mcFrame1.SetupReceive(MC_1_Receive_Callback);
  mcFrame2.SetupReceive(MC_2_Receive_Callback);
  SUBSYSTEM_DATA_MODULE::StartCAN();

  uint16_t motorRPM = 0;
  uint16_t AcceleratorPosition = 0;
  bool accelPosError = false;
  uint16_t sum = 0;

  mcRequest.SendData();

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(newInput_CAN)
		{
			//reset flag
			newInput_CAN = false;

			if (mc_0_flag)
			{
				mc_0_flag = false;
				motorRPM = mcFrame0Packet.motorRPM;
			}
			if (mc_1_flag)
			{
				mc_1_flag = false;
				AcceleratorPosition = mcFrame1Packet.AcceleratorPosition;
			}
			if (mc_2_flag)
			{
				mc_2_flag = false;
				accelPosError = mcFrame2Packet.accelPosError;
			}
			sum = motorRPM + AcceleratorPosition + accelPosError;
			// Useless code just used to suppress warnings.
			sum--;
			sum++;
		}
  }
  /* USER CODE END 3 */
}

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
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

/* USER CODE BEGIN 4 */

// -----------------------------
// --- ISR DEFINITION(S) -------
// -----------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// check which timer triggered interrupt
	if(htim->Instance == TIM2)
	{
		/**
		 * SG
		 * Send out a MC Tx message to ask Mitsuba for data
		 * This timer elapses every 1 millisecond
		 */
		mcRequest.SendData();
	}
	else __NOP();	// no operation
}

/**
 * SG
 * The callback that runs automatically whenever we receive a new message from
 * Mitsuba. One for each type of dataframe. The method "GetOldestDataPacket" sets
 * the input flag to true.
 *
 */
void MC_0_Receive_Callback(SUBSYSTEM_DATA_MODULE*) {
	if(!mcFrame0.isFifoEmpty())
		mcFrame0Packet = mcFrame0.GetOldestDataPacket(&mc_0_flag);
	newInput_CAN = true;
}
void MC_1_Receive_Callback(SUBSYSTEM_DATA_MODULE*) {
	if(!mcFrame1.isFifoEmpty())
		mcFrame1Packet = mcFrame1.GetOldestDataPacket(&mc_1_flag);
	newInput_CAN = true;
}
void MC_2_Receive_Callback(SUBSYSTEM_DATA_MODULE*) {
	if(!mcFrame2.isFifoEmpty())
		mcFrame2Packet = mcFrame2.GetOldestDataPacket(&mc_2_flag);
	newInput_CAN = true;
}

/* USER CODE END 4 */


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
