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
  * Mitsuba emulator board. This board simulates how the mitsuba
  * would respond to the Motor controller board when that sends message requests
  *
  * SUMMARY
  * This acts as the mitsuba by sending a message back with the info asked for.
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


/**
 * SG
 * Flags to alert the main code when a new CAN message has been received
 */
static bool mc_request_flag = 0;
static bool newInput_CAN = 0;

/* USER CODE BEGIN PV */

/**
 * SG
 *
 * Initialize the phony dummy data. We will be sending these back to the motor controller
 * Note: I modified the drivers so that we are able to send these RX frames. With the default
 * drivers, we would not be able to even send these messages on a board.
 */
static MITSUBA_DRIVER_RX_FRAME_0 mcFrame0(MC_RX_FRAME0_ID);
static MITSUBA_DRIVER_RX_FRAME_1 mcFrame1(MC_RX_FRAME1_ID);
static MITSUBA_DRIVER_RX_FRAME_2 mcFrame2(MC_RX_FRAME2_ID);

/*
 * SG
 * Placeholder for the TX request message and packet that will come from the Motor controller.
 */
static MITSUBA_DRIVER_TX_RL_MESSAGE mcRequest(MC_REQUEST_ID);
static MITSUBA_DRIVER_TX_RL_DATA_PACKET mcRequest_packet;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

// ----------------------------
// --- ISR PROTOTYPE(S) -------
// ----------------------------
/**
 *
 */

/**
 * SG
 * Prototypes for the callback functions.
 */
void MC_Request_Receive_Callback(SUBSYSTEM_DATA_MODULE*);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  mcFrame0.txData.battVoltage=0;
  mcFrame0.txData.battCurrent=0;
  mcFrame0.txData.battCurrentDir=0;
  mcFrame0.txData.motorCurrentPkAvg=0;
  mcFrame0.txData.FETtemp=0;
  mcFrame0.txData.motorRPM=0;
  mcFrame0.txData.PWMDuty=0;
  mcFrame0.txData.LeadAngle=0;

  mcFrame1.txData.powerMode=0;
  mcFrame1.txData.MCmode=0;
  mcFrame1.txData.AcceleratorPosition=0;
  mcFrame1.txData.regenVRposition=0;
  mcFrame1.txData.digitSWposition=0;
  mcFrame1.txData.outTargetVal=0;
  mcFrame1.txData.driveActStat=0;
  mcFrame1.txData.regenStat=0;
  
  mcFrame2.txData.adSensorError=0;
  mcFrame2.txData.motorCurrSensorUError=0;
  mcFrame2.txData.motorCurrSensorWError=0;
  mcFrame2.txData.fetThermError=0;
  mcFrame2.txData.battVoltSensorError=0;
  mcFrame2.txData.battCurrSensorError=0;
  mcFrame2.txData.battCurrSensorAdjError=0;
  mcFrame2.txData.motorCurrSensorAdjError=0;
  mcFrame2.txData.accelPosError=0;
  mcFrame2.txData.contVoltSensorError=0;
  mcFrame2.txData.powerSystemError=0;
  mcFrame2.txData.overCurrError=0;
  mcFrame2.txData.overVoltError=0;
  mcFrame2.txData.overCurrLimit=0;
  mcFrame2.txData.motorSystemError=0;
  mcFrame2.txData.motorLock=0;
  mcFrame2.txData.hallSensorShort=0;
  mcFrame2.txData.hallSensorOpen=0;
  mcFrame2.txData.overHeatLevel=0;
  
  /**
   * SG
   * Configure/enable the callback functions. Will run whenever the mitsuba emulator receives a message request from MC
   *
   * Also, don't forget to start CAN!
   */
  mcRequest.SetupReceive(MC_Request_Receive_Callback);
  SUBSYSTEM_DATA_MODULE::StartCAN();

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(newInput_CAN)
		{
			//reset flag
			newInput_CAN = false;

			if (mc_request_flag)
			{
				mc_request_flag = false;
				if (mcRequest_packet.requestFrame0)
				{
					mcFrame0.SendData();
					mcFrame0.txData.motorRPM += 100;
					mcFrame0.txData.motorRPM %= 1000;
				}
				if (mcRequest_packet.requestFrame1)
				{
					mcFrame1.SendData();
					mcFrame1.txData.AcceleratorPosition += 10;
					mcFrame1.txData.AcceleratorPosition %= 100;
				}
				if (mcRequest_packet.requestFrame2)
				{
					mcFrame2.SendData();
					mcFrame2.txData.accelPosError = !mcFrame2.txData.accelPosError;
				}
			}
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


/**
 * SG
 * The callback that runs automatically whenever we receive a new message from
 * Mitsuba. One for each type of dataframe. The method "GetOldestDataPacket" sets
 * the input flag to true.
 *
 */
void MC_Request_Receive_Callback(SUBSYSTEM_DATA_MODULE*) {
	if(!mcRequest.isFifoEmpty())
		mcRequest_packet = mcRequest.GetOldestDataPacket(&mc_request_flag);
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
