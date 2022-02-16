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
  *
  * SOLAR GATORS CAN EXAMPLE
  * Aux TX board. Demonstrates the implementation of sending Aux data messages
  * Aux TX messages will be sent every 10 milliseconds to toggle the headlights.
  * Crucial information will be indicated by comments starting with "SG"
  *
  * John has overwritten the definition of void CEC_CAN_IRQHandler(void). Delete
  * the auto-generated version of this method in stm32f0xx_it.c
  ******************************************************************************
  */
  
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/**
*	SG
*	Includes the AUX_MESSAGE_0 and packet class. AUX_MESSAGE_0 is 
*	a child of the base class, SUBSYSTEM_DATA_MODULE
*/
#include "subsystem-can-driver/subsystem-data-module.hpp"
#include "subsystem-can-driver/aux-data-module.hpp"

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

/* USER CODE BEGIN PV */
static bool newInput_CAN = false;

/**
*	SG
*	Initialize a Aux RX CAN placeholder object. This way we can access the
*	Aux RX fifo. Fifo explanation later on.
*
* 	For this example, this MUST happen globally e.g. outside of Main because
*   aux0 and aux0Packet will be referenced in a global function
*
*	Also, initialize an AUX_MESSAGE_0_DATA_PACKET. Data received will be
* 	stored here first, before any decoding happens.
*/
static AUX_MESSAGE_0 aux0;
static AUX_MESSAGE_0_DATA_PACKET aux0Packet;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/**
 *  SG
 *  Function prototype for the callback function. The name of this
 *  function is arbitrary. Later in the code we will "activate" this
 *  by assigning it to be the callback function for whenever we received
 *  any AUX message.
 */
void AUX_MotherReceive_Callback(SUBSYSTEM_DATA_MODULE*);

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
  //MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  /**
  *  SG
  *  Call SetupReceive to assign a function as the "callback" function for
  *  whenever we receive a new AUX Can message.
  *  Now, whenever we get a message, AUX_MotherReceive_Callback will run
  *  automatically.
  *
  *  Also, don't forget to StartCAN!
  */
  aux0.SetupReceive(AUX_MotherReceive_Callback);
  SUBSYSTEM_DATA_MODULE::StartCAN();
  
  uint16_t did_i_leave_my_headlights_on;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(newInput_CAN)
	{
		// Reset flag
		newInput_CAN = false;

		// Set a debug breakpoint here to see the magic
		if (aux0Packet.headlightsOn)
		{
			did_i_leave_my_headlights_on = 1;
		}
		else
		{
			did_i_leave_my_headlights_on = 0;
		}

		// useless code to supress warnings
		did_i_leave_my_headlights_on++;
		did_i_leave_my_headlights_on--;
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
// -----------------------------
// --- ISR DEFINITION(S) -------
// -----------------------------

/**
* SG 
* CRUCIAL STEP. This unloads data from the CAN Rx mailbox into
* aux0Packet. It also sets the flag newInput_CAN to true.
* Now, we can operate on this data in main, and we also have a flag that we
* received a new message.
*/
void AUX_MotherReceive_Callback(SUBSYSTEM_DATA_MODULE*)
{
	if(!aux0.isFifoEmpty())
		aux0Packet = aux0.GetOldestDataPacket(&newInput_CAN);
}

// ----------------------------------
// --- FUNCTION DEFINITION(S) -------
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
