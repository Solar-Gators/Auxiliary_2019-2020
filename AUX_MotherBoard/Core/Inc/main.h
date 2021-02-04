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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CruiseLED_out_Pin GPIO_PIN_0
#define CruiseLED_out_GPIO_Port GPIOC
#define Cruise_in_Pin GPIO_PIN_1
#define Cruise_in_GPIO_Port GPIOC
#define Cruise_in_EXTI_IRQn EXTI0_1_IRQn
#define LCD_MISO_Pin GPIO_PIN_2
#define LCD_MISO_GPIO_Port GPIOC
#define LCD_MOSI_Pin GPIO_PIN_3
#define LCD_MOSI_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_4
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_5
#define LCD_DC_GPIO_Port GPIOA
#define Hazards_out_Pin GPIO_PIN_7
#define Hazards_out_GPIO_Port GPIOA
#define Regen_out_Pin GPIO_PIN_4
#define Regen_out_GPIO_Port GPIOC
#define LT_out_Pin GPIO_PIN_5
#define LT_out_GPIO_Port GPIOC
#define Throttle_in_Pin GPIO_PIN_0
#define Throttle_in_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_1
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_10
#define LCD_SCK_GPIO_Port GPIOB
#define RT_out_Pin GPIO_PIN_6
#define RT_out_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Reserved1_Pin GPIO_PIN_15
#define Reserved1_GPIO_Port GPIOA
#define Reserved1_EXTI_IRQn EXTI4_15_IRQn
#define Reserved0_Pin GPIO_PIN_10
#define Reserved0_GPIO_Port GPIOC
#define Reserved0_EXTI_IRQn EXTI4_15_IRQn
#define Horn_in_Pin GPIO_PIN_11
#define Horn_in_GPIO_Port GPIOC
#define Regen_in_Pin GPIO_PIN_12
#define Regen_in_GPIO_Port GPIOC
#define Regen_in_EXTI_IRQn EXTI4_15_IRQn
#define Headlights_in_Pin GPIO_PIN_2
#define Headlights_in_GPIO_Port GPIOD
#define Headlights_in_EXTI_IRQn EXTI2_3_IRQn
#define Hazards_in_Pin GPIO_PIN_3
#define Hazards_in_GPIO_Port GPIOB
#define Hazards_in_EXTI_IRQn EXTI2_3_IRQn
#define CMinus_in_Pin GPIO_PIN_4
#define CMinus_in_GPIO_Port GPIOB
#define CMinus_in_EXTI_IRQn EXTI4_15_IRQn
#define CPlus_in_Pin GPIO_PIN_5
#define CPlus_in_GPIO_Port GPIOB
#define CPlus_in_EXTI_IRQn EXTI4_15_IRQn
#define RT_in_Pin GPIO_PIN_6
#define RT_in_GPIO_Port GPIOB
#define RT_in_EXTI_IRQn EXTI4_15_IRQn
#define LT_in_Pin GPIO_PIN_7
#define LT_in_GPIO_Port GPIOB
#define LT_in_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
