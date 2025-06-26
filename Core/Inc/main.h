/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

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
#define LED_Default_Pin GPIO_PIN_13
#define LED_Default_GPIO_Port GPIOC
#define RE2_Pin GPIO_PIN_4
#define RE2_GPIO_Port GPIOA
#define Valve_2_Pin GPIO_PIN_0
#define Valve_2_GPIO_Port GPIOB
#define Valve_1_Pin GPIO_PIN_1
#define Valve_1_GPIO_Port GPIOB
#define RE1_Pin GPIO_PIN_8
#define RE1_GPIO_Port GPIOA
#define CAN_RS_Pin GPIO_PIN_15
#define CAN_RS_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_3
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_4
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_5
#define LED_3_GPIO_Port GPIOB
#define LED_4_Pin GPIO_PIN_6
#define LED_4_GPIO_Port GPIOB
#define LED_5_Pin GPIO_PIN_7
#define LED_5_GPIO_Port GPIOB
#define LED_6_Pin GPIO_PIN_8
#define LED_6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
