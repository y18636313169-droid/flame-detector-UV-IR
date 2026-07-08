/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ap_util.h"
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

#if defined(IR_TEST_MODE)
void TEST_SetIrEnabled(uint8_t en);
uint8_t TEST_GetIrEnabled(void);
void TEST_SetUvEnabled(uint8_t en);
uint8_t TEST_GetUvEnabled(void);
void TEST_InsertMarker(const char *msg);
#endif

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_OUT_3_Pin GPIO_PIN_0
#define IR_OUT_3_GPIO_Port GPIOC
#define IR_OUT_2_Pin GPIO_PIN_1
#define IR_OUT_2_GPIO_Port GPIOC
#define IR_OUT_1_Pin GPIO_PIN_2
#define IR_OUT_1_GPIO_Port GPIOC
#define ALM2_Pin GPIO_PIN_0
#define ALM2_GPIO_Port GPIOA
#define ALM1_Pin GPIO_PIN_1
#define ALM1_GPIO_Port GPIOA
#define CHK_MODE_Pin GPIO_PIN_2
#define CHK_MODE_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
