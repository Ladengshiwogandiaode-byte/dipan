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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float err_last;
  float integral_control;
  float err;
}PID_TypeDef;
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
void BOARD_PowerOutputsInit(void);
void BOARD_PowerOutputSet(uint8_t output, FunctionalState state);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_OUT1_CTRL_Pin GPIO_PIN_2
#define POWER_OUT1_CTRL_GPIO_Port GPIOH
#define POWER_OUT2_CTRL_Pin GPIO_PIN_3
#define POWER_OUT2_CTRL_GPIO_Port GPIOH
#define POWER_OUT3_CTRL_Pin GPIO_PIN_4
#define POWER_OUT3_CTRL_GPIO_Port GPIOH
#define POWER_OUT4_CTRL_Pin GPIO_PIN_5
#define POWER_OUT4_CTRL_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
