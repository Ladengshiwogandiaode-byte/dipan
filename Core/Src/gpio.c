/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
/*
 * 开发板 A 的 4 路电源输出由 PH2~PH5 控制，MOS 低电平导通。
 */
void BOARD_PowerOutputsInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();

  GPIO_InitStruct.Pin = POWER_OUT1_CTRL_Pin | POWER_OUT2_CTRL_Pin |
                        POWER_OUT3_CTRL_Pin | POWER_OUT4_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* 上电先全部关断，等显式打开 */
  HAL_GPIO_WritePin(GPIOH, GPIO_InitStruct.Pin, GPIO_PIN_SET);
}

void BOARD_PowerOutputSet(uint8_t output, FunctionalState state)
{
  GPIO_PinState level = (state == ENABLE) ? GPIO_PIN_RESET : GPIO_PIN_SET;

  switch (output)
  {
    case 1U:
      HAL_GPIO_WritePin(POWER_OUT1_CTRL_GPIO_Port, POWER_OUT1_CTRL_Pin, level);
      break;
    case 2U:
      HAL_GPIO_WritePin(POWER_OUT2_CTRL_GPIO_Port, POWER_OUT2_CTRL_Pin, level);
      break;
    case 3U:
      HAL_GPIO_WritePin(POWER_OUT3_CTRL_GPIO_Port, POWER_OUT3_CTRL_Pin, level);
      break;
    case 4U:
      HAL_GPIO_WritePin(POWER_OUT4_CTRL_GPIO_Port, POWER_OUT4_CTRL_Pin, level);
      break;
    default:
      break;
  }
}

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, POWER_OUT1_CTRL_Pin|POWER_OUT2_CTRL_Pin|POWER_OUT3_CTRL_Pin|POWER_OUT4_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : POWER_OUT1_CTRL_Pin POWER_OUT2_CTRL_Pin POWER_OUT3_CTRL_Pin POWER_OUT4_CTRL_Pin */
  GPIO_InitStruct.Pin = POWER_OUT1_CTRL_Pin|POWER_OUT2_CTRL_Pin|POWER_OUT3_CTRL_Pin|POWER_OUT4_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
