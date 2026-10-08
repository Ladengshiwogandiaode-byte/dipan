/**
  ******************************************************************************
  * @file    mpu6500.h
  * @brief   MPU6500 SPI driver for DJI Development Board Type A
  ******************************************************************************
  */

#pragma once

#include "stm32f4xx_hal.h"

void    MPU6500_Init(void);
void    MPU6500_ResetYaw(void);
void    MPU6500_UpdateYaw(void);
float   MPU6500_GetYaw(void);
float   MPU6500_GetGyroZ_DPS(void);
uint8_t MPU6500_ReadID(void);
void    MPU6500_ReadRawGyro(int16_t *gx, int16_t *gy, int16_t *gz);
void    MPU6500_ReadRawAccel(int16_t *ax, int16_t *ay, int16_t *az);
uint8_t MPU6500_IsReady(void);
