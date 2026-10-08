/**
  ******************************************************************************
  * @file    mpu6500.c
  * @brief   MPU6500 SPI driver for DJI Development Board Type A
  *
  * SPI5: SCK=PF7, MISO=PF8, MOSI=PF9, CS=PF6
  ******************************************************************************
  */

#include "spi.h"
#include "mpu6500.h"

enum
{
    MPU6500_PWR_MGMT_1 = 0x6B,
    MPU6500_ACCEL_XOUT_H = 0x3B,
    MPU6500_GYRO_XOUT_H = 0x43,
    MPU6500_WHO_AM_I = 0x75,
    MPU6500_WHO_AM_I_VALUE = 0x70,
    MPU6500_TIMEOUT = 10,
    MPU6500_CAL_SAMPLES = 100
};

static const float MPU6500_GYRO_SENS = 16.4f;
static const float MPU6500_GYRO_TC = 0.02f;

static uint8_t MPU6500_Ready;
static int16_t Gyro_Bias_X;
static int16_t Gyro_Bias_Y;
static int16_t Gyro_Bias_Z;
static float Yaw_Angle;
static float Gyro_Z_Filt;
static uint32_t Last_Update_Ms;
static uint8_t Update_Time_Valid;

static void MPU6500_CsLow(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
}

static void MPU6500_CsHigh(void)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
}

static HAL_StatusTypeDef MPU6500_WriteReg(uint8_t Reg, uint8_t Data)
{
    HAL_StatusTypeDef Status;
    uint8_t Tx;
    uint8_t Rx;

    MPU6500_CsLow();

    Tx = Reg & 0x7F;
    Status = HAL_SPI_TransmitReceive(&hspi5, &Tx, &Rx, 1, MPU6500_TIMEOUT);
    if (Status == HAL_OK)
    {
        Tx = Data;
        Status = HAL_SPI_TransmitReceive(&hspi5, &Tx, &Rx, 1, MPU6500_TIMEOUT);
    }

    MPU6500_CsHigh();
    return Status;
}

static HAL_StatusTypeDef MPU6500_ReadRegs(uint8_t Reg, uint8_t *pBuf, uint16_t Len)
{
    HAL_StatusTypeDef Status;
    uint8_t Tx;
    uint8_t Rx;

    MPU6500_CsLow();

    Tx = Reg | 0x80;
    Status = HAL_SPI_TransmitReceive(&hspi5, &Tx, &Rx, 1, MPU6500_TIMEOUT);
    if (Status == HAL_OK)
    {
        Status = HAL_SPI_Receive(&hspi5, pBuf, Len, MPU6500_TIMEOUT);
    }

    MPU6500_CsHigh();
    return Status;
}

static HAL_StatusTypeDef MPU6500_ReadReg(uint8_t Reg, uint8_t *pData)
{
    return MPU6500_ReadRegs(Reg, pData, 1);
}

static int16_t MPU6500_ToInt16(uint8_t High, uint8_t Low)
{
    return (int16_t)(((uint16_t)High << 8) | Low);
}

static void MPU6500_GyroCalibrate(void)
{
    int32_t SumX = 0;
    int32_t SumY = 0;
    int32_t SumZ = 0;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint16_t i;

    for (i = 0; i < MPU6500_CAL_SAMPLES; i++)
    {
        MPU6500_ReadRawGyro(&gx, &gy, &gz);
        SumX += gx;
        SumY += gy;
        SumZ += gz;
        HAL_Delay(2);
    }

    Gyro_Bias_X = (int16_t)(SumX / MPU6500_CAL_SAMPLES);
    Gyro_Bias_Y = (int16_t)(SumY / MPU6500_CAL_SAMPLES);
    Gyro_Bias_Z = (int16_t)(SumZ / MPU6500_CAL_SAMPLES);
}

void MPU6500_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t ID = 0;

    MPU6500_Ready = 0;

    __HAL_RCC_GPIOF_CLK_ENABLE();
    MPU6500_CsHigh();

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi5) != HAL_OK)
    {
        return;
    }

    MPU6500_CsHigh();
    HAL_Delay(10);

    if (MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x80) != HAL_OK)
    {
        return;
    }
    HAL_Delay(100);

    if (MPU6500_WriteReg(MPU6500_PWR_MGMT_1, 0x01) != HAL_OK)
    {
        return;
    }
    HAL_Delay(10);

    MPU6500_WriteReg(0x6C, 0x00);
    MPU6500_WriteReg(0x19, 0x00);
    MPU6500_WriteReg(0x1A, 0x03);
    MPU6500_WriteReg(0x1B, 0x18);
    MPU6500_WriteReg(0x1C, 0x00);
    MPU6500_WriteReg(0x1D, 0x03);
    MPU6500_WriteReg(0x23, 0x00);
    MPU6500_WriteReg(0x6A, 0x00);
    MPU6500_WriteReg(0x24, 0x00);
    MPU6500_WriteReg(0x68, 0x07);
    HAL_Delay(10);
    MPU6500_WriteReg(0x68, 0x00);

    if (MPU6500_ReadReg(MPU6500_WHO_AM_I, &ID) != HAL_OK)
    {
        return;
    }
    if (ID != MPU6500_WHO_AM_I_VALUE)
    {
        return;
    }

    MPU6500_GyroCalibrate();
    MPU6500_Ready = 1;
    MPU6500_ResetYaw();
}

uint8_t MPU6500_ReadID(void)
{
    uint8_t ID = 0;

    MPU6500_ReadReg(MPU6500_WHO_AM_I, &ID);
    return ID;
}

void MPU6500_ReadRawGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];

    if (MPU6500_ReadRegs(MPU6500_GYRO_XOUT_H, buf, 6) != HAL_OK)
    {
        return;
    }

    *gx = MPU6500_ToInt16(buf[0], buf[1]);
    *gy = MPU6500_ToInt16(buf[2], buf[3]);
    *gz = MPU6500_ToInt16(buf[4], buf[5]);
}

void MPU6500_ReadRawAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];

    if (MPU6500_ReadRegs(MPU6500_ACCEL_XOUT_H, buf, 6) != HAL_OK)
    {
        return;
    }

    *ax = MPU6500_ToInt16(buf[0], buf[1]);
    *ay = MPU6500_ToInt16(buf[2], buf[3]);
    *az = MPU6500_ToInt16(buf[4], buf[5]);
}

void MPU6500_UpdateYaw(void)
{
    int16_t gx;
    int16_t gy;
    int16_t gz;
    uint32_t NowMs;
    float DtS;
    float Alpha;
    float GzDps;

    if (MPU6500_Ready == 0)
    {
        return;
    }

    MPU6500_ReadRawGyro(&gx, &gy, &gz);
    (void)gx;
    (void)gy;

    NowMs = HAL_GetTick();
    if (Update_Time_Valid == 0)
    {
        Last_Update_Ms = NowMs;
        Update_Time_Valid = 1;
        return;
    }
    if (NowMs == Last_Update_Ms)
    {
        return;
    }

    DtS = (float)(NowMs - Last_Update_Ms) / 1000.0f;
    Last_Update_Ms = NowMs;
    if (DtS > 0.1f)
    {
        DtS = 0.1f;
    }

    GzDps = (float)(gz - Gyro_Bias_Z) / MPU6500_GYRO_SENS;
    Alpha = DtS / (MPU6500_GYRO_TC + DtS);
    Gyro_Z_Filt += Alpha * (GzDps - Gyro_Z_Filt);
    Yaw_Angle += Gyro_Z_Filt * DtS-0.00025;

    if (Yaw_Angle > 180.0f)
    {
        Yaw_Angle -= 360.0f;
    }
    if (Yaw_Angle < -180.0f)
    {
        Yaw_Angle += 360.0f;
    }
}

float MPU6500_GetYaw(void)
{
    return Yaw_Angle;
}

float MPU6500_GetGyroZ_DPS(void)
{
    return Gyro_Z_Filt;
}

void MPU6500_ResetYaw(void)
{
    Yaw_Angle = 0.0f;
    Gyro_Z_Filt = 0.0f;
    Last_Update_Ms = HAL_GetTick();
    Update_Time_Valid = 1;
}

uint8_t MPU6500_IsReady(void)
{
    return MPU6500_Ready;
}
