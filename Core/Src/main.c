/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6500.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DT7_control 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
  uint16_t ecd;//编码器值
  int16_t speed;//速度
  int16_t current;//电流
  uint8_t tem;//温度
}Motor_data;
Motor_data Motor[6];//1到4是底盘电机，5是水平云台电机，6是垂直云台电机
int Fact[4];//读取遥控器值
int horizontal_angle;//水平电机编码器值
int vertical_angle;//垂直编码器值
uint8_t Rx_uart[18];//从遥控器读取的原始数据
int x;//遥控器水平值 -660到660
int y;//遥控器垂直值，x,y管云台
int P;//摇杆水平值
int Q;//摇杆垂直值，P,Q管底盘
int Flag_x;
int Flag_y;
float  a;//对应x发送给电机值
float b;//对应y发送给电机值
float c,d;//底盘发送的速度值c是Vx，d是Vy
PID_TypeDef yaw;//偏航角闭环参数
PID_TypeDef encoder;//垂直编码器闭环参数
PID_TypeDef h_encoder;//水平编码器闭环参数
PID_TypeDef motor[4];
float Yaw;//实际偏航角
float target_yaw;//目标偏航角
float target_encoder;//目标位置
float target_yaw_encoder;//水平目标位置(打杆时累加)
float target_y_encoder;//垂直目标位置(打杆时累加)
float k=0.0062;//摇杆增加系数
int o;//用来设置开机垂直目标值只用一次
float K;//麦轮系数
float Wz=0;//麦轮Z轴角速度(归一化系数，和c/d同量级)，由DT7左拨杆S1给值
float Target_speed[4];//四个底盘电机目标转速
int16_t Out[4];
int rx_left;
int ry_left;
int rx_right;
int ry_right;
int S1;//DT7左边拨杆: 1=上 2=中 3=下
int16_t Speed_1;
int16_t Speed_2;
int16_t Speed_3;
int16_t Speed_4;
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
  MX_CAN1_Init();
  MX_SPI5_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  BOARD_PowerOutputsInit();
  BOARD_PowerOutputSet(2U, ENABLE);//开启A板24V输出

HAL_CAN_Start( &hcan1);
  HAL_CAN_Start( &hcan2);
  __HAL_UART_CLEAR_PEFLAG(&huart1);
  (void)HAL_UARTEx_ReceiveToIdle_IT(&huart1, Rx_uart ,sizeof(Rx_uart));
  (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  (void)HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
MPU6500_Init() ;
  Yaw = MPU6500_GetYaw();
  target_yaw = MPU6500_GetYaw();



  yaw.Kp =250;
  yaw.Kd=1.5;
  yaw.integral_control =5000;
  encoder.Kp =40;
  encoder.Kd =0.5;
  encoder.integral_control =5000;
  h_encoder.Kp =40;
  h_encoder.Kd =0.5;
  h_encoder.integral_control =5000;
  motor[0].Kp = 3.0f;
  motor[0].Ki = 0.05f;
  motor[0].Kd = 0.0f;
  motor[0].integral_control = 10000.0f;
  motor[1].Kp = 3.0f;
  motor[1].Ki = 0.05f;
  motor[1].Kd = 0.0f;
  motor[1].integral_control = 10000.0f;
  motor[2].Kp = 3.0f;
  motor[2].Ki = 0.05f;
  motor[2].Kd = 0.0f;
  motor[2].integral_control = 10000.0f;
  motor[3].Kp = 3.0f;
  motor[3].Ki = 0.05f;
  motor[3].Kd = 0.0f;
  motor[3].integral_control = 10000.0f;
  o=vertical_angle ;
  target_encoder=o;
  (void)HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MPU6500_UpdateYaw();
    Yaw = MPU6500_GetYaw();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

float PID(PID_TypeDef* pid, float target, float actual)
{
  pid->err = target - actual;

  if (pid == &yaw)
  {
    if (pid->err > 180.0f)
    {
      pid->err -= 360.0f;
    }
    if (pid->err < -180.0f)
    {
      pid->err += 360.0f;
    }
  }

  if (pid == &encoder || pid == &h_encoder)
  {
    if (pid->err > 4096.0f)
    {
      pid->err -= 8192.0f;
    }
    if (pid->err < -4096.0f)
    {
      pid->err += 8192.0f;
    }
  }

  pid->integral += pid->err;
  if (pid->integral > pid->integral_control)
  {
    pid->integral = pid->integral_control;
  }
  if (pid->integral < -pid->integral_control)
  {
    pid->integral = -pid->integral_control;
  }

  float output = pid->Kp * pid->err + pid->integral * pid->Ki
               + pid->Kd * (pid->err - pid->err_last);
  pid->err_last = pid->err;
  return output;
}

void CAN_Send(uint32_t id,int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{

  uint8_t data[8];
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  uint32_t mailbox = 0U;

  data[0] = (uint8_t)(v1 >> 8);
  data[1] = (uint8_t)v1;
  data[2] = (uint8_t)(v2 >> 8);
  data[3] = (uint8_t)v2;
  data[4] = (uint8_t)(v3 >> 8);
  data[5] = (uint8_t)v3;
  data[6] = (uint8_t)(v4 >> 8);
  data[7] = (uint8_t)v4;


  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;
if (id==0x1FF) {
  (void)HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}
  if (id==0x200) {
    (void)HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &mailbox);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  /* 回调是 CAN1/CAN2 共用的，必须用传进来的 hcan：
     读错实例会让 CAN2 的 FIFO0 永远排不空，RF0N 一直置位，
     CPU 反复进出 CAN2 中断，把优先级更低的 TIM4 饿死。 */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }
  if (hcan->Instance == CAN1)
  {
    if (rx_header.StdId == 0x205)
    {
      horizontal_angle = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    }
    else if (rx_header.StdId == 0x206)
    {
      vertical_angle = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    }

  }

  if (hcan->Instance == CAN2) {
     if (rx_header.StdId == 0x201) {
      Motor[0].ecd=(rx_data[0]<<8)|rx_data[1];
      Motor[0].speed=(rx_data[2]<<8)|rx_data[3];
      Motor[0].current=(rx_data[4]<<8)|rx_data[5];
      Motor[0].tem=rx_data[6];
    }//左上
    else if (rx_header.StdId == 0x202) {
      Motor[1].ecd=(rx_data[0]<<8)|rx_data[1];
      Motor[1].speed=(rx_data[2]<<8)|rx_data[3];
      Motor[1].current=(rx_data[4]<<8)|rx_data[5];
      Motor[1].tem=rx_data[6];

    }//右上
    else if (rx_header.StdId == 0x203) {
      Motor[2].ecd=(rx_data[0]<<8)|rx_data[1];
      Motor[2].speed=(rx_data[2]<<8)|rx_data[3];
      Motor[2].current=(rx_data[4]<<8)|rx_data[5];
      Motor[2].tem=rx_data[6];
    }//左下
    else if (rx_header.StdId == 0x204) {
      Motor[3].ecd=(rx_data[0]<<8)|rx_data[1];
      Motor[3].speed=(rx_data[2]<<8)|rx_data[3];
      Motor[3].current=(rx_data[4]<<8)|rx_data[5];
      Motor[3].tem=rx_data[6];
    }//右下
  }




}

void DBUS_code(void)
{

  Fact[0] = (Rx_uart[1] << 8 | Rx_uart[0]) & 0x07FF;
  Fact[1] = (Rx_uart[2] << 5 | Rx_uart[1] >> 3) & 0x07FF;
  Fact[2] = (Rx_uart[2] >> 6 | Rx_uart[3] << 2 | Rx_uart[4] << 10) & 0x07FF;
  Fact[3] = (Rx_uart[4] >> 1 | Rx_uart[5] << 7) & 0x07FF;
  rx_left=Fact[2];
  ry_left=Fact[3];
  rx_right=Fact[0];
  ry_right=Fact[1];
  S1 = (Rx_uart[5] >> 4) & 0x03;//DT7左边拨杆(S1)在bit44~45，即第6字节的bit4~5
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  if (Size == sizeof(Rx_uart))
  {
    DBUS_code();
    x = -Fact[2] + 1024;
    y = -Fact[3]+1024;
    Q=(-Fact[0]+1024)*0.3;
    P=(-Fact[1]+1024)*0.3;

    /* 左拨杆控制自转：上档反转，中档不转，下档正转 */
    if (S1 == 1)
    {
      Wz = -0.1f;
    }
    else if (S1 == 2)
    {
      Wz = 0.0f;
    }
    else if (S1 == 3)
    {
      Wz = 0.1f;
    }
  }

  if (x > DT7_control || x < -DT7_control)
  {
    if (Flag_x == 0)
    {
      target_yaw_encoder = horizontal_angle;//从静止切入打杆，目标对齐当前编码器
    }
    Flag_x = 1;
  }
  else
  {
    if (Flag_x == 1)
    {
      yaw.integral = 0;
      yaw.err_last = 0;
      target_yaw = Yaw;
    }
    Flag_x = 0;
  }

  if (y > DT7_control || y < -DT7_control)
  {
    if (Flag_y == 0)
    {
      target_y_encoder = vertical_angle;//从静止切入打杆，目标对齐当前编码器
    }
    Flag_y = 1;
  }
  else
  {
    if (Flag_y == 1)
    {
      encoder.integral = 0;
      encoder.err_last = 0;
      target_encoder = vertical_angle;
    }
    Flag_y = 0;
  }

  __HAL_UART_CLEAR_PEFLAG(huart);
  (void)HAL_UARTEx_ReceiveToIdle_IT(huart, Rx_uart, sizeof(Rx_uart));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  __HAL_UART_CLEAR_PEFLAG(huart);
  (void)HAL_UARTEx_ReceiveToIdle_IT(huart, Rx_uart, sizeof(Rx_uart));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  int16_t horizontal_out;
  int16_t vertical_out;

  if (htim->Instance != TIM4)
  {
    return;
  }

  if (Flag_x == 1)
  {
    target_yaw_encoder += x*k;//打杆：摇杆量当编码器目标增量
    a = PID(&h_encoder, target_yaw_encoder, horizontal_angle);
  }
  else
  {
    a = PID(&yaw, target_yaw, Yaw);
  }

  if (Flag_y == 1)
  {
    target_y_encoder += y*k;
    b = PID(&encoder, target_y_encoder, vertical_angle);
  }
  else
  {
    b = PID(&encoder, target_encoder, vertical_angle);
  }

  /* 云台输出限幅：GM6020 电压模式满量程 ±30000，先给 ±10000，不够再加 */
  if (a > 10000.0f)  { a = 10000.0f; }
  if (a < -10000.0f) { a = -10000.0f; }
  if (b > 10000.0f)  { b = 10000.0f; }
  if (b < -10000.0f) { b = -10000.0f; }

  horizontal_out = (int16_t)a;
  vertical_out = (int16_t)b;

  c = -(float)P / 660  ;//Vy
  d = -(float)Q / 660  ;//Vx
Target_speed[0]=(d+c+Wz)*2000;//左前

  Target_speed[1]=-(d-c-Wz)*2000;//右前

  Target_speed[2]=(d-c+Wz)*2000;//左后

Target_speed[3]=-(d+c-Wz)*2000;//右后


  Speed_1=Motor[0].speed;//右前
 Speed_2 =Motor[1].speed;//左前
  Speed_3 =Motor[2].speed;//左后
  Speed_4 =Motor[3].speed;//右后
  float LF=PID(&motor[0], Target_speed[0],Motor[1].speed );
  float RF=PID(&motor[1], Target_speed[1],Motor[2].speed );
  float LB=PID(&motor[2], Target_speed[2],Motor[0].speed );
  float RB=PID(&motor[3], Target_speed[3],Motor[3].speed );
  if (RF>10000) {
    RF=10000;
  }
  if (RF<-10000) {
    RF=-10000;
  }


  if (LF>10000) {
    LF=10000;
  }
  if (LF<-10000) {
    LF=-10000;
  }

  if (LB>10000) {
    LB=10000;
  }
  if (LB<-10000) {
    LB=-10000;
  }

  if (RB>10000) {
    RB=10000;
  }
  if (RB<-10000) {
    RB=-10000;
  }
  Out[1]=(int16_t)LF;
  Out[2]=(int16_t)RF;
  Out[0]=(int16_t)LB;
  Out[3]=(int16_t)RB;

  //
  // if ((horizontal_angle < 3000 && horizontal_out < 0)
  //     || (horizontal_angle > 6000 && horizontal_out > 0))
  // {
  //   horizontal_out = 0;
  // }
  // if ((vertical_angle < 800 && vertical_out < 0)
  //     || (vertical_angle > 8000 && vertical_out > 0))
  // {
  //   vertical_out = 0;
  // }

  CAN_Send(0x1FF,horizontal_out, vertical_out, 0, 0);
 CAN_Send(0x200,Out[0] ,Out[1], Out[2], Out[3]);
}

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
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
