/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/*
大齿轮是100:10 -> RigthSocket 地址0x02 Down
小的是50:10 -> LeftSocket 地址0x01 Up
PB2->节光电门
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Serial.h"
#include "Motor.h"
#include "OLED.h"
#include "Timer.h"
#include "math.h"
#include "Pid.h"
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

/* USER CODE BEGIN PV */
extern volatile uint8_t DownMotorLocation_Array[6];
extern volatile int32_t DownMotorLocation;
extern volatile int8_t DownMotorLocateDataGetFlag; // 下面电机接收数据标志位
extern volatile uint8_t UpMotorLocation_Array[6];
extern volatile int32_t UpMotorLocation;
extern volatile int8_t UpMotorLocateDataGetFlag; // 上面电机接收数据标志位

struct UltraSerial Usart1, Usart2, Usart3; // 初始化3种串口
struct Pid UpMotor_Pid_Black , DownMotor_Pid_Black ,
           UpMotor_Pid_Green , DownMotor_Pid_Green ,
           UpMotor_Pid_Red , DownMotor_Pid_Red ;

extern volatile int oledupdate_state;
// extern volatile int TurningUpdate_state;
// extern volatile int TurningDowndate_state;
extern volatile int date_state;
volatile float DownLocation; // x轴当前位置
volatile float UpLocation;   // y轴当前位置

extern float UpCurrentAngle;   // 记录当前角度，用于归零
extern float DownCurrentAngle; // 记录当前角度，用于归零

/*提取串口读到的数据*/
extern float fRxData[8];
extern double dRxData[4];
extern int32_t iRxData[8];

extern UART_DAT dat_Uart1;
/*提取光电门读取到的电平*/
volatile uint8_t PhotoelectricSensorStatus;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_EnableIT_RXNE(USART2);
    LL_USART_EnableIT_RXNE(USART3);

    HAL_TIM_Base_Start_IT(&htim3); // 使能定时器中断

    // LL_USART_ClearFlag_RXNE(USART2);
    // LL_USART_EnableIT_RXNE(USART2);

    OLED_Init();

    Serial_Registration(&Usart1, USART1);
    // Serial_SetLogLevel(&Usart1, LogInfo);
    Serial_Registration(&Usart2, USART2);
    Serial_SetLogLevel(&Usart2, LogShutDown); // PA2->RX PA3->TX CW顺时针 CCW逆时针 --->Down
    Serial_Registration(&Usart3, USART3);     // PB10 -> RX PB11 -> TX --->Up
    Serial_SetLogLevel(&Usart3, LogShutDown);

    PID_Init(&UpMotor_Pid_Black, 0.7f, 0.01f, 0.0f, 0.025f);
    PID_Init(&DownMotor_Pid_Black, 0.7f, 0.01f, 0.0f, 0.025f);
    PID_Init(&UpMotor_Pid_Green, 0.7f, 0.01f, 0.0f, 0.025f);
    PID_Init(&DownMotor_Pid_Green, 0.7f, 0.01f, 0.0f, 0.025f);
    PID_Init(&UpMotor_Pid_Red, 0.7f, 0.01f, 0.0f, 0.025f);
    PID_Init(&DownMotor_Pid_Red, 0.7f, 0.01f, 0.0f, 0.025f);

    uint8_t Header[] = {0xAA, 0x55};      // 包头
    float Data[2] = {723.2231, 412.4326}; // 3FA3D70A 4048F5C3
    UartVarInit();
    InitHardUart();
    // Serial_PackTranAgrDecide(&Usart1, 2, Header);
    // Serial_SendPacket_float(&Usart1, 2, &Data, 0x01);
    static int Turnstate = 0, avoid = 0;
    // Emm_V5_Pos_UpControl(&Usart1, 0x01, 0x01, 1850, 0, 800, false, false);
    // Serial_SendPacket_float(&Usart1 , 1 , Data , 0xFF );
    // AA 55 FF FF 03 04 C3 F5 48 40 FF
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
        if(HAL_GPIO_ReadPin( GPIOB,GPIO_PIN_1 ) == GPIO_PIN_RESET)//低电平有球
        {
            PhotoelectricSensorStatus = 1;
        }
        else if(HAL_GPIO_ReadPin( GPIOB,GPIO_PIN_1 ) == GPIO_PIN_SET)//高电平无球
        {
            PhotoelectricSensorStatus = 0;
        }
        OLED_ShowNum( 0 , 32 , PhotoelectricSensorStatus , 2 , OLED_6X8);
        // 发送字符串

        if (dat_Uart1.FLG)
        {
            Serial_DataDeal();
            OLED_ShowFloatNum(0, 56, fRxData[0], 3, 4, OLED_6X8);
            OLED_ShowFloatNum(64, 56, fRxData[1], 3, 4, OLED_6X8);
            
        }

        OLED_ShowHexNum(0, 0, DownMotorLocation_Array[0], 2, OLED_6X8);
        OLED_ShowHexNum(18, 0, DownMotorLocation_Array[1], 2, OLED_6X8);
        OLED_ShowHexNum(36, 0, DownMotorLocation_Array[2], 2, OLED_6X8);
        OLED_ShowHexNum(54, 0, DownMotorLocation_Array[3], 2, OLED_6X8);
        OLED_ShowHexNum(72, 0, DownMotorLocation_Array[4], 2, OLED_6X8);
        OLED_ShowHexNum(90, 0, DownMotorLocation_Array[5], 2, OLED_6X8);

        OLED_ShowHexNum(0, 8, UpMotorLocation_Array[0], 2, OLED_6X8);
        OLED_ShowHexNum(18, 8, UpMotorLocation_Array[1], 2, OLED_6X8);
        OLED_ShowHexNum(36, 8, UpMotorLocation_Array[2], 2, OLED_6X8);
        OLED_ShowHexNum(54, 8, UpMotorLocation_Array[3], 2, OLED_6X8);
        OLED_ShowHexNum(72, 8, UpMotorLocation_Array[4], 2, OLED_6X8);
        OLED_ShowHexNum(90, 8, UpMotorLocation_Array[5], 2, OLED_6X8);

        if (DownMotorLocateDataGetFlag == 1)
        {
            DownMotorLocateDataGetFlag = 0;

            if (DownMotorLocation_Array[0] == 0x00)
            {
                DownLocation = (DownMotorLocation * 360) / 65536 / 10;
            }
            else if (DownMotorLocation_Array[0] == 0x01)
            {
                DownLocation = -(DownMotorLocation * 360) / 65536 / 10;
            }

            OLED_ShowFloatNum(0, 16, DownLocation, 8, 2, OLED_6X8);
        }

        if (UpMotorLocateDataGetFlag == 1)
        {
            UpMotorLocateDataGetFlag = 0;

            if (UpMotorLocation_Array[0] == 0x00)
            {
                UpLocation = (UpMotorLocation * 360) / 65536 / 5;
            }
            else if (UpMotorLocation_Array[0] == 0x01)
            {
                UpLocation = -(UpMotorLocation * 360) / 65536 / 5;
            }

            OLED_ShowFloatNum(0, 24, UpLocation, 8, 2, OLED_6X8);
        }

        /*----------------------------中断控制部分----------------------------*/
        if (oledupdate_state == 1) // 屏幕刷新以及电机数据部分
        {
            oledupdate_state = 0;
            Emm_V5_GetCurrentLocation(&Usart2, DownAdr);

            Emm_V5_GetCurrentLocation(&Usart3, UpAdr);
            OLED_Update();
        }
        if (date_state == 1)
        {
            date_state = 0;
            Motor_TargetAngleControl();
        }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
