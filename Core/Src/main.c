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
大齿轮是100:10 -> RigthSocket 地址0x02
小的是50:10 -> LeftSocket 地址0x01
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Serial.h"
#include "Motor.h"
#include "OLED.h"
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
struct UltraSerial Usart1, Usart2 , Usart3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float GetDownLocation(void)
{
    float result;
    if(DownMotorLocateDataGetFlag == 1)
    {
        DownMotorLocateDataGetFlag = 0;
        
        if(DownMotorLocation_Array[0] == 0x00)
        { 
            result =  (DownMotorLocation * 360) / 65536;
        }
        else if(DownMotorLocation_Array[0] == 0x01)
        {
            result = -(DownMotorLocation * 360) / 65536;
            
        }
        OLED_ShowFloatNum(0 ,  32 , result , 8 , 1 ,  OLED_8X16);
        return result;
    }
    else return 0 ;
}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_EnableIT_RXNE(USART2);
    LL_USART_EnableIT_RXNE(USART3);

    // LL_USART_ClearFlag_RXNE(USART2);
    // LL_USART_EnableIT_RXNE(USART2);

    OLED_Init();

    uint8_t Header[] = {0xAA, 0x55}; // 包头
    float Data[1] = {3.14};          // 3FA3D70A 4048F5C3
    Serial_Registration(&Usart1, USART1);
    Serial_Registration(&Usart2, USART2); // PA2->RX PA3->TX
    Serial_Registration(&Usart3, USART3); // PA2->RX PA3->TX
    Serial_PackTranAgrDecide(&Usart1, 2, Header);//PB10 -> RX PB11 -> TX
    volatile float DownLocation;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // 发送字符串

        // HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_13 , GPIO_PIN_RESET);

        // Serial_Printf(&Usart1 , "Hello STM32!\r\n");

        Emm_V5_Pos_DownControl(&Usart2, 0x02, CW, 0, 0, false);

        // Serial_SendPacket_float(&Usart1 , 1 , Data , 0xFF );

        HAL_Delay(10);


        // OLED_ShowString(0, 0, "Hello World", OLED_8X16);

        Emm_V5_GetDownCurrentLocation(&Usart2 , 0x02);

        OLED_ShowHexNum(0, 0, DownMotorLocation_Array[0], 2, OLED_8X16);
        OLED_ShowHexNum(24, 0, DownMotorLocation_Array[1], 2, OLED_8X16);
        OLED_ShowHexNum(48, 0, DownMotorLocation_Array[2], 2, OLED_8X16);
        OLED_ShowHexNum(72, 0, DownMotorLocation_Array[3], 2, OLED_8X16);
        OLED_ShowHexNum(0, 16, DownMotorLocation_Array[4], 2, OLED_8X16);
        OLED_ShowHexNum(24, 16, DownMotorLocation_Array[5], 2, OLED_8X16);
        // OLED_ShowHexNum(48, 16, DownMotorLocation_Array[6], 2, OLED_8X16);
        DownLocation = GetDownLocation();
        OLED_ShowFloatNum(0, 48, DownLocation, 8, 5, OLED_8X16);
        OLED_Update();

        // // 延时
        // for (volatile uint32_t i = 0; i < 1000000; i++)
        //   ;

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
