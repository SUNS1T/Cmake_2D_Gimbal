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
#include "Timer.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define UpAdr 0x01
#define DownAdr 0x02
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
struct UltraSerial Usart1, Usart2, Usart3;       // 初始化3种串口
extern volatile int oledupdate_state;
extern volatile int TurningUpdate_state;
extern volatile int TurningDowndate_state;
volatile float DownLocation; // x轴当前位置
volatile float UpLocation;   // y轴当前位置

extern float UpCurrentAngle;   // 记录当前角度，用于归零
extern float DownCurrentAngle; // 记录当前角度，用于归零
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// int GetDownLocation(void)
// {
//     int result;
//     if (DownMotorLocateDataGetFlag == 1)
//     {
//         DownMotorLocateDataGetFlag = 0;

//         if (DownMotorLocation_Array[0] == 0x00)
//         {
//             result = (DownMotorLocation * 360) / 65536;
//         }
//         else if (DownMotorLocation_Array[0] == 0x01)
//         {
//             result = -(DownMotorLocation * 360) / 65536;
//         }

//         OLED_ShowSignedNum(0, 32, result, 8,OLED_8X16);

//         return result;
//     }
//     else
//         return 0;
// }

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
    MX_TIM3_Init();
    /* USER CODE BEGIN 2 */
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_EnableIT_RXNE(USART2);
    LL_USART_EnableIT_RXNE(USART3);

    HAL_TIM_Base_Start_IT(&htim3); // 使能定时器中断

    // LL_USART_ClearFlag_RXNE(USART2);
    // LL_USART_EnableIT_RXNE(USART2);

    OLED_Init();

    uint8_t Header[] = {0xAA, 0x55}; // 包头
    float Data[1] = {3.14};          // 3FA3D70A 4048F5C3
    Serial_Registration(&Usart1, USART1);
    Serial_SetLogLevel(&Usart1, LogInfo);
    Serial_Registration(&Usart2, USART2);
    Serial_SetLogLevel(&Usart2, LogShutDown); // PA2->RX PA3->TX CW顺时针 CCW逆时针
    Serial_Registration(&Usart3, USART3);     // PB10 -> RX PB11 -> TX
    Serial_SetLogLevel(&Usart3, LogShutDown);
    static int Turnstate = 0 , avoid = 0;
    // Serial_PackTranAgrDecide(&Usart1, 2, Header);

    // Emm_V5_Pos_DownControl(&Usart2, 0x02, CW, 0, 0, false);
    HAL_Delay(10);
    // HAL_GPIO_WritePin(GPI*OC,GPIO_PIN_13 , GPIO_PIN_SET);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        // 发送字符串

        // Serial_SendPacket_float(&Usart1 , 1 , Data , 0xFF );

        // HAL_Delay(10);

        
        
        if(Turnstate == 0)
        {
            if(avoid == 1)
            {
                if (GetUpMotorState() == Ready && GetDownMotorState() == Ready)
                {
                    Turnstate = 1;
                    OLED_ShowString(0, 48, "[Info->State]:Second", OLED_6X8);
                }
            }
            TurnDownAngle(&Usart2, -20);
            TurnUpAngle(&Usart3, 20);
            avoid = 1;
        }
        else if(Turnstate == 1)
        {
            TurnDownAngle(&Usart2, -40);
            TurnUpAngle(&Usart3, 40);
        }
        
        
        
        // if(Turnstate == 0)
        // {
        //     if(GetUpMotorState() == Ready && GetDownMotorState() == Ready )
        //     {
        //         Turnstate = 1;
        //     }
        //     else
        //     {
        //         TurnDownAngle(&Usart2 , -20);
        //         TurnUpAngle(&Usart3, 20);
        //     }

        //     OLED_ShowString(0, 48, "[Info->Move1]:Done!", OLED_6X8);
        // }
        // else if(Turnstate == 1)
        // {
        //     if(GetUpMotorState() == Ready && GetDownMotorState() == Ready )
        //     {
        //         Turnstate = 2;
        //     }
        //     else
        //     {
        //         TurnDownAngle(&Usart2 , 20);
        //         TurnUpAngle(&Usart3, -20);
        //     }
        //     OLED_ShowString(0, 48, "[Info->Move2]:Done!", OLED_6X8);
        // }

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

        // OLED_ShowNum(0 , 40 , oledupdate_state , 1 , OLED_6X8);

        /*----------------------------中断控制部分----------------------------*/
        if (oledupdate_state == 1) // 屏幕刷新以及电机数据部分
        {
            oledupdate_state = 0;
            Emm_V5_GetCurrentLocation(&Usart2, DownAdr);

            Emm_V5_GetCurrentLocation(&Usart3, UpAdr);
            OLED_Update();
        }

        if (TurningDowndate_state == 1) // x轴电机控制
        {
            TurningDowndate_state = 0;
            if (DownMoveState == 0)
            {

                if (fabs(DownLocation) >= fabs(DownTagectAngle))
                {
                    DownTagectAngle = 0;
                    DownMoveState = 1;
                    Emm_V5_Pos_DownControl(&Usart2, DownAdr, CW, 0, 0, false);
                    OLED_ShowString(0, 32, "[Info->Down]:Done!", OLED_6X8);
                    SetDownCurrentAngle_0();//伪清除，如需要调零这个功能后期需要更改
                }
                else
                {
                    if (DownTagectAngle >= 0) // 顺时针
                    {
                        Emm_V5_Pos_DownControl(&Usart2, DownAdr, CW, 200, 0, false);
                    }
                    else if (DownTagectAngle < 0)
                    {
                        Emm_V5_Pos_DownControl(&Usart2, DownAdr, CCW, 200, 0, false);
                    }
                }
            }
        }
        if (TurningUpdate_state == 1) // y轴电机控制
        {
            TurningUpdate_state = 0;
            if (UpMoveState == 0)
            {
                if (fabs(UpLocation) >= fabs(UpTagectAngle))
                {
                    UpTagectAngle = 0;
                    UpMoveState = 1;
                    Emm_V5_Pos_UpControl(&Usart3, UpAdr, CW, 0, 0, false);
                    OLED_ShowString(0, 40, "[Info->Up]:Done!", OLED_6X8);
                    SetUpCurrentAngle_0();//伪清除，如需要调零这个功能后期需要更改
                }
                else
                {
                    if (UpTagectAngle >= 0) // 顺时针
                    {
                        Emm_V5_Pos_UpControl(&Usart3, UpAdr, CW, 150, 0, false);
                    }
                    else if (UpTagectAngle < 0)
                    {
                        Emm_V5_Pos_UpControl(&Usart3, UpAdr, CCW, 150, 0, false);
                    }
                }
            }
        }
        /*----------------------------中断控制部分----------------------------*/
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
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
    {
    }
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
    {
    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }
    LL_SetSystemCoreClock(72000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
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
