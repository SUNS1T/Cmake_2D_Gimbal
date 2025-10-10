#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f1xx_hal.h"
#include "OLED.h"
#include "Motor.h"
#include "tim.h"
#include "gpio.h"
extern struct UltraSerial Usart1, Usart2, Usart3;
extern volatile float DownTagectAngle;
extern volatile uint8_t DownMoveState;//跨文件提供三种状态
extern volatile float UpTagectAngle;
extern volatile uint8_t UpMoveState;//跨文件提供三种状态
extern volatile float DownLocation;//x轴当前位置
extern volatile float UpLocation;//y轴当前位置


#endif
