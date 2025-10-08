#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

#include <stdint.h>   
#include <stdbool.h>  
#include "Serial.h"  
#include <OLED.h>

#define CW 0x00
#define CCW 0x01



typedef struct 
{
    uint8_t Header;
    uint8_t Func        ;
    uint8_t Symbol;
    uint8_t Data[4];
    uint8_t CheckBit;
    
    
}MotorDataRecieve;

enum 


MotorDataRecieveSTATE{
    HEADER_STATE , 
    FUNC_STATE,
    SYMBOL_STATE,
    LOCATIONDATA_STATE,
    CHECKBIT_STATE
} ;



// 函数声明
void Emm_V5_Pos_DownControl(struct UltraSerial *Serial , uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

void Emm_V5_Pos_UpControl(struct UltraSerial *Serial , uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);


// 发送函数
void Emm_V5_Send1(struct UltraSerial *Serial , uint8_t *cmd, uint8_t len);

void Emm_V5_Send2(struct UltraSerial *Serial , uint8_t *cmd, uint8_t len);

void Emm_V5_GetDownCurrentLocation( struct UltraSerial *Serial , uint8_t addr);
float GetDownLocation(void);

#endif
