#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"
#include "stdbool.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
//仅标准库
#define PrioritySET_Default 1//配置默认
#define PrioritySET_Manual 2//手动配置

#define HEADERLENGTH 2//定义帧头的长度

/*与C++不同，直接使用宏定义*/
#define LogInfo 3 
#define LogWarn 2 
#define LogError 1 
#define LogShutDown 0 

#define header_length 8

// extern uint8_t Serial_IMURecieve[10];

// extern uint8_t SerFuncData[16];//无线串口接收的数据，数据格式见前页



struct UltraSerial
{
    //default configuration
	USART_TypeDef * USARTPort;//传入LL库窗口标志接口
    //data transmitte configuration
    uint8_t Serial_RxFlag;
	int m_LogLevel;
	char SerialSendBuffer[128];
    //data packect send param

    uint8_t headerlen;//帧头长度（最大8位）
    uint8_t header[header_length];//帧头

    uint8_t func;//数据功能位
    uint8_t Empty1;
    uint8_t datatype;//数据类型0x01->uint|0x02->int|0x03->float|0x04->double
    uint8_t datalength;//数据长度
    union {
        float fValue[8];
        double dValue[8];
        int32_t  i32Value[8];         // 32位整数
        uint8_t rawData[32];
    } data;
    uint8_t checksum;   // 校验和
};

typedef struct 
{
    uint8_t headerlen;//帧头长度（最大8位）
    uint8_t header[header_length];//帧头

    uint8_t func;//数据功能位
    uint8_t Empty1;
    uint8_t datatype;//数据类型0x01->uint|0x02->int|0x03->float|0x04->double
    uint8_t datalength;//数据长度
    union {
        float fValue;
        double dValue;
        int32_t  i32Value;         // 32位整数
        uint8_t rawData[32];
    } data;
    
    uint8_t checksum;   // 校验和
}UltraSerialRecieve;

typedef enum {
    STATE_HEADER,
    STATE_FUNC,
    STATE_EMPTY,
    STATE_DATA_TYPE,
    STATE_DATA_LENGTH,
    STATE_DATA,
    STATE_CHECKSUM,
    STATE_FREE
} ParserState_t;

typedef enum {
    UInt = 0x01,
    Int = 0x02,
    Float = 0x03,
    Double = 0x04
} Recieve_Type;

// #pragma pack(push, 1)
// typedef struct {
//     // uint8_t Headerlen;//帧头长度（最大8位）
//     uint8_t Header[header_length];//帧头
//     uint8_t func;//数据功能位
//     uint8_t Empty1;
//     uint8_t datatype;//数据类型
//     uint8_t databits;//数据位数
//     uint8_t datalength;//数据长度
//     union {
//         float fValue;
//         double dValue;
//         int32_t  i32Value;         // 32位整数
//         uint8_t rawData[32];
//     } data;
//     uint8_t checksum;   // 校验和
// } USART_Frame_t;
// #pragma pack(pop)

// extern uint8_t Serial_TxPacket[];
// extern uint8_t Serial_RxPacket[];

void Serial_Registration(struct UltraSerial * Serial , USART_TypeDef * Port_ );
void Serial_SendByte(struct UltraSerial * Serial , uint8_t Byte);
void Serial_SendArray(struct UltraSerial * Serial , uint8_t *Array, uint16_t Length);
void Serial_SendString(struct UltraSerial * Serial , char *String);
void Serial_SendNumber(struct UltraSerial * Serial , uint32_t Number, uint8_t Length);
void Serial_Printf(struct UltraSerial * Serial , char *format, ...);
void Serial_SendTranMoreToSingleByte(struct UltraSerial * Serial , uint32_t Num , int NumOfByte);


void Serial_PackTranAgrDecide(struct UltraSerial * Serial ,uint8_t HeaderLen_ ,  uint8_t * HeaderDate_ );//串口包头协议定义
uint8_t CalculateChecksum(uint8_t *data, uint8_t len);
void Serial_SendPacket_float(struct UltraSerial * Serial ,uint8_t DataBits_ , float * Data  , uint8_t func_);

uint8_t binary_array_to_hex(uint8_t *array, int size);//从数组转到16进制
void Serial_SetLogLevel(struct UltraSerial * Serial , int level);
void Serial_Info(struct UltraSerial * Serial , const char* message,...);//打印普通信息
void Serial_Warn(struct UltraSerial * Serial , const char* message , ...);//打印警告信息
void Serial_Error(struct UltraSerial * Serial , const char* message , ...);//打印错误信息
void Usart2_DataProcess(void);
//void Serial_SendPacket();


uint8_t Serial_GetRxFlag(struct UltraSerial * Serial);

#endif
