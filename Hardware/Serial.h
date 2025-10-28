#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"
#include "stdbool.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "OLED.h"

/*与C++不同，直接使用宏定义*/
#define LogInfo 3 
#define LogWarn 2 
#define LogError 1 
#define LogShutDown 0 

//-------------RECIEVE CONFIGURE LIST-------------
#define HeaderMAXLength 8 //接收包头最大长度(默认为8)
#define HEADERLENGTH 2//实际接受的报头长度
//包头请在seiral.c中手动配置
//-------------RECIEVE CONFIGURE LIST-------------
//---------------SEND CONFIGURE LIST--------------
#define HAL_LL_USART
// #define STANDARD_LIBRARY_USART
//仅标准库
#define PrioritySET_Default 1//配置默认
#define PrioritySET_Manual 2//手动配置
//---------------SEND CONFIGURE LIST--------------


// extern uint8_t Serial_IMURecieve[10];

// extern uint8_t SerFuncData[16];//无线串口接收的数据，数据格式见前页
#ifdef HAL_LL_USART
struct UltraSerial
{
    //default configuration
	USART_TypeDef * USARTPort;//传入LL库串口标志接口
    //data transmitte configuration
    uint8_t Serial_RxFlag;
	int m_LogLevel;
	char SerialSendBuffer[128];
    //data packect send param
    uint8_t headerlen;                   // 帧头长度（最大8位）
    uint8_t header[HeaderMAXLength + 1]; // 帧头+功能位接收
    union {
        float fValue[8];
        double dValue[4];//double因为占用8字节而最大字节数是32，所以用了
        int32_t  i32Value[8];         // 32位整数
        uint8_t rawData[32];
    } data;
    uint8_t checksum;   // 校验和
};
#endif
/*-----接收数据处理-----*/
/* 串口设备数据接收结构体 */
typedef struct
{
    uint8_t *pTxBuf; /* 发送缓冲区 */
    uint8_t *pRxBuf; /* 接收缓冲区 */
    uint16_t LEN;    /* 接收到的数据长度 */
    uint8_t FLG;     /* 接收标志位 */
} UART_DAT;


/*接收过程：
    ---数据存入层---
    DMA接收
    ---数据转存层---
    读取标志位
    存储入结构体    可以随意调取数据
    ---数据处理层---
    对比是否符合当前在初始化函数中所定义的结构规范
    | 是 ->留下
    | 否 ->删除 准备接受下一帧
    ->|返回|->/数据存入层/
*/
enum DataFrameInLaw
{
    LEGAL,
    INLEGAL
};

struct UltraSerialDataDeal
{
    uint8_t headerlen;                   // 帧头长度（最大8位）
    uint8_t header[HeaderMAXLength + 1]; // 帧头+功能位接收

    uint8_t func;       // 数据功能位->嚴禁設置爲0xFF
    uint8_t Empty1;     // 0xFF
    uint8_t datatype;   // 数据类型0x01->uint|0x02->int|0x03->float|0x04->double
    uint8_t datalength; // 数据长度
    union
    {
        float fValue[8];
        double dValue[4];
        int32_t UValue[8];  // 32位整数
        uint32_t IValue[8]; // 32位整数
        uint8_t rawData[32];
    } data;
    uint8_t checksum; // 校验和
    enum DataFrameInLaw dataframeinlaw;
};

#ifdef STANDARD_LIBRARY_USART
struct UltraSerial
{
	uint8_t BautRate;
	uint16_t Rx_Port;
	uint16_t Tx_Port;
	uint16_t USART_StopBits;
	uint16_t USART_Parity;
	uint16_t USART_WordLength;
	GPIO_TypeDef*  RCC_Periph;
	USART_TypeDef * USARTPort;

	int m_LogLevel;
	char SerialSendBuffer[128];
};
#endif

void MY_UART_IRQHandler(USART_TypeDef *USARTx);
void UartVarInit(void);
void InitHardUart(void);
void Serial_DataDeal(void); // 将数据转存到结构体中存储
void Serial_RecieveInit(struct UltraSerialDataDeal *Serial, uint8_t HeaderLength);
/*-----接收数据处理-----*/

extern struct UltraSerial Usart1, Usart2;

void Serial_Registration(struct UltraSerial * Serial , USART_TypeDef * Port_ );
void Serial_SendByte(struct UltraSerial * Serial , uint8_t Byte);
void Serial_SendArray(struct UltraSerial * Serial , uint8_t *Array, uint16_t Length);
void Serial_SendString(struct UltraSerial * Serial , char *String);
void Serial_SendNumber(struct UltraSerial * Serial , uint32_t Number, uint8_t Length);
void Serial_Printf(struct UltraSerial * Serial , char *format, ...);
void Serial_SendTranMoreToSingleByte(struct UltraSerial * Serial , uint32_t Num , int NumOfByte);


void Serial_PackTranAgrDecide(struct UltraSerial * Serial ,uint8_t HeaderLen_ ,  uint8_t * HeaderDate_ );//串口包头协议定义
uint8_t Serial_SendPacket_float(struct UltraSerial *Serial, uint8_t DataBits_, float *Data, uint8_t func_);
uint8_t Serial_SendPacket_double(struct UltraSerial *Serial, uint8_t DataBits_, double *Data, uint8_t func_);
uint8_t Serial_SendPacket_int(struct UltraSerial *Serial, uint8_t DataBits_, int *Data, uint8_t func_);
uint8_t Serial_SendPacket_uint(struct UltraSerial *Serial, uint8_t DataBits_, uint32_t *Data, uint8_t func_);
// 将单个数字转换为多个16进制数发送

uint8_t CalculateChecksum(uint8_t *data, uint8_t len);

uint8_t binary_array_to_hex(uint8_t *array, int size);//从数组转到16进制
void Serial_SetLogLevel(struct UltraSerial * Serial , int level);
void Serial_Info(struct UltraSerial * Serial , const char* message,...);//打印普通信息
void Serial_Warn(struct UltraSerial * Serial , const char* message , ...);//打印警告信息
void Serial_Error(struct UltraSerial * Serial , const char* message , ...);//打印错误信息
void Usart2_DataProcess(void);
//void Serial_SendPacket();


uint8_t Serial_GetRxFlag(struct UltraSerial * Serial);

#endif
