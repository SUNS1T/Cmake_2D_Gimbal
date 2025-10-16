/**
 ******************************************************************************
 * @file           : Serial.c
 * @brief          : 串口配置
 ******************************************************************************
 * @attention
 *
 * Created By SUNS1T
 *
 * 功能:
 * 1、基础LL库串口发送，串口端口注册
 * 2、调试信息发送，在你发送的信息前面加上[Info]或[Warn]或[Error]，和在你发的信息后面添加\r\n,可以自定义发送等级，用函数Serial_SetLogLevel
 * 	  三个可调节单位#define LogInfo 3 
 *				   #define LogWarn 2 
 *				   #define LogError 1 
 *				   #define LogShutDown 0 
 * 3、可以实现自动发包只需要初始化包头，包头长度，在发送时添加数据个数、用于发送数据的数组还有功能位就行
 * 4、可以实现只添加包头就可以自动解析由上一项函数发送的数据包，数据统一存在一个数组里面，方便调取，不需要再写拆包程序(拆包与解包自带校验位)
 * 5、处理好的数据会直接放入float fRxData[8]; double dRxData[4]; int32_t iRxData[8];中
 * 
 * 迭代版本：
 * v1.0 
 * v2.0
 * v3.0
 * 当前版本：
 * v4.0 2025-10-13
 * v4.1.0Beta 2025-10-14 修复了一些bug
 * 
 * 配置示例：
 * 详见实例Cmake工程和main.c
 * 
 * 使用库：
 * stm32 LL库
 * stm32 标准库
 * 
 * 计划更新：
 * stm32标准库DMA版本
 * 支持配置大端序和小端序解码(目前不支持)
 ******************************************************************************
 */
#include "Serial.h"
//-------------RECIEVE CONFIGURE LIST-------------
uint8_t SetRxHeader[HeaderMAXLength] = {0xAA , 0x55};
//-------------RECIEVE CONFIGURE LIST-------------

/**
 * 函    数：电机设置地址
 * 参    数：  struct Emm42Motor *Motor 创建的电机实例对象
 *             NOWAddress 电机地址
 *             MultiMach 多机控制
 *             AccelerateMode 是否启用加速度模式
 * 返 回 值：无
 * 说    明：无
 */
void Serial_Registration(struct UltraSerial *Serial, USART_TypeDef *Port_)
{
	Serial->USARTPort = Port_;
}
//
void Serial_SendByte(struct UltraSerial *Serial, uint8_t Byte)
{
	// 等待发送缓冲区为空
	while (!LL_USART_IsActiveFlag_TXE(Serial->USARTPort))
		;
	// 发送数据
	LL_USART_TransmitData8(Serial->USARTPort, Byte);
}

void Serial_SendArray(struct UltraSerial *Serial, uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i++)
	{
		Serial_SendByte(Serial, Array[i]);
	}
}

void Serial_SendString(struct UltraSerial *Serial, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		Serial_SendByte(Serial, String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(struct UltraSerial *Serial, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)
	{
		Serial_SendByte(Serial, Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

// 将单个数字转换为多个16进制数发送
void Serial_SendTranMoreToSingleByte(struct UltraSerial *Serial, uint32_t Num, int NumOfByte)
{
	int bytes;
	for (int i = NumOfByte - 1; i >= 1; i--)
	{
		bytes = (Num >> (8 * i)) & 0xFF;
		Serial_SendByte(Serial, bytes);
	}
	bytes = Num & 0xFF;
	Serial_SendByte(Serial, bytes);
}

// 检测和
uint8_t CalculateChecksum(uint8_t *data, uint8_t len)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < len; i++)
	{
		sum += data[i];
	}
	return sum;
}
/*--------------串口发送及调试部分--------------*/
void Serial_Printf(struct UltraSerial *Serial, char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(Serial, String);
}

uint8_t Serial_GetRxFlag(struct UltraSerial *Serial)
{
	if (Serial->Serial_RxFlag == 1)
	{
		Serial->Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

void Serial_SetLogLevel(struct UltraSerial *Serial, int level)
{
	Serial->m_LogLevel = level;
}

void Serial_Info(struct UltraSerial *Serial, const char *message, ...) // 打印普通信息
{
	if (Serial->m_LogLevel == 3)
	{
		char String[100];
		va_list arg;
		va_start(arg, message);
		vsprintf(String, message, arg);
		va_end(arg);
		sprintf(Serial->SerialSendBuffer, "[Info]:%s\r\n", String); // 字符串拷贝
		Serial_Printf(Serial, Serial->SerialSendBuffer);
	}
}

void Serial_Warn(struct UltraSerial *Serial, const char *message, ...) // 打印警告信息
{
	if (Serial->m_LogLevel == 2)
	{
		char String[100];
		va_list arg;
		va_start(arg, message);
		vsprintf(String, message, arg);
		va_end(arg);
		sprintf(Serial->SerialSendBuffer, "[WARN]:%s\r\n", message); // 字符串拷贝
		Serial_Printf(Serial, Serial->SerialSendBuffer);
	}
}

void Serial_Error(struct UltraSerial *Serial, const char *message, ...) // 打印错误信息
{
	if (Serial->m_LogLevel == 1)
	{
		char String[100];
		va_list arg;
		va_start(arg, message);
		vsprintf(String, message, arg);
		va_end(arg);

		sprintf(Serial->SerialSendBuffer, "[ERROR]:%s\r\n", message); // 字符串拷贝
		Serial_Printf(Serial, Serial->SerialSendBuffer);
	}
}
/*--------------串口发送及调试部分--------------*/
/*--------------串口数据包发送部分--------------*/
// 串口包头协议定义
void Serial_PackTranAgrDecide(struct UltraSerial *Serial, uint8_t HeaderLen_, uint8_t *HeaderDate_)
{
	if (Serial == NULL || HeaderDate_ == NULL || HeaderLen_ == 0)
		return;
	// 复制帧头数据（关键修正！）
	memcpy(Serial->header, HeaderDate_, HeaderLen_);
	Serial->headerlen = HeaderLen_;
}
 // 数据类型0x01->uint|0x02->int|0x03->float|0x04->double
uint8_t Serial_SendPacket_float(struct UltraSerial *Serial, uint8_t DataBits_, float *Data, uint8_t func_)
{
	if (Serial == NULL || Data == NULL || DataBits_ == 0)
		return 0;
	uint8_t DataLength = DataBits_ * sizeof(float);
	uint8_t CrcNumLength = 0, Temp[2] = {0xFF, 0x03};
	CrcNumLength = Serial->headerlen + 4 + DataLength;
	uint8_t CheckSum[CrcNumLength];
	if (DataLength > 32)
		return 0; // 防止溢出

	// for (int i = 0; i < Serial->headerlen; i++)
	// {
	// 	Serial_SendByte(Serial, Serial->header[i]);
	// }
	// Serial_SendByte(Serial, func_);
	// Serial_SendByte(Serial, 0xFF);
	// Serial_SendByte(Serial, 0x03);
	// Serial_SendByte(Serial, DataLength);

	memcpy(Serial->data.rawData, Data, DataBits_ * 4); // 发送数据

	// Serial_SendArray(Serial, Serial->data.rawData, DataBits_ * 4);
	memcpy(CheckSum, Serial->header, sizeof(Serial->header));// 发送帧头	
	memcpy(CheckSum + Serial->headerlen, &func_, 1);
	memcpy(CheckSum + Serial->headerlen + 1, &Temp, 2);
	memcpy(CheckSum + Serial->headerlen + 3, &DataLength, 1);
	memcpy(CheckSum + Serial->headerlen + 4, Serial->data.rawData, DataLength);

	Serial->checksum = CalculateChecksum(CheckSum, CrcNumLength);

	Serial_SendArray(&Usart1 , CheckSum , CrcNumLength);//发送数据
	Serial_SendByte(Serial, Serial->checksum);
	// Serial_SendByte(Serial, CrcNumLength);
	return 1;//发送成功
}

//AA 55 FF FF 04 10 70 41 B6 2C 5F AF 28 40 52 B8 1E 85 1B 91 D2 40 85 包头AA 55 功能位FF 数据12.342523、19012.43
uint8_t Serial_SendPacket_double(struct UltraSerial *Serial, uint8_t DataBits_, double *Data, uint8_t func_)
{

	if (Serial == NULL || Data == NULL || DataBits_ == 0)
		return 0;
	uint8_t DataLength = DataBits_ * sizeof(double);
	uint8_t CrcNumLength = 0, Temp[2] = {0xFF, 0x04} , CheckValue = 0;
	CrcNumLength = Serial->headerlen + 4 + DataLength;
	uint8_t CheckSum[CrcNumLength];
	if (DataLength > 32)
		return 0; // 防止溢出

	//内存转存
	memcpy(Serial->data.rawData, Data, DataBits_ * 8); 
	memcpy(CheckSum, Serial->header, sizeof(Serial->header));// 发送帧头	
	memcpy(CheckSum + Serial->headerlen, &func_, 1);
	memcpy(CheckSum + Serial->headerlen + 1, &Temp, 2);
	memcpy(CheckSum + Serial->headerlen + 3, &DataLength, 1);
	memcpy(CheckSum + Serial->headerlen + 4, Serial->data.rawData, DataLength);

	CheckValue = CalculateChecksum(CheckSum, sizeof(CheckSum));
	Serial_SendArray(&Usart1 , CheckSum , CrcNumLength);//发送数据
	
	Serial_SendByte(Serial, CheckValue);
	return 1;//发送成功
}

//int的协议：数据位一共有4个字节
//AA 55 FF FF 02 08 FB 54 BC 00 BB 02 1D 00 EC 测试数据帧头：AA 55 功能位FF 数据1：12342523 数据二:1901243
uint8_t Serial_SendPacket_int(struct UltraSerial *Serial, uint8_t DataBits_, int *Data, uint8_t func_)
{
	if (Serial == NULL || Data == NULL || DataBits_ == 0)
		return 0;
	uint8_t DataLength = DataBits_ * sizeof(int);
	uint8_t CrcNumLength = 0, Temp[2] = {0xFF, 0x02};
	CrcNumLength = Serial->headerlen + 4 + (DataBits_ * 4);
	uint8_t CheckSum[CrcNumLength];
	if (DataLength > 32)
		return 0; // 防止溢出

	//内存转存
	memcpy(Serial->data.rawData, Data, DataBits_ * 4); 
	memcpy(CheckSum, Serial->header, sizeof(Serial->header));// 发送帧头	
	memcpy(CheckSum + Serial->headerlen, &func_, 1);
	memcpy(CheckSum + Serial->headerlen + 1, &Temp, 2);
	memcpy(CheckSum + Serial->headerlen + 3, &DataLength, 1);
	memcpy(CheckSum + Serial->headerlen + 4, Serial->data.rawData, DataLength);

	Serial->checksum = CalculateChecksum(CheckSum, CrcNumLength - 1);
	// Serial_SendByte(Serial, Serial->checksum);
	Serial_SendArray(Serial , CheckSum , CrcNumLength);//发送数据
	Serial_SendByte(Serial, Serial->checksum);
	return 1;//发送成功
}
	
/*--------------串口数据包发送部分--------------*/
/*--------------串口接收部分--------------*/
/* 定义串口缓冲区大小，分为发送缓冲区和接收缓冲区 */
#define UART1_TX_BUF_SIZE 1
#define UART1_RX_BUF_SIZE 1 * 512

/* 定义每个串口结构体变量 */
UART_DAT dat_Uart1 = {0};
static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE + 1]; /* 发送缓冲区 */
static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE + 1]; /* 接收缓冲区 */
struct UltraSerialDataDeal frame = {0};//用于数据处理，严禁私调
/*定义外部文件可调取变量*/
float fRxData[8];
double dRxData[4];
int32_t iRxData[8];

void UartVarInit(void)
{
    /* 清空结构体 */
    memset(&dat_Uart1, 0, sizeof(UART_DAT));

    /* 串口1 用到的缓存 */
    dat_Uart1.pRxBuf = g_RxBuf1;
    dat_Uart1.pTxBuf = g_TxBuf1;
}


void Serial_RecieveInit(struct UltraSerialDataDeal *Serial, uint8_t HeaderLength)
{
    Serial->headerlen = HeaderLength;
}

void Serial_DataDeal(void) // 将数据转存到结构体中存储
{
    if (dat_Uart1.FLG == 1)
    {
        Serial_RecieveInit(&frame, HEADERLENGTH);
        uint8_t datatypelength = 0, CheckArray[dat_Uart1.LEN]; // 當前接受訊息位置 , datatypelength数据接收字节数
        // 接收开始处理

        __disable_irq(); // 关闭所有中断

        for (int i = 0; i < frame.headerlen; i++)
        {
            frame.header[i] = dat_Uart1.pRxBuf[i];
        }

        frame.func = dat_Uart1.pRxBuf[HEADERLENGTH]; // 接收功能位數據

        if (dat_Uart1.pRxBuf[HEADERLENGTH + 1] != 0xFF)
        {
            frame.dataframeinlaw = INLEGAL;
        }
        else
        {
            frame.Empty1 = 0xFF;
        }

        frame.datatype = dat_Uart1.pRxBuf[HEADERLENGTH + 2]; // 数据类型

        frame.datalength = dat_Uart1.pRxBuf[HEADERLENGTH + 3]; // 数据个数

        // 数据类型0x01->uint|0x02->int|0x03->float|0x04->double
        if (frame.datatype == 0x01 || frame.datatype == 0x03|| frame.datatype == 0x02)
        {
            datatypelength = frame.datalength * 1;
        }
        else if (frame.datatype == 0x04)
        {
            datatypelength = frame.datalength * 2;
        }

        for (int i = 0; i < datatypelength; i++)
        {
            frame.data.rawData[i] = dat_Uart1.pRxBuf[HEADERLENGTH + 4 + i];
        }

		uint8_t num = dat_Uart1.LEN - 1;
        frame.checksum = dat_Uart1.pRxBuf[num]; // 获取校验位

        memcpy(CheckArray, dat_Uart1.pRxBuf, dat_Uart1.LEN); // 将要检查的数组传递给checkarray

        uint8_t SUMCheck = CalculateChecksum(CheckArray, dat_Uart1.LEN - 1);
		
		uint8_t HeaderCheck = memcmp(frame.header , SetRxHeader , HEADERLENGTH);//包头比对

        if (SUMCheck == frame.checksum && HeaderCheck == 0)
        {
            frame.dataframeinlaw = LEGAL;
			
            switch (frame.datatype)
            {
            case 0x02:
                memcpy(iRxData, frame.data.IValue, sizeof(frame.data.rawData));
				// Serial_Printf(&Usart1 , "Data1:%d,Data2:%d\r\n",iRxData[0],iRxData[1]);//调试代码
                break;
            case 0x03:
                memcpy(fRxData, frame.data.fValue, sizeof(frame.data.rawData));
				// Serial_Printf(&Usart1 , "Data1:%f,Data2:%f\r\n",frame.data.fValue[0] , frame.data.fValue[1]);//调试代码
                break;
            case 0x04:
                memcpy(dRxData, frame.data.dValue, sizeof(frame.data.rawData));
				// Serial_Printf(&Usart1 , "Data1:%lf,Data2:%lf\r\n",frame.data.dValue[0],frame.data.dValue[1]);//调试代码
                break;
            default:
                break;
            }
        }
        else
        {
            frame.dataframeinlaw = INLEGAL;
        }
        /*----------调试代码----------*/
        // Serial_SendArray(&Usart1 , frame.header , 2);
        // Serial_SendByte(&Usart1 , frame.func);
        // Serial_SendByte(&Usart1 , frame.Empty1);
        // Serial_SendByte(&Usart1 , frame.datatype);
        // Serial_SendByte(&Usart1 , frame.datalength);

        // Serial_SendArray(&Usart1 , frame.data.rawData , datatypelength);

        // Serial_SendByte(&Usart1 , frame.checksum);

        // Serial_SendArray(&Usart1 , (uint8_t *)frame.data.fValue , sizeof(frame.data.fValue));
        /*----------调试代码----------*/

        // Serial_Printf(&Usart1, "ActuallyCheck:%d , frame.checksum:%d , dat_Uart1.LEN%d\r\n", Check , dat_Uart1.pRxBuf[10] , dat_Uart1.LEN);
		// Serial_Printf(&Usart1 , "HeaderCheck:%d\r\n" , HeaderCheck );
        dat_Uart1.FLG = 0;
        __enable_irq(); // 使能所有中断
        // 接收完成处理
    }
}

/*
*******************************************************************************************
*	函 数 名: InitHardUart
*	功能说明: 配置串口的硬件参数、启用串口中断等功能
*	形    参: 无
*	返 回 值: 无
*******************************************************************************************
*/
void InitHardUart(void)
{
    /* 设置DMA接收缓存和缓存大小 */
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, LL_USART_DMA_GetRegAddr(USART1));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)dat_Uart1.pRxBuf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, UART1_RX_BUF_SIZE);

    // 启用串口空闲中断
    LL_USART_EnableIT_IDLE(USART1);

    /* 开启DMA传输完成 传输错误中断 */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);

    /* 启动串口接收DMA通道  并启用串口接收DMA */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    LL_USART_EnableDMAReq_RX(USART1);
}
/*
*****************************************************************************************
*	函 数 名: MY_UART_IRQHandler
*	功能说明: 串口中断处理函数
*			  可同时处理多个串口的中断事件
*	形    参：USARTx: 串口设备
*	返 回 值: 无
*****************************************************************************************
*/
void MY_UART_IRQHandler(USART_TypeDef *USARTx) // 记得去定义一下
{
    /* 判断空闲中断标志位 */
    if (LL_USART_IsActiveFlag_IDLE(USARTx) && LL_USART_IsEnabledIT_IDLE(USARTx))
    {
        // 清除空闲中断标志位
        LL_USART_ClearFlag_IDLE(USARTx);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

        /* LED灯翻转指示数据收发 */
        // LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);

        dat_Uart1.LEN = UART1_RX_BUF_SIZE - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);

        /* 打开接收完成标志位 并将数据传入接收函数中处理 */
        dat_Uart1.FLG = 1;
        dat_Uart1.pRxBuf[dat_Uart1.LEN] = 0;

        /* 重新设置数据长度并打开DMA，使DMA从头开始接收 */
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, UART1_RX_BUF_SIZE);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
    }
    else
    {
        /* 错误回调函数 */
        // UART_Error_Callback();
    }
}
/*--------------串口接收部分--------------*/
#ifdef STANDARD_LIBRARY_USART
/**
  * 函    数：串口默认配置
  * 参    数：struct UltraSerial * Serial 传入初始化的结构体 
  * 		  PortChoose 选择端口，能够为GPIOx (x为A , B , C中任意一个值)
  * 		  BuatRate 串口波特率
  *           RxSerial_Port Rx端口 GPIO_Pin_x (x为你使用的端口的数字部分的值，比如PA9就填GPIO_Pin_9)
  * 		  TxSerial_Port Tx端口 GPIO_Pin_x (x为你使用的端口的数字部分的值，比如PA10就填GPIO_Pin_10)
  * 		  PrioritySet 优先级设置 PrioritySET_Default->自动配置 PrioritySET_Manual->手动配置	 手动配置请自行创建函数配置，模板见自动配置
  * 返 回 值：无
  * 说    明：无
  */
void SerialInit_Default(struct UltraSerial * Serial  , GPIO_TypeDef *  PortChoose , int BuatRate , uint16_t RxSerial_Port , uint16_t TxSerial_Port , uint8_t PrioritySet)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/*----------------UltraSerialInit----------------*/
	Serial->RCC_Periph = PortChoose;
	Serial->Rx_Port = RxSerial_Port;
	Serial->Tx_Port = TxSerial_Port;
	Serial->USART_Parity = USART_Parity_No;
	Serial->USART_StopBits = USART_StopBits_1;
	Serial->USART_WordLength = USART_WordLength_8b ;
	Serial->BautRate = BuatRate;
	
	USART_TypeDef * SerialPort;
	uint8_t SerialIQHandler;
	/*----------------UltraSerialInit----------------*/
	if(RxSerial_Port == GPIO_Pin_10 && TxSerial_Port == GPIO_Pin_9)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		SerialPort = USART1;
		SerialIQHandler = USART1_IRQn;
		Serial->USARTPort = USART1;
	}
	else if(RxSerial_Port == GPIO_Pin_3 && TxSerial_Port == GPIO_Pin_2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		SerialPort = USART2;
		SerialIQHandler = USART2_IRQn;
		Serial->USARTPort = USART2;
	}
	else if(RxSerial_Port == GPIO_Pin_11 && TxSerial_Port == GPIO_Pin_10)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		SerialPort = USART3;
		SerialIQHandler = USART3_IRQn;
		Serial->USARTPort = USART3;
	}	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = Serial->Tx_Port;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Serial->RCC_Periph, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = Serial->Rx_Port;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Serial->RCC_Periph, &GPIO_InitStructure);
	
	
	USART_InitStructure.USART_BaudRate = BuatRate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b ;
	USART_Init(SerialPort, &USART_InitStructure);
	
	USART_ITConfig(SerialPort, USART_IT_RXNE, ENABLE);

	if(PrioritySet == PrioritySET_Default)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = SerialIQHandler;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_Init(&NVIC_InitStructure);
	}
	else if(PrioritySet == PrioritySET_Manual)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = SerialIQHandler;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_Init(&NVIC_InitStructure);
	}
	
	
	USART_Cmd(SerialPort, ENABLE);

}


//串口中断手动配置
// void SerialInit_PrioritySET(void)
// {
// 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
// 	NVIC_InitTypeDef NVIC_InitStructure;
// 	NVIC_InitStructure.NVIC_IRQChannel = SerialIQHandler;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
// 	NVIC_Init(&NVIC_InitStructure);
// }


/**
  * 函    数：串口手动详细配置
  * 参    数：struct UltraSerial * Serial 传入初始化的结构体 
  * 		  PortChoose 选择端口，能够为GPIOx (x为A , B , C中任意一个值)
  * 		  BuatRate 串口波特率
  *           RxSerial_Port Rx端口 GPIO_Pin_x (x为你使用的端口的数字部分的值，比如PA9就填GPIO_Pin_9)
  * 		  TxSerial_Port Tx端口 GPIO_Pin_x (x为你使用的端口的数字部分的值，比如PA10就填GPIO_Pin_10)
  * 		  PrioritySet 优先级设置 PrioritySET_Default->自动配置 PrioritySET_Manual->手动配置	 手动配置请自行创建函数配置，模板见自动配置
  * 		  USART_StopBits停止位 值为 USART_StopBits_1 或 USART_StopBits_0_5 或 USART_StopBits_2 或 USART_StopBits_1_5
  * 		  USART_Parity 中断优先级 值为 USART_Parity_No 或 USART_Parity_Even 或 USART_Parity_Odd 
  * 		  USART_WordLength 数据为 值为 USART_WordLength_8b 或 USART_WordLength_9b
  * 返 回 值：无
  * 说    明：无
  */
void SerialInit_Manual(struct UltraSerial * Serial , GPIO_TypeDef *  PortChoose , int BuatRate , uint16_t RxSerial_Port , uint16_t TxSerial_Port , uint16_t USART_StopBits , 
	uint16_t USART_Parity , uint16_t USART_WordLength , uint8_t PrioritySet)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	/*----------------UltraSerialInit----------------*/
	Serial->RCC_Periph = PortChoose;
	Serial->Rx_Port = RxSerial_Port;
	Serial->Tx_Port = TxSerial_Port;
	Serial->USART_Parity = USART_Parity;
	Serial->USART_StopBits = USART_StopBits;
	Serial->USART_WordLength = USART_WordLength ;
	Serial->BautRate = BuatRate;

	USART_TypeDef * SerialPort;
	uint8_t SerialIQHandler;
	/*----------------UltraSerialInit----------------*/
	
	if(RxSerial_Port == GPIO_Pin_10 && TxSerial_Port == GPIO_Pin_9)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		SerialPort = USART1;
		SerialIQHandler = USART1_IRQn;
		Serial->USARTPort = USART1;
	}
	else if(RxSerial_Port == GPIO_Pin_3 && TxSerial_Port == GPIO_Pin_2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		SerialPort = USART2;
		SerialIQHandler = USART2_IRQn;
		Serial->USARTPort = USART2;
	}
	else if(RxSerial_Port == GPIO_Pin_11 && TxSerial_Port == GPIO_Pin_10)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		SerialPort = USART3;
		SerialIQHandler = USART3_IRQn;
		Serial->USARTPort = USART3;
	}	
	
	
	
	

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = Serial->Tx_Port;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Serial->RCC_Periph, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = Serial->Rx_Port;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Serial->RCC_Periph, &GPIO_InitStructure);
	
	
	USART_InitStructure.USART_BaudRate = BuatRate;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = Serial->USART_Parity;
	USART_InitStructure.USART_StopBits = Serial->USART_StopBits;
	USART_InitStructure.USART_WordLength = Serial->USART_WordLength ;
	USART_Init(SerialPort, &USART_InitStructure);
	
	USART_ITConfig(SerialPort, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SerialIQHandler;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(SerialPort, ENABLE);

	if(PrioritySet == PrioritySET_Default)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = SerialIQHandler;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_Init(&NVIC_InitStructure);
	}
	if(PrioritySet == PrioritySET_Manual)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = SerialIQHandler;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_Init(&NVIC_InitStructure);
	}
	
}

#endif