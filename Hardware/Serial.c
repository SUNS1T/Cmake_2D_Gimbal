#include "Serial.h"
float RxData_Float[header_length];
float RxData_Double[header_length];
float RxData_Int[header_length];
float RxData_Uint[header_length];
//[帧头]
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

// 串口包头协议定义
void Serial_PackTranAgrDecide(struct UltraSerial *Serial, uint8_t HeaderLen_, uint8_t *HeaderDate_)
{
	if (Serial == NULL || HeaderDate_ == NULL || HeaderLen_ == 0)
	{
		return;
	}

	// 复制帧头数据（关键修正！）
	memcpy(Serial->header, HeaderDate_, HeaderLen_);
	// *Serial->header = *HeaderDate_;
	Serial->headerlen = HeaderLen_;
}

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

// 在内存中的二进制表示：
// 25.67 = 0x41CD70A4 (十六进制)
// 内存布局: 0x41 0xCD 0x70 0xA4 (大端序)
// 或者:    0xA4 0x70 0xCD 0x41 (小端序，STM32是小端)
void Serial_SendPacket_float(struct UltraSerial *Serial, uint8_t DataBits_, float *Data, uint8_t func_)
{
	// // 计算实际需要的数据长度
	// uint8_t dataLength = DataBits_ * sizeof(float);
	// // uint8_t currentptr;//展示当前操作到第几位
	// USART_Frame_t frame;

	// for(int i = 0 ; i < Serial->headerlen ; i++)
	// {
	// 	frame.Header[i] = Serial->header[i];
	// }
	// // memcpy( frame.Header , Serial->header , Serial->headerlen);

	// frame.func = func_;
	// frame.Empty1 = 0x01;//空置位
	// frame.datatype = 0x03; //数据类型0x01->uint|0x02->int|0x03->float|0x04->double
	// frame.datalength = dataLength;//数据长度
	// memcpy(frame.data.rawData , Data , dataLength);

	// // 计算校验和（不包括帧头和校验和本身）
	// uint8_t *dataPtr = (uint8_t*)&frame + Serial->headerlen; // 跳过headerlen个帧头
	// frame.checksum = CalculateChecksum(dataPtr , sizeof(frame) - Serial->headerlen - 1);

	// Serial_SendArray(Serial , (uint8_t*)&frame , sizeof(frame));

	if (Serial == NULL || Data == NULL || DataBits_ == 0)
		return;
	uint8_t DataLength = DataBits_ * sizeof(float);
	if (DataLength > 32)
		return; // 防止溢出

			
	for (int i = 0; i < Serial->headerlen; i++)
	{
		Serial_SendByte(Serial, Serial->header[i]);
	}
	Serial_SendByte(Serial, func_);
	Serial_SendByte(Serial, 0xFF);
	Serial_SendByte(Serial, 0x03);
	Serial_SendByte(Serial, DataLength);

	memcpy(Serial->data.rawData , Data , DataBits_*4 );
	
	Serial_SendArray(Serial , Serial->data.rawData , DataBits_*4);
	//  if(Serial == NULL || Data == NULL || DataBits_ == 0) return;

	// uint8_t dataLength = DataBits_ * sizeof(float);
	// if(dataLength > 32) return; // 防止溢出

	// // 清零初始化
	// USART_Frame_t frame = {0};

	// // 设置帧头
	// memcpy(frame.Header, Serial->header, Serial->headerlen);

	// // 设置字段
	// frame.func = func_;
	// frame.Empty1 = 0x01;
	// frame.datatype = 0x03;
	// frame.databits = DataBits_;   // 重要！
	// frame.datalength = dataLength;

	// // 复制数据
	// memcpy(frame.data.rawData, Data, dataLength);

	// // 计算校验和（从func开始到数据结束）
	// uint8_t *checksumData = (uint8_t*)&frame + Serial->headerlen;
	// uint16_t checksumLen = 5 + dataLength; // 5个字段 + 数据

	// frame.checksum = CalculateChecksum(checksumData, checksumLen);

	// // 发送精确长度
	// uint16_t sendSize = Serial->headerlen + 5 + dataLength + 1;
	// Serial_SendArray(Serial, (uint8_t*)&frame, sendSize);
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

/*---------------串口中断接收部分---------------*/
void GetFloatData()
{

}

void USART1_IRQHandler()
{
	// 检查是否是接收数据寄存器非空（RXNE）中断
	if (LL_USART_IsActiveFlag_RXNE(USART1))
	{
		uint8_t receivedByte = LL_USART_ReceiveData8(USART1); // 获取接收到的字节

		// 如果需要接收浮点数，确保接收的字节数足够，或按照协议格式处理数据
		static ParserState_t state = STATE_HEADER;
		// static Recieve_Type Rtype;
		static UltraSerialRecieve frame;
		static uint8_t dataIndex = 0;
		static uint8_t expectedLength = 0;
		static uint8_t HeaderRecieveRecord = 0;
		static uint8_t HeaderRecord[HEADERLENGTH];
		uint8_t Header[2] = {0xAA , 0x55};
		switch (state)
		{
			case STATE_HEADER:
				// if (receivedByte == 0xAA)
				// {
			
				// 	state = STATE_HEADER2;
				// }
				if (HeaderRecieveRecord <= HEADERLENGTH)
				{
					HeaderRecord[HeaderRecieveRecord] = receivedByte;
					HeaderRecieveRecord++;
				}
				else
				{
					for (int i = 0; i < HEADERLENGTH; i++)
					{
						if (HeaderRecord[i] != Header[i])
							return;
					}
					state = STATE_DATA_TYPE;
				}
				break;
			case STATE_DATA_TYPE:
				frame.datatype = receivedByte;
				state = STATE_DATA_LENGTH;
				break;
			
			case STATE_DATA_LENGTH:
				frame.datalength = receivedByte;
				
				expectedLength = receivedByte;
				dataIndex = 0;
				state = STATE_DATA;
				break;
			
			case STATE_DATA:
				frame.data.rawData[dataIndex++] = receivedByte;
				if (dataIndex >= expectedLength)
				{
					switch (frame.datatype)
					{
					case 0x01:
						/* code */
						memcpy(  RxData_Uint , frame.data.rawData , sizeof(frame.data.rawData) );
						break;
					case 0x02:
						/* code */
						memcpy(  RxData_Int  , frame.data.rawData , sizeof(frame.data.rawData) );
						break;
					case 0x03:
						/* code */
						memcpy( RxData_Float , frame.data.rawData , sizeof(frame.data.rawData) );
						break;
					case 0x04:
						/* code */
						memcpy(RxData_Double , frame.data.rawData , sizeof(frame.data.rawData) );
						break;
					default:
						break;
					}
					state = STATE_HEADER;
				}	
				break;
			
				// case STATE_CHECKSUM:
				// 	frame.checksum = receivedByte;
			
				// 	// 验证校验和
				// 	uint8_t calcChecksum = CalculateChecksum(
				// 		(uint8_t *)&frame + 2, sizeof(frame) - 3);
			
				// 	if (calcChecksum == frame.checksum && frame.dataType == 0x01)
				// 	{
				// 		state = STATE_HEADER1;
				// 		return frame.data.fValue;
				// 	}
				// 	else
				// 	{
				// 		state = STATE_HEADER1;
				// 		return 0.0f; // 错误处理
				// 	}
				// 	break;
		}
	}
}