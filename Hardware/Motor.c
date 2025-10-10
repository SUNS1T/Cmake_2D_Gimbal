#include <Motor.h>
volatile uint8_t DownMotorLocation_Array[6];
volatile int32_t DownMotorLocation;
volatile int8_t DownMotorLocateDataGetFlag;//下面电机接收数据标志位
volatile uint8_t UpMotorLocation_Array[6];
volatile int32_t UpMotorLocation;
volatile int8_t UpMotorLocateDataGetFlag;//上面电机接收数据标志位
volatile float DownTagectAngle;
volatile uint8_t DownMoveState = 1;//跨文件提供三种状态
volatile float UpTagectAngle;
volatile uint8_t UpMoveState = 1;//跨文件提供三种状态
// uint8_t 

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_UpControl(struct UltraSerial *Serial , uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[8] = {0};

    // 装载命令
    cmd[0] = addr;                // 地址
    cmd[1] = 0xF6;                // 功能码
    cmd[2] = dir;                 // 方向
    cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
    cmd[5] = acc;                 // 加速度，注意：0是直接启动
    cmd[6] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[7] = 0x6B;                // 校验字节

    // 发送命令
    Emm_V5_Send1(Serial , cmd, 8);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_DownControl(struct UltraSerial *Serial , uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[8] = {0};

    // 装载命令
    cmd[0] = addr;                // 地址
    cmd[1] = 0xF6;                // 功能码
    cmd[2] = dir;                 // 方向
    cmd[3] = (uint8_t)(vel >> 8); // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0); // 速度(RPM)低8位字节
    cmd[5] = acc;                 // 加速度，注意：0是直接启动
    cmd[6] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[7] = 0x6B;                // 校验字节

    // 发送命令
    Emm_V5_Send2(Serial , cmd, 8);
}

void Emm_V5_Send1(struct UltraSerial *Serial , uint8_t *cmd, uint8_t len)
{
    Serial_SendArray( Serial , cmd , len);
}

void Emm_V5_Send2(struct UltraSerial *Serial , uint8_t *cmd, uint8_t len)
{
    Serial_SendArray( Serial , cmd , len);
}

void Emm_V5_GetCurrentLocation( struct UltraSerial *Serial , uint8_t addr)//02 36 01 00 00 00 03 6B 
{
    Serial_SendByte(Serial , addr);
    Serial_SendByte(Serial , 0x36);
    Serial_SendByte(Serial , 0x6B);
}



float TurnAnglePIDAdjust( struct UltraSerial * Serial , float current , float target )
{
    Down_RTPositPIDValue(Serial , target , current );
}

float TurnDownAngle(struct UltraSerial * Serial , float Angle  )
{
    if(DownMoveState == 1)
    {
        DownMoveState = 2;
    }
    if(DownMoveState == 2)
    {
        DownTagectAngle = Angle;
        DownMoveState = 0;
    }   
        
}

void USART2_IRQHandler(void)//
{
    static uint8_t RxDataFlag = 0 , DataRecieveState = 0 ; 
    // uint8_t decimal_value = 0;
    
    if (LL_USART_IsActiveFlag_RXNE(USART2))  // 判断接收中断标志
    {
        uint8_t received_data = LL_USART_ReceiveData8(USART2);  // 读取接收到的数据

        // // 通过串口回显收到的数据
        // while (!LL_USART_IsActiveFlag_TXE(USART2));  // 等待发送缓冲区空
        // LL_USART_TransmitData8(USART2, received_data);

        // Serial_Printf(&Usart2 , "received_data:%x\r\n" , received_data);
        
        // Serial_SendByte(&Usart2 , received_data);

        // Serial_Printf(&Usart2 , "1\r\n");

        // static MotorDataRecieve DataProcess;
        // static uint8_t ReFlag;
        // static uint8_t DataRecieveState;
        // static uint8_t RxDataFlag;
        // enum MotorDataRecieveSTATE state;
        // if(ReFlag == 0)
        // {
        //     ReFlag = 1;
        //     state = HEADER_STATE;
        // }
        // Serial_Printf(&Usart2 , "entered\r\n");
        // HAL_GPIO_WritePin(GPIOC , GPIO_PIN_13 , GPIO_PIN_RESET);



        if(DataRecieveState == 0)
        {
            if(received_data == 0x02)
            {
                DataRecieveState = 1;
            }
        }
        else if(DataRecieveState == 1)
        {
            if(received_data == 0x36)
            {
                DataRecieveState = 2;
            }
            else{
                DataRecieveState = 1;
            }
        }
        else if(DataRecieveState == 2)
        {
            DownMotorLocation_Array[RxDataFlag] = received_data;
            RxDataFlag++;
            if(RxDataFlag >= 6)
            {
                DownMotorLocation = (DownMotorLocation_Array[1] << 24) | (DownMotorLocation_Array[2] << 16) | (DownMotorLocation_Array[3] << 8) | DownMotorLocation_Array[4];
                DownMotorLocateDataGetFlag = 1;
                DataRecieveState = 0;
                RxDataFlag = 0;
            }
        }

        // Serial_Printf(&Usart2 , " DataRecieveState:%d\r\n" , DataRecieveState);


        // 在这里处理接收到的数据
        // switch(state)
        // {
        //     case(HEADER_STATE):
        //         // Serial_Printf(&Usart2 , "[Info]:HEADER_STATE\r\n");
        //         if(received_data == 0x02)
        //         {
        //             state = FUNC_STATE;
                    
        //         }
        //         else if(received_data == 0x01)return;
        //         break;
        //     case(FUNC_STATE):
        //         // Serial_Printf(&Usart2 , "[Info]:FUNC_STATE\r\n");
        //         if(received_data == 0x36)//电机实时位置
        //         {
        //             state = SYMBOL_STATE;
        //             // Serial_Printf(&Usart2 , "[Info]:WAS_SET_SYMBOL_STATE\r\n");
        //         }
        //         else {
        //             state = HEADER_STATE;
        //             return;
        //         }
        //         state = SYMBOL_STATE;
        //         break;
        //     case(SYMBOL_STATE):
        //         // Serial_Printf(&Usart2 , "[Info]:SYMBOL_STATE:%x\r\n" , received_data);
        //         if(received_data == 0x01)
        //         {
        //             DataProcess.Symbol = 0x01;
        //             state = LOCATIONDATA_STATE;
        //         }
        //         else if(received_data == 0x00)
        //         {
        //             DataProcess.Symbol = 0x00;
        //             state = LOCATIONDATA_STATE;
        //         }
        //         else 
        //         {
        //             state = HEADER_STATE;
        //             return;
        //         }
                
        //         break;
        //     case(LOCATIONDATA_STATE):
        //         // Serial_Printf(&Usart2 , "[Info]:LOCATIONDATA_STATE\r\n");
        //         DownMotorLocation_Array[DataRecieveState] = received_data;
        //         DataRecieveState++;
        //         if(DataRecieveState == 3)
        //         {
        //             DownMotorLocation_Array[DataRecieveState] = received_data;
        //             DataRecieveState = 0;
        //             state = CHECKBIT_STATE;
        //         }
        //         else{
        //             state = HEADER_STATE;
        //             return;
        //         }
        //     case(CHECKBIT_STATE):
        //         Serial_Printf(&Usart2 , "[Info]:CHECKBIT_STATE\r\n");
        //         if(received_data == 0x6B)
        //         {
        //             state = HEADER_STATE;
        //             ReFlag = 0;
        //         }
        //         else{
        //             state = HEADER_STATE;
        //             ReFlag = 0;
        //             return;
        //         }
        //         break;
        // }


        
    }
    
}

void USART3_IRQHandler(void)
{
    static uint8_t RxDataFlag = 0 , DataRecieveState = 0 ; 
    // uint8_t decimal_value = 0;
    
    if (LL_USART_IsActiveFlag_RXNE(USART3))  // 判断接收中断标志
    {
        uint8_t received_data = LL_USART_ReceiveData8(USART3);  // 读取接收到的数据
        
        if(DataRecieveState == 0)
        {
            if(received_data == 0x01)
            {
                DataRecieveState = 1;
            }
        }
        else if(DataRecieveState == 1)
        {
            if(received_data == 0x36)
            {
                DataRecieveState = 2;
            }
            else{
                DataRecieveState = 1;
            }
        }
        else if(DataRecieveState == 2)
        {
            UpMotorLocation_Array[RxDataFlag] = received_data;
            RxDataFlag++;
            if(RxDataFlag >= 6)
            {
                UpMotorLocation = (UpMotorLocation_Array[1] << 24) | (UpMotorLocation_Array[2] << 16) | (UpMotorLocation_Array[3] << 8) | UpMotorLocation_Array[4];
                UpMotorLocateDataGetFlag = 1;
                DataRecieveState = 0;
                RxDataFlag = 0;
            }
        }
    }


}
