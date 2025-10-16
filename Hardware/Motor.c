#include <Motor.h>
volatile uint8_t DownMotorLocation_Array[6];
volatile int32_t DownMotorLocation;
volatile int8_t DownMotorLocateDataGetFlag; // 下面电机接收数据标志位
volatile uint8_t UpMotorLocation_Array[6];
volatile int32_t UpMotorLocation;
volatile int8_t UpMotorLocateDataGetFlag; // 上面电机接收数据标志位
volatile float DownTagectAngle;
volatile uint8_t DownMoveState = 1; // 跨文件提供三种状态
volatile float UpTagectAngle;
volatile uint8_t UpMoveState = 1; // 跨文件提供三种状态

float UpCurrentAngle;               // 记录当前角度，用于归零
float DownCurrentAngle;             // 记录当前角度，用于归零
extern volatile float DownLocation; // x轴当前位置
extern volatile float UpLocation;   // y轴当前位置

extern struct Pid UpMotor_Pid , LeftMotor_Pid ;
extern struct UltraSerial Usart1, Usart2, Usart3;       // 初始化3种串口

/*提取串口读到的数据*/
extern float fRxData[8];
extern double dRxData[4];
extern int32_t iRxData[8];
// uint8_t

/**
 * @brief    速度模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Vel_UpControl(struct UltraSerial *Serial, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
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
    Emm_V5_Send1(Serial, cmd, 8);
}

/**
 * @brief    速度模式
 *
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Vel_DownControl(struct UltraSerial *Serial, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
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
    Emm_V5_Send2(Serial, cmd, 8);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_UpControl(struct UltraSerial *Serial, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 发送命令
    //   usart_SendCmd(cmd, 13);
    Emm_V5_Send1(Serial, cmd, 13);
}

/**
 * @brief    位置模式
 * @param    addr：电机地址
 * @param    dir ：方向        ，0为CW，其余值为CCW
 * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
 * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
 * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
 * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param    snF ：多机同步标志 ，false为不启用，true为启用
 * @retval   地址 + 功能码 + 命令状态 + 校验字节
 */
void Emm_V5_Pos_DownControl(struct UltraSerial *Serial, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )
    cmd[10] = raF;                 // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;                 // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B;                // 校验字节

    // 发送命令
    //   usart_SendCmd(cmd, 13);
    Emm_V5_Send1(Serial, cmd, 13);
}

void Emm_V5_Send1(struct UltraSerial *Serial, uint8_t *cmd, uint8_t len)
{
    Serial_SendArray(Serial, cmd, len);
}

void Emm_V5_Send2(struct UltraSerial *Serial, uint8_t *cmd, uint8_t len)
{
    Serial_SendArray(Serial, cmd, len);
}

void Emm_V5_GetCurrentLocation(struct UltraSerial *Serial, uint8_t addr) // 02 36 01 00 00 00 03 6B
{
    Serial_SendByte(Serial, addr);
    Serial_SendByte(Serial, 0x36);
    Serial_SendByte(Serial, 0x6B);
}


void Motor_TargetAngleControl( )
{
    float x = fRxData[0] , y = fRxData[1] , Vel = 50 , UpTargetAngle = 0 , DownTargetAngle = 0;
    UpTargetAngle = Up_RTPositPIDValue(&UpMotor_Pid , 0 , y);
    DownTargetAngle =  Down_RTPositPIDValue(&UpMotor_Pid , 0 , x);
    if(y >= 0)
    {
        Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CW , Vel , 0 ,  UpTargetAngle , false , false );//>0
    }
    else if(y < 0)
    {   
        Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CCW , Vel , 0 ,  UpTargetAngle , false , false );//<0
    }
    if(x >= 0)
    {
        Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CW, Vel , 0 ,  DownTargetAngle , false , false );//<0    
    }
    else if(x < 0)
    {
        Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CCW, Vel , 0 ,  DownTargetAngle , false , false );//<0    
    }
    
}


void TurnDownAngle(struct UltraSerial *Serial, float Angle)
{
    if (DownMoveState == 1)
    {
        DownMoveState = 2;
    }
    if (DownMoveState == 2)
    {
        DownTagectAngle = Angle;
        DownMoveState = 0;
    }
}

void TurnUpAngle(struct UltraSerial *Serial, float Angle)
{
    if (UpMoveState == 1)
    {
        UpMoveState = 2;
    }
    if (UpMoveState == 2)
    {
        UpTagectAngle = Angle;
        UpMoveState = 0;
    }
}

void SetUpCurrentAngle_0(void)
{
    UpCurrentAngle = UpLocation;
    UpLocation = 0; // 将当前的角度清零以继续使用
}

void SetDownCurrentAngle_0(void)
{
    DownCurrentAngle = DownLocation;
    DownLocation = 0; // 将当前的角度清零以继续使用
}

uint8_t GetUpMotorState(void)
{
    if (UpTagectAngle == 0 && UpMoveState == 1)
    {
        return Ready;
    }
    return Unready;
}

uint8_t GetDownMotorState(void)
{
    if (DownTagectAngle == 0 && DownMoveState == 1)
    {
        return Ready;
    }
    return Unready;
}

void USART2_IRQHandler(void) //
{
    static uint8_t RxDataFlag = 0, DataRecieveState = 0;
    // uint8_t decimal_value = 0;

    if (LL_USART_IsActiveFlag_RXNE(USART2)) // 判断接收中断标志
    {
        uint8_t received_data = LL_USART_ReceiveData8(USART2); // 读取接收到的数据

        if (DataRecieveState == 0)
        {
            if (received_data == 0x02)
            {
                DataRecieveState = 1;
            }
        }
        else if (DataRecieveState == 1)
        {
            if (received_data == 0x36)
            {
                DataRecieveState = 2;
            }
            else
            {
                DataRecieveState = 1;
            }
        }
        else if (DataRecieveState == 2)
        {
            DownMotorLocation_Array[RxDataFlag] = received_data;
            RxDataFlag++;
            if (RxDataFlag >= 6)
            {
                DownMotorLocation = (DownMotorLocation_Array[1] << 24) | (DownMotorLocation_Array[2] << 16) | (DownMotorLocation_Array[3] << 8) | DownMotorLocation_Array[4];
                DownMotorLocateDataGetFlag = 1;
                DataRecieveState = 0;
                RxDataFlag = 0;
            }
        }
    }
}

void USART3_IRQHandler(void)
{
    static uint8_t RxDataFlag = 0, DataRecieveState = 0;
    // uint8_t decimal_value = 0;

    if (LL_USART_IsActiveFlag_RXNE(USART3)) // 判断接收中断标志
    {
        uint8_t received_data = LL_USART_ReceiveData8(USART3); // 读取接收到的数据

        if (DataRecieveState == 0)
        {
            if (received_data == 0x01)
            {
                DataRecieveState = 1;
            }
        }
        else if (DataRecieveState == 1)
        {
            if (received_data == 0x36)
            {
                DataRecieveState = 2;
            }
            else
            {
                DataRecieveState = 1;
            }
        }
        else if (DataRecieveState == 2)
        {
            UpMotorLocation_Array[RxDataFlag] = received_data;
            RxDataFlag++;
            if (RxDataFlag >= 6)
            {
                UpMotorLocation = (UpMotorLocation_Array[1] << 24) | (UpMotorLocation_Array[2] << 16) | (UpMotorLocation_Array[3] << 8) | UpMotorLocation_Array[4];
                UpMotorLocateDataGetFlag = 1;
                DataRecieveState = 0;
                RxDataFlag = 0;
            }
        }
    }
}
