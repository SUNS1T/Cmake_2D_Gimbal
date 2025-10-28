#include <Motor.h>
#include "math.h"
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

extern volatile uint8_t PhotoelectricSensorStatus;//跨文件提供光电门的状态

float UpCurrentAngle;               // 记录当前角度，用于归零
float DownCurrentAngle;             // 记录当前角度，用于归零
extern volatile float DownLocation; // x轴当前位置
extern volatile float UpLocation;   // y轴当前位置

// extern struct Pid UpMotor_Pid , LeftMotor_Pid ;
extern struct Pid UpMotor_Pid_Black, DownMotor_Pid_Black , \
            UpMotor_Pid_Green, DownMotor_Pid_Green  , \
            UpMotor_Pid_Red, DownMotor_Pid_Red  ;
extern volatile uint8_t Func_;
extern struct UltraSerial Usart1, Usart2, Usart3;       // 初始化3种串口
extern struct UltraSerialDataDeal frame;
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

/*---------发射部分---------*/
extern uint32_t Shoot_CurrentTime;
uint8_t HaveShoot_Flag;
uint8_t BallPassDetect(void)
{
    if(HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_2) == GPIO_PIN_SET)
    {
        return 1;
    }
    else{
        return 0;
    }
}

void ShootInstruction(void)
{
    Emm_V5_Pos_UpControl(&Usart1, 0x01, 0x01, 1650, 0, 800, false, false);
}

uint8_t GetGentleData(int x , int y)
{
    static int DataDeal;
    if( ( x >= -6 && x <= 6 ) && ( y >= -6 && x <= 6 ) )
    {
        DataDeal++;
        if(DataDeal >= 10)
        {
            DataDeal = 0 ; 
            return 1;
        }
    }
        return 0;
}

void ShootBall(uint8_t x , uint8_t y ,uint8_t color)//挡住是0v
{
    static uint8_t ShootState = 0  , lastcolor = 0;
    static uint32_t ShootTime = 0;
    if(lastcolor != color && Func_ != 0x04)
    {
        uint8_t DetectDataWave = GetGentleData(x , y);
        if( DetectDataWave == 1 && ShootState == 0)
        {
            if(color == 0x01)
            {
                OLED_ShowString(0 , 48 , "Black Shoot!" , OLED_6X8);
            }
            else if(color == 0x02)
            {
                OLED_ShowString(0 , 48 , "Green Shoot!" , OLED_6X8);
            }
            else if(color == 0x03)
            {
                OLED_ShowString(0 , 48 , "Red Shoot!" , OLED_6X8);
            }
                
            OLED_Update();  
            ShootInstruction();
            
            ShootState = 1;
            ShootTime = Shoot_CurrentTime;//获取当前时间
        }
        else if(ShootState == 1)
        {
            // OLED_ShowStr 
            
            if( PhotoelectricSensorStatus == 0 )//未检测到发送
            {
                OLED_ShowNum(64 , 40 , ShootTime , 8 , OLED_6X8 );
                OLED_ShowNum(64 , 32 , Shoot_CurrentTime , 8 , OLED_6X8 );

                if( Shoot_CurrentTime >= ShootTime + 2000 )//非阻塞判断，隔2s
                {
                    OLED_ShowString(0 , 40 , "Un_Detect!" , OLED_6X8);
                    OLED_Update();
                    ShootTime = Shoot_CurrentTime;
                    ShootInstruction();
                }
            }
            else if (PhotoelectricSensorStatus == 1)//检测到当前已经发送
            {
                OLED_ShowString(0 , 40 , "Detect!" , OLED_6X8);
                OLED_Update();
                ShootState = 0;
                HaveShoot_Flag = 0;
                lastcolor = color;
            }
        }
    }
    
    
    
}


/*---------发射部分---------*/

void TurnMethod(int x  , int y , int Vel)
{
    if(y >= 0)
    {
        Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CCW , Vel , 0 ,  -y , false , false );//>0
    }
    else if(y < 0)
    {   
        Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CW , Vel , 0 ,  y , false , false );//<0
    }
    if(x >= 0)
    {
        Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CW, 350 , 0 ,  -x , false , false );//<0    
    }
    else if(x < 0)
    {
        Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CCW, 350 , 0 ,  x , false , false );//<0    
    }
    ShootBall(x , y , Func_);
}



void Motor_TargetAngleControl( )
{
    OLED_ShowHexNum(64, 48, Func_ , 2, OLED_6X8);
    float x = fRxData[0] , y = fRxData[1] + 60 , Vel = 50 , UpTargetAngle = 0 , DownTargetAngle = 0;
    if(frame.dataframeinlaw == LEGAL)//检测包的合法性
    {
        if(Func_ == 0x01)//黑色识别
        {
            UpTargetAngle = Up_RTPositPIDValue(&UpMotor_Pid_Black , 0 , y);
            DownTargetAngle =  Down_RTPositPIDValue(&UpMotor_Pid_Black , 0 , x);
            if(y >= 0)
            {
                Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CCW , Vel , 0 ,  -UpTargetAngle , false , false );//>0
            }
            else if(y < 0)
            {   
                Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CW , Vel , 0 ,  UpTargetAngle , false , false );//<0
            }
            if(x >= 0)
            {
                Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CW, 350 , 0 ,  -DownTargetAngle , false , false );//<0    
            }
            else if(x < 0)
            {
                Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CCW, 350 , 0 ,  DownTargetAngle , false , false );//<0    
            }
            // TurnMethod( DownTargetAngle , UpTargetAngle , Vel);
            ShootBall(x , y , Func_);
        }
        else if(Func_ == 0x02)//绿色识别
        {
            UpTargetAngle = Up_RTPositPIDValue(&UpMotor_Pid_Green , 0 , y);
            DownTargetAngle =  Down_RTPositPIDValue(&UpMotor_Pid_Green , 0 , x);
            if(y >= 0)
            {
                Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CCW , Vel , 0 ,  -UpTargetAngle , false , false );//>0
            }
            else if(y < 0)
            {   
                Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CW , Vel , 0 ,  UpTargetAngle , false , false );//<0
            }
            if(x >= 0)
            {
                Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CW, 350 , 0 ,  -DownTargetAngle , false , false );//<0    
            }
            else if(x < 0)
            {
                Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CCW, 350 , 0 ,  DownTargetAngle , false , false );//<0    
            }
            // TurnMethod( DownTargetAngle , UpTargetAngle , Vel);
            ShootBall(x , y , Func_);
        }
        else if(Func_ == 0x03)//红色识别
        {
            UpTargetAngle = Up_RTPositPIDValue(&UpMotor_Pid_Red , 0 , y);
            DownTargetAngle =  Down_RTPositPIDValue(&UpMotor_Pid_Red , 0 , x);
            if(y >= 0)
            {
                Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CCW , Vel , 0 ,  -UpTargetAngle , false , false );//>0
            }
            else if(y < 0)
            {   
                Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CW , Vel , 0 ,  UpTargetAngle , false , false );//<0
            }
            if(x >= 0)
            {
                Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CW, 350 , 0 ,  -DownTargetAngle , false , false );//<0    
            }
            else if(x < 0)
            {
                Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CCW, 350 , 0 ,  DownTargetAngle , false , false );//<0    
            }
            ShootBall(x , y , Func_);

        }
        else if(Func_ == 0x04)
        {
            Up_RTPositPIDValue(&UpMotor_Pid_Black , 0 , 0);
            Down_RTPositPIDValue(&UpMotor_Pid_Black , 0 , 0);
            Up_RTPositPIDValue(&UpMotor_Pid_Green , 0 , 0);
            Down_RTPositPIDValue(&UpMotor_Pid_Green , 0 , 0);
            Up_RTPositPIDValue(&UpMotor_Pid_Red , 0 , 0);
            Down_RTPositPIDValue(&UpMotor_Pid_Red , 0 , 0);
            Emm_V5_Pos_UpControl(&Usart3 , UpAdr , CCW , Vel , 0 ,  0 , false , false );//>0
            Emm_V5_Pos_DownControl(&Usart2 , DownAdr , CCW, 350 , 0 ,  0 , false , false );//<0    

        }
    }  
    // 
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