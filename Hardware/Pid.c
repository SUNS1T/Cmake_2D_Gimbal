#include "Pid.h"


/*
    函数:PID_Init
    参数:struct PID *PID 输入创建的PID , 初始化P值 , 初始化I值 , 初始化D值 , 初始化间隔时间的值
    返回值:无
    特殊:这是一个初始化PID的函数
*/
void PID_Init(struct Pid *PID , float P , float I , float D , float Time)
{
	PID->Kp = P;
	PID->Ki = I;
	PID->Kd = D;
	PID->CurrentBias = 0;
	PID->Time = Time;
}

/*
    函数:Up_RTIncrePIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值值 int GetGroup 使用存储数据的组数
    返回值:返回增量式PID的控制值
    特殊:和上一次的值相加左轮的值
*/
float Up_RTIncrePIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{
	float static UpIncrePreviousBias , UpIncreLastBias ;
	float UpIncreError = TargetVelocity - CurrentVelocity;
	float Up_IncreSum = PID->Kp * ( UpIncreError - UpIncreLastBias ) + PID->Ki * UpIncreError* PID->Time + PID->Kd * ( UpIncreError - 2 * UpIncreLastBias + UpIncrePreviousBias ) / PID->Time;  
	UpIncrePreviousBias = UpIncreLastBias;
	UpIncreLastBias = UpIncreError;	
	
	return Up_IncreSum;
}

/*
    函数:Down_RTIncrePIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值值 int GetGroup 使用存储数据的组数
    返回值:返回增量式PID的控制值
    特殊:和上一次的值相加右轮的值
*/
float Down_RTIncrePIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{
	float static DownIncrePreviousBias , DownIncreLastBias ;
	float DownIncreError = TargetVelocity - CurrentVelocity;
	
	float Down_IncreSum = PID->Kp * ( DownIncreError - DownIncreLastBias ) + PID->Ki * DownIncreError * PID->Time + PID->Kd * ( DownIncreError - 2 * DownIncreLastBias + DownIncrePreviousBias ) / PID->Time;  
	
	DownIncrePreviousBias = DownIncreLastBias;
	DownIncreLastBias = DownIncreError;	
	
	return Down_IncreSum;
}

/*
    函数:RTPositPIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值 int GetGroup 使用存储数据的组数
    返回值:返回位置式PID的控制值
    特殊:无
*/
float Up_RTPositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{

	PID->CurrentBias = TargetVelocity - CurrentVelocity;
	PID->Error_Sum += PID->CurrentBias * PID->Time;
//	/*-------------积分限幅-------------*/
//	if(PID->Error_Sum > 500)
//	{
//		PID->Error_Sum = 500;
//	}
//	else if(PID->Error_Sum < -500)
//	{
//		PID->Error_Sum = -500;
//	}
//	/*-------------积分限幅-------------*/
	float ControlVelocity = PID->Kp * PID->CurrentBias + PID->Ki * PID->Error_Sum +PID->Kd * ( PID->CurrentBias - PID->Last_bias ) / PID->Time;  
	PID->Previous_bias = PID->Last_bias;
	PID->Last_bias = PID->CurrentBias;

	return ControlVelocity;
}

/*
    函数:RTPositPIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值 int GetGroup 使用存储数据的组数
    返回值:返回位置式PID的控制值
    特殊:无
*/
float Down_RTPositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{

	PID->CurrentBias = TargetVelocity - CurrentVelocity;
	PID->Error_Sum += PID->CurrentBias * PID->Time;
//	/*-------------积分限幅-------------*/
//	if(PID->Error_Sum > 500)
//	{
//		PID->Error_Sum = 500;
//	}
//	else if(PID->Error_Sum < -500)
//	{
//		PID->Error_Sum = -500;
//	}
//	/*-------------积分限幅-------------*/
	float ControlVelocity = PID->Kp * PID->CurrentBias + PID->Ki * PID->Error_Sum +PID->Kd * ( PID->CurrentBias - PID->Last_bias ) / PID->Time;  
	PID->Previous_bias = PID->Last_bias;
	PID->Last_bias = PID->CurrentBias;

	return ControlVelocity;
}


/*
    函数:RTPositPIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值 int GetGroup 使用存储数据的组数
    返回值:返回位置式PID的控制值
    特殊:无
*/
float Down_AnglePositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{

	PID->CurrentBias = TargetVelocity - CurrentVelocity;
	PID->Error_Sum += PID->CurrentBias * PID->Time;
//	/*-------------积分限幅-------------*/
//	if(PID->Error_Sum > 500)
//	{
//		PID->Error_Sum = 500;
//	}
//	else if(PID->Error_Sum < -500)
//	{
//		PID->Error_Sum = -500;
//	}
//	/*-------------积分限幅-------------*/
	float ControlVelocity = PID->Kp * PID->CurrentBias + PID->Ki * PID->Error_Sum +PID->Kd * ( PID->CurrentBias - PID->Last_bias ) / PID->Time;  
	PID->Previous_bias = PID->Last_bias;
	PID->Last_bias = PID->CurrentBias;

	return ControlVelocity;
}


/*
    函数:RTPositPIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值 int GetGroup 使用存储数据的组数
    返回值:返回位置式PID的控制值
    特殊:无
*/
float Up_AnglePositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{

	PID->CurrentBias = TargetVelocity - CurrentVelocity;
	PID->Error_Sum += PID->CurrentBias * PID->Time;
//	/*-------------积分限幅-------------*/
//	if(PID->Error_Sum > 500)
//	{
//		PID->Error_Sum = 500;
//	}
//	else if(PID->Error_Sum < -500)
//	{
//		PID->Error_Sum = -500;
//	}
//	/*-------------积分限幅-------------*/
	float ControlVelocity = PID->Kp * PID->CurrentBias + PID->Ki * PID->Error_Sum +PID->Kd * ( PID->CurrentBias - PID->Last_bias ) / PID->Time;  
	PID->Previous_bias = PID->Last_bias;
	PID->Last_bias = PID->CurrentBias;

	return ControlVelocity;
}

