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
    函数:Left_RTIncrePIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值值 int GetGroup 使用存储数据的组数
    返回值:返回增量式PID的控制值
    特殊:和上一次的值相加左轮的值
*/
float Left_RTIncrePIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{
	float static LeftIncrePreviousBias , LeftIncreLastBias ;
	float LeftIncreError = TargetVelocity - CurrentVelocity;
	float Left_IncreSum = PID->Kp * ( LeftIncreError - LeftIncreLastBias ) + PID->Ki * LeftIncreError* PID->Time + PID->Kd * ( LeftIncreError - 2 * LeftIncreLastBias + LeftIncrePreviousBias ) / PID->Time;  
	LeftIncrePreviousBias = LeftIncreLastBias;
	LeftIncreLastBias = LeftIncreError;	
	
	return Left_IncreSum;
}

/*
    函数:Right_RTIncrePIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值值 int GetGroup 使用存储数据的组数
    返回值:返回增量式PID的控制值
    特殊:和上一次的值相加右轮的值
*/
float Right_RTIncrePIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
{
	float static RightIncrePreviousBias , RightIncreLastBias ;
	float RightIncreError = TargetVelocity - CurrentVelocity;
	
	float Right_IncreSum = PID->Kp * ( RightIncreError - RightIncreLastBias ) + PID->Ki * RightIncreError * PID->Time + PID->Kd * ( RightIncreError - 2 * RightIncreLastBias + RightIncrePreviousBias ) / PID->Time;  
	
	RightIncrePreviousBias = RightIncreLastBias;
	RightIncreLastBias = RightIncreError;	
	
	return Right_IncreSum;
}

/*
    函数:RTPositPIDValue
    参数:struct PID *PID 初始化的PID结构体  int TargetVelocity 目标值 int CurrentVelocity 当前值 int GetGroup 使用存储数据的组数
    返回值:返回位置式PID的控制值
    特殊:无
*/
float Left_RTPositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
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
float Right_RTPositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
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
float Right_AnglePositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
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
float Left_AnglePositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity )
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

