#ifndef __PID_H
#define __PID_H
#include "main.h"                  // Device header
struct Pid
{
	float Kp;
	float Ki;
	float Kd;
	float Time;
	int CurrentBias;//当前误差值
	float Last_bias;
	float Previous_bias;
	float Error_Sum;
	float OutPut_Limit;
};


void PID_Init(struct Pid *PID , float P , float I , float D , float Time);

float Up_RTIncrePIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity );
float Down_RTIncrePIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity );
float Up_RTPositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity );
float Down_RTPositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity );

float Down_AnglePositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity );
float Up_AnglePositPIDValue(struct Pid *PID , float TargetVelocity, float CurrentVelocity );

#endif
