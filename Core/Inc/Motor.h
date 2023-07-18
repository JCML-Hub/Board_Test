//
// Created by mulai on 2023/6/18.
//

#ifndef _MOTOR_H
#define _MOTOR_H
#include "main.h"

#define DISABLE_TIM1//如果使用到另外的一个timer请删除该段
#define DISABLE_ENCODER
#undef DISABLE_ENCODER
//定义所用的定时器//
#define MotorTimer1 htim8
#define MotorTimer2 htim1
//定义每个电机所用的通道//
#define MotorA_Ch1 TIM_CHANNEL_4
#define MotorA_Ch2 TIM_CHANNEL_3
#define MotorB_Ch1 TIM_CHANNEL_2
#define MotorB_Ch2 TIM_CHANNEL_1

#define MotorC_Ch1 TIM_CHANNEL_1
#define MotorC_Ch2 TIM_CHANNEL_2
#define MotorD_Ch1 TIM_CHANNEL_3
#define MotorD_Ch2 TIM_CHANNEL_4

//定义编码器的接入引脚
#define Encoder_Timer1 htim2
#define Encoder_Timer2 htim3
#define Encoder_TimeCounter htim6

typedef struct
{
    float M1_ActualSpeed;
    float M2_ActualSpeed;//接收来自电机的原始速度
}_Motor;


void Motor_Init(void);
void Control_A(int32_t Speed);
void Control_B(int32_t Speed);
void Control_C(int32_t Speed);
void Control_D(int32_t Speed);
void Servo1_Set(uint16_t Angle);
void GetSpeed(_Motor *speed);
void Encode_CallBack(void);
#endif
