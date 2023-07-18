//
// Created by mulai on 2023/6/18.
//
#include "tim.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_gpio.h"
#include "Motor.h"

_Motor _motor;
short motorA,MotorB;

/***********************************************************
*@名称 :Motor_Init
*@描述	:Motor_Init，包含驱动PWM的初始化以及旋转编码器定时器的初始化
*@参数	:无
*@返回值	:无
*@作者	:JCML
*@日期	:2023-04-06
***********************************************************/

void Motor_Init(void)
{
  HAL_TIM_PWM_Start(&MotorTimer1,MotorA_Ch1);
  HAL_TIM_PWM_Start(&MotorTimer1,MotorA_Ch2);
  HAL_TIM_PWM_Start(&MotorTimer1,MotorB_Ch1);
  HAL_TIM_PWM_Start(&MotorTimer1,MotorB_Ch2);
#ifndef DISABLE_TIM1
  HAL_TIM_PWM_Start(&MotorTimer2,MotorC_Ch1);
  HAL_TIM_PWM_Start(&MotorTimer2,MotorC_Ch2);
  HAL_TIM_PWM_Start(&MotorTimer2,MotorD_Ch1);
  HAL_TIM_PWM_Start(&MotorTimer2,MotorD_Ch2);
#endif
#ifndef DISABLE_ENCODER
  HAL_TIM_Encoder_Start(&Encoder_Timer1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&Encoder_Timer2, TIM_CHANNEL_ALL);//开启定时器计数
  HAL_TIM_Base_Start_IT(&Encoder_TimeCounter);//开启用于获取计数的定时器
#endif
}

/***********************************************************
*@名称 :Control_A  & 	Control_B  & 	Control_C  & 	Control_D
*@描述	:四个轮子的PWM赋值函数
*@参数	:Speed：正负表方向 范围由Tim设定的自动重装值决定
*@返回值	:无
*@作者	:JCML
*@日期	:2023-07-11
***********************************************************/
void Control_A(int32_t Speed) {
  Speed=-Speed;
  if (Speed >= 0) {//w为防止出现死区，此处直接将另外一个的PWM置为，此处为其判断语句
            __HAL_TIM_SetCompare(&MotorTimer1, MotorA_Ch1, Speed);
            __HAL_TIM_SetCompare(&MotorTimer1, MotorA_Ch2, 0);

  } else {
            __HAL_TIM_SetCompare(&MotorTimer1, MotorA_Ch1, 0);
            __HAL_TIM_SetCompare(&MotorTimer1, MotorA_Ch2, -Speed);}//速度为负取相反数
}
void Control_B(int32_t Speed) {
  if (Speed >= 0) {//w为防止出现死区，此处直接将另外一个的PWM置为，此处为其判断语句
            __HAL_TIM_SetCompare(&MotorTimer1, MotorB_Ch1, Speed);
            __HAL_TIM_SetCompare(&MotorTimer1, MotorB_Ch2, 0);
  } else {
            __HAL_TIM_SetCompare(&MotorTimer1, MotorB_Ch1, 0);
            __HAL_TIM_SetCompare(&MotorTimer1, MotorB_Ch2, -Speed);}//速度为负取相反数
}
#ifndef DISABLE_TIM1
void Control_C(int32_t Speed) {
  if (Speed >= 0) {//w为防止出现死区，此处直接将另外一个的PWM置为，此处为其判断语句
            __HAL_TIM_SetCompare(&MotorTimer2, MotorC_Ch1, Speed);
            __HAL_TIM_SetCompare(&MotorTimer2, MotorC_Ch2, 0);
  } else {
            __HAL_TIM_SetCompare(&MotorTimer2, MotorC_Ch1, 0);
            __HAL_TIM_SetCompare(&MotorTimer2, MotorC_Ch2, -Speed);}//速度为负取相反数
}
void Control_D(int32_t Speed) {
  if (Speed >= 0) {//w为防止出现死区，此处直接将另外一个的PWM置为，此处为其判断语句
            __HAL_TIM_SetCompare(&MotorTimer2, MotorD_Ch1, Speed);
            __HAL_TIM_SetCompare(&MotorTimer2, MotorD_Ch2, 0);
  } else {
            __HAL_TIM_SetCompare(&MotorTimer2, MotorD_Ch1, 0);
            __HAL_TIM_SetCompare(&MotorTimer2, MotorD_Ch2, -Speed);}//速度为负取相反数
}
#endif

#ifndef DISABLE_ENCODER
/***********************************************************
*@名称 :GetSpeed
*@描述	:获取旋转编码器的速度值
*@参数	:_Motor结构体，该函数放在while循环获取
*@返回值	:无
*@作者	:JCML
*@日期	:2023-06-18
***********************************************************/
void GetSpeed(_Motor *speed)
{
  speed->M1_ActualSpeed = -(float )motorA;
  speed->M2_ActualSpeed = (float )motorA;//获取当前轮子的速度,此处直接将速度转为float型，可在后面加上滤波
}


void Encode_CallBack(void)
{
  motorA = ((short)__HAL_TIM_GET_COUNTER(&Encoder_Timer1));//读取M1的旋转次数,此处取的是霍尔编码器一周计数加一
  MotorB = ((short)__HAL_TIM_GET_COUNTER(&Encoder_Timer2));//读取M2的旋转次数
  __HAL_TIM_SET_COUNTER(&Encoder_Timer1, 0);
  __HAL_TIM_SET_COUNTER(&Encoder_Timer2, 0);//将两个计数器清零
}
/***********************************************************
*@名称 :HAL_TIM_PeriodElapsedCallback
*@描述	:定时器的回调函数,如果有冲突请将其放在main.c中，来调用Encode_CallBack更新计数值
*@参数	:TIM_HandleTypeDef
*@返回值	:无
*@作者	:JCML
*@日期	:2023-06-18
***********************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//每100ms进一次中断
{
  if(htim==(&Encoder_TimeCounter))
  {
    Encode_CallBack();
  }
}

#endif