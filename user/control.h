#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f10x.h"

#define BRUSHLESS

  //电调 10初始化  2 3号260启动  1、4号300 500最大
	
#ifdef BRUSHLESS
	#define CH1_START	300
	#define CH2_START 260
	#define CH3_START 260
	#define CH4_START 300
	
	#define CH_END 500
#endif



float Limit_Duty(float duty);

#define Limit(value,max)     if(value>max)value=max;else if(value<-max)value=-max


#define Set_Brush_Speed(CH,duty) TIM_SetCompare##CH(TIM2,Limit_Duty(duty)*4095/100)
void Set_Brushless_Speed(int ch,int duty);

#define Brushless_Stop() TIM1->CCR1=230;TIM1->CCR2=230;TIM1->CCR3=230;TIM1->CCR4=230



#ifndef BRUSHLESS
	#define Set_Motor_Speed(CH,duty)	Set_Brush_Speed(CH,duty)
#else
	#define Set_Motor_Speed(CHN,duty)	Set_Brushless_Speed(CHN,duty)
#endif

void Brush_Init();
float Pitch_Control();
float Roll_Control();
void Fly_Control();


#define Height_Out_Max 50
#define Limit(value,max)    if(value>max)value=max;else if(value<-max)value=-max

void Brushless_Init();


typedef struct{
	float KP;
	float KD;
	float KI;
	float i;
	float last_err;
	float i_max;
	float i_time;
}PID_S;

void clear_i(PID_S * temp);

extern char Fly;
extern PID_S Pitch_PID_Single;
extern PID_S Roll_PID_Single;

extern float Angle_Speed_X_Out;
extern float Angle_Speed_Y_Out;


extern PID_S Roll_PID;
extern PID_S Pitch_PID;
extern PID_S ANGLE_SPEED_X_PID;
extern PID_S ANGLE_SPEED_Y_PID;
extern PID_S ANGLE_SPEED_Z_PID;
extern PID_S Height_PID;
extern float base_duty;

extern float pitch_target;   
extern float roll_target;
extern float CH1_Out,CH2_Out,CH3_Out,CH4_Out;
extern float Height_Out;
extern float Pitch_Out;
extern float Roll_Out;
#define clear_i(a) a.i=0
#endif