#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f10x.h"



float Limit_Duty(float duty);

#define Set_Brush_Speed(CH,duty) TIM_SetCompare##CH(TIM2,Limit_Duty(duty)*4095/100)

void Brush_Init();
float Pitch_Control();
float Roll_Control();
void Fly_Control();

typedef struct{
	float KP;
	float KD;
	float KI;
	float i;
	float last_err;
	float i_max;
}PID_S;

extern char Fly;
#endif