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
extern PID_S Pitch_PID_Single;
extern PID_S Roll_PID_Single;

extern float Angle_Speed_X_Out;
extern float Angle_Speed_Y_Out;


extern PID_S Roll_PID;
extern PID_S Pitch_PID;
extern PID_S ANGLE_SPEED_X_PID;
extern PID_S ANGLE_SPEED_Y_PID;
extern PID_S ANGLE_SPEED_Z_PID;
extern float base_duty;

extern float pitch_target;   
extern float roll_target;
extern float CH1_Out,CH2_Out,CH3_Out,CH4_Out;

#endif