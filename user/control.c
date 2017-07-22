#include "control.h"
#include "set.h"
#include "angle.h"

//期望的俯仰、横滚、偏航角
float pitch_target=5;   
float roll_target=-1;
float yaw_target=0;

char Fly=0;

PID_S Pitch_PID={2,0,0,0,0,1000};
PID_S Roll_PID={-2,0,0,0,0,1000};

float Limit_Duty(float duty){
	float max_duty=96;
	if(Debug){
		max_duty=30;
	}
	if(duty>max_duty){
		duty=max_duty;
	}else if(duty<0){
		duty=0;
	}
	return duty;
}

float PID_Control(PID_S *PID,float target,float now){
	float err;
	float err_dt;
	float result;
	err=target-now;
	
	err_dt=err-PID->last_err;
	
	PID->last_err=err;
	
	PID->i+=err;
	
	if(PID->i>PID->i_max){
		PID->i=PID->i_max;
	}
	
	result = err * PID->KP  +   err_dt * PID->KD   +   PID->i * PID->KI;
	return result;
}


void Brush_Init(){   //TIM2
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_11 ;    //B3 B10 B11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 4095;            //f=17khz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;								//duty=50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  
	TIM_Cmd(TIM2, ENABLE);

	//TIM_CtrlPWMOutputs(TIM2, ENABLE);
}


void Fly_Control(){
	float Pitch_Out;
	float Roll_Out;
	float CH1_Out,CH2_Out,CH3_Out,CH4_Out;
	//Pitch_Out=Pitch_Control();
	//Pitch_Out=PID_Control(&Pitch_PID,pitch_target,pitch);
	Pitch_Out=0;
	Roll_Out=PID_Control(&Roll_PID,roll_target,roll);
	
	CH1_Out=Pitch_Out-Roll_Out;  //以角度向前倾，向左倾为标准
	CH2_Out=-Pitch_Out-Roll_Out;
	CH3_Out=-Pitch_Out+Roll_Out;
	CH4_Out=Pitch_Out+Roll_Out;
	
	if(Fly){
		Set_Brush_Speed(1,CH1_Out);
		Set_Brush_Speed(2,CH2_Out);
		Set_Brush_Speed(3,CH3_Out);
		Set_Brush_Speed(4,CH4_Out);
	}else{
		Set_Brush_Speed(1,0);
		Set_Brush_Speed(2,0);
		Set_Brush_Speed(3,0);
		Set_Brush_Speed(4,0);
	}
	
}





#define SINGLE_PID 1    //单级PID
float pitch_P=2;
float pitch_D=0;
float pitch_I=0;
float pitch_err_integral;
float last_pitch_err;
#define INTEGRAL_MAX 1000
float Pitch_Control(){
	float pitch_err;
	float pitch_err_dt;
	pitch_err=pitch_target-pitch;
	pitch_err_dt=pitch_err-last_pitch_err;
	pitch_err_integral+=pitch_err;
	
	last_pitch_err=pitch_err;
	
	if(pitch_err_integral>INTEGRAL_MAX){
		pitch_err_integral=INTEGRAL_MAX;
	}
	
	return pitch_err*pitch_P+pitch_err_dt*pitch_D+pitch_err_integral*pitch_I;
}

float Roll_Control(){
	return 0;
}


