#include "control.h"
#include "set.h"
#include "angle.h"
#include "SR04.h"

//期望的俯仰、横滚、偏航角
float pitch_target=3.25;     //22 
float roll_target=0;  //48.5
float yaw_target=0;
float height_target=80;   //单位cm
float Angle_Speed_X_Out=0;
float Angle_Speed_Y_Out=0;
float Angle_Speed_Z_Out=0;
float Height_Out=0;
float Pitch_Out=0;
float Roll_Out=0;
char Fly=0;

float base_duty=0; //50
float CH1_Out,CH2_Out,CH3_Out,CH4_Out;

//PID 结构体参数
//带Single是单级PID
//不带Single是串级PID，内环为角速度环，外环为角度环
PID_S Pitch_PID_Single={2,0,0,0,0,1000,1}; 
PID_S Roll_PID_Single={-2,0,0,0,0,1000,1};


PID_S Roll_PID={0.03,0,0.0033,0,0,1000,1};
PID_S Pitch_PID={-0.03,0,-0.0033,0,0,1000,1};
PID_S ANGLE_SPEED_Y_PID={-3.5,-25,0,0,0,1000,1};
PID_S ANGLE_SPEED_X_PID={-3.5,-25,0,0,0,1000,1};
PID_S ANGLE_SPEED_Z_PID={15,0,0,0,0,1000,1};
PID_S Height_PID={0.5,0,0.0055,0,0,9000,0.005};  //0.0055

float Limit_Duty(float duty){
	float max_duty=100;
	if(Debug){
		max_duty=50;
	}   
	if(duty>max_duty){
		duty=max_duty;
	}else if(duty<0){
		duty=0;
	}
	return duty;
}



void Set_Brushless_Speed(int ch,int duty){
	if(ch>4||ch<1)
		return ;
	switch(ch){
		case 1:
			TIM1->CCR1=Limit_Duty(duty)*(CH_END-CH1_START)/100+CH1_START;
			break;
		case 2:
			TIM1->CCR2=Limit_Duty(duty)*(CH_END-CH2_START)/100+CH2_START;
			break;
		case 3:
			TIM1->CCR3=Limit_Duty(duty)*(CH_END-CH3_START)/100+CH3_START;
			break;
		case 4:
			TIM1->CCR4=Limit_Duty(duty)*(CH_END-CH4_START)/100+CH4_START;
			break;
	}
}

float PID_Control(PID_S *PID,float target,float now){
	float err;
	float err_dt;
	float result;
	err=target-now;
	
	err_dt=err-PID->last_err;
	
	PID->last_err=err;
	
	PID->i+=err*PID->i_time;
	
	Limit(PID->i,PID->i_max);
	
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

void Brushless_Init(){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11 ;    //B3 B10 B11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 35;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 4999;            //f=200hz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;								//duty=50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  
	TIM_Cmd(TIM1, ENABLE);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	delay_us(5000);
	
	TIM1->CCR1=20;
	TIM1->CCR2=20;
	TIM1->CCR3=20;
	TIM1->CCR4=20;
	
	
}
int height_cnt;  //高度环计算计数
#define Heignt_CNT_MAX 100
float Height_Out_Dt;

void Fly_Control(){

	float height_out_temp;
	float height_out_sub;
	
	
	 
	/*
	if(height_cnt>=Heignt_CNT_MAX){	
		height_out_temp=PID_Control(&Height_PID,height_target,height);
		
		//高度环限幅
		if(height_out_temp>Height_Out_Max){
			height_out_temp=Height_Out_Max;
		}else if(height_out_temp<-Height_Out_Max){
			height_out_temp=-Height_Out_Max;
		}
		
		height_out_sub=height_out_temp-Height_Out;       //本次高度环与上次高度环输出做差，得到高度环改变量
	
		Height_Out_Dt=height_out_sub/Heignt_CNT_MAX;     //将100ms一次算出的高度环改变量，分20次加上。
		
		height_cnt=0;
	}else{
		height_cnt++;
	}
	
	Height_Out+=Height_Out_Dt;
	*/
	
	
	
	if(!Fly){
		Brushless_Stop();
		return ;
	}
	


//	Height_Out=PID_Control(&Height_PID,height_target,height);
//	Limit(Height_Out,Height_Out_Max);
//	
//	Roll_Out=PID_Control(&Roll_PID,roll_target,roll);
//	Limit(Roll_Out,20);

	Angle_Speed_X_Out=PID_Control(&ANGLE_SPEED_X_PID,Roll_Out,angle_speed_X);
	
	
	
//	Pitch_Out=PID_Control(&Pitch_PID,pitch_target,pitch);
//	Limit(Pitch_Out,20);
	
	Angle_Speed_Y_Out=PID_Control(&ANGLE_SPEED_Y_PID,Pitch_Out,angle_speed_Y);
//	Angle_Speed_Z_Out=PID_Control(&ANGLE_SPEED_Z_PID,0,angle_speed_Z);
//	
	
	
	//串级PID
	CH1_Out=-Angle_Speed_X_Out+Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Height_Out;  //以角度向前倾，向左倾为标准
	CH2_Out=-Angle_Speed_X_Out-Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Height_Out;
	CH3_Out=Angle_Speed_X_Out-Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Height_Out;
	CH4_Out=Angle_Speed_X_Out+Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Height_Out;
	
	Set_Motor_Speed(1,CH1_Out);
	Set_Motor_Speed(2,CH2_Out);
	Set_Motor_Speed(3,CH3_Out);
	Set_Motor_Speed(4,CH4_Out);
}

	



