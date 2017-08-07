#include "control.h"
#include "set.h"
#include "angle.h"
#include "SR04.h"


#define HEIGHT_CONSTANT  70
#define UP_DUTY			35
//期望的俯仰、横滚、偏航角
float pitch_target=-3.5;     //3.25
float roll_target=0.3;  //48.5
float yaw_target=0.9;
float height_target=HEIGHT_CONSTANT;   //单位cm
float Angle_Speed_X_Out=0;
float Angle_Speed_Y_Out=0;
float Angle_Speed_Z_Out=0;
float Accel_Speed_Z_Out=0; //加速度环 高度环的内环
float Height_Out=0;
float Pitch_Out=0;
float Roll_Out=0;
float Velocity_Out=0;
char Fly=0;

float height_offset; 		//高度初始值

float base_duty=20; //50
float CH1_Out,CH2_Out,CH3_Out,CH4_Out;

//PID 结构体参数
//带Single是单级PID
//不带Single是串级PID，内环为角速度环，外环为角度环
PID_S Pitch_PID_Single={2,0,0,0,0,1000};    
PID_S Roll_PID_Single={-2,0,0,0,0,1000};


PID_S Roll_PID={0.25,0,0.1,0,0,2};
PID_S Pitch_PID={-0.25,0,-0.1,0,0,2};
PID_S ACCEL_SPEED_Z_PID={7,0,0,0,0,2};
PID_S ANGLE_SPEED_Y_PID={-5,-60,-1,0,0,5};
PID_S ANGLE_SPEED_X_PID={-5,-60,-1,0,0,5};
PID_S ANGLE_SPEED_Z_PID={10,0,0,0,0,1000};
PID_S Height_PID={0.3,0,0,0,0,9000};  //0.0055

PID_S Velocity_PID={0.32,5,0.77,0,0,1000};

Fly_State  State=STOP;

Kal_Struct kal_acc_z={1,0,0.00001,0.0012,0,1};

float Limit_Duty(float duty){
	float max_duty=DUTY_MAX;
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

void Fly_Init(void){      //解锁飞行，初始化高度、yaw
	State=FLY_WAIT;
	base_duty=28;           //FLY_WAIT状态，占空比可以手动更改
}

void Fly_Up(void){		//起飞
	State=FLY;
	//base_duty=UP_DUTY;
	
}
void Fly_Stop(void){
	State=STOP;
}

void Fly_Land(void){
	State=LAND;
}

void Fly_Land_Process(void){		//降落过程
	if(State!=LAND)
		return ;
	
	if(height_target>13){
		height_target-=0.2;   //5ms减0.2cm  0.2*200=40cm    1s降低40cm
	}else{
		height_target=13; 
		base_duty=20;
	}
	
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
	
	err_dt*=0.384;
	err_dt+=PID->last_d*0.615;
	
	PID->last_err=err;
	
	PID->i+=err*I_TIME;
	
	Limit(PID->i,PID->i_max);
	PID->last_d=err_dt;
	
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

float Height_Control(){
	
}

void Fly_Control(){

	float height_out_temp;
	float height_out_sub;
	

	if((State==STOP)||Pram_Error){
		Brushless_Stop();
		return ;
	}
	
	Fly_Land_Process();
	
	/*
	if(height_cnt>=Heignt_CNT_MAX){	
		height_out_temp=PID_Control(&Height_PID,height_target,height);
		
		
		
		height_out_sub=height_out_temp-Height_Out;       //本次高度环与上次高度环输出做差，得到高度环改变量
	
		Height_Out_Dt=height_out_sub/Heignt_CNT_MAX;     //将100ms一次算出的高度环改变量，分20次加上。
		
		height_cnt=0;
	}else{
		height_cnt++;
	}
	
	Height_Out+=Height_Out_Dt;
	*/
	
	
//	Height_Out=PID_Control(&Height_PID,height_target,height);
//	Limit(Height_Out,30);
	
	

	if(State==FLY_WAIT){       //即FLY_State状态
		;
		
	}else if(State==FLY){
		
		Height_Out=PID_Control(&Height_PID,height_target,height);
		Limit(Height_Out,40);
		
		Velocity_Out=PID_Control(&Velocity_PID,Height_Out,SR04_V);   //测试速度环用，正式时要删除
		Limit(Velocity_Out,30); 

		//base_duty=Height_Out+30;
		
	}
		

	
	
	Roll_Out=PID_Control(&Roll_PID,roll_target,roll);
	Limit(Roll_Out,10);

	Angle_Speed_X_Out=PID_Control(&ANGLE_SPEED_X_PID,Roll_Out,angle_speed_X);
	
	
	
	Pitch_Out=PID_Control(&Pitch_PID,pitch_target,pitch);
	Limit(Pitch_Out,10);
	
	Angle_Speed_Y_Out=PID_Control(&ANGLE_SPEED_Y_PID,Pitch_Out,angle_speed_Y);
	Angle_Speed_Z_Out=PID_Control(&ANGLE_SPEED_Z_PID,0,angle_speed_Z);
//	
	


	//串级PID
	CH1_Out=-Angle_Speed_X_Out+Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Velocity_Out;  //以角度向前倾，向左倾为标准
	CH2_Out=-Angle_Speed_X_Out-Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Velocity_Out;
	CH3_Out=Angle_Speed_X_Out-Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Velocity_Out;
	CH4_Out=Angle_Speed_X_Out+Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Velocity_Out;
	
	Set_Motor_Speed(1,CH1_Out);
	Set_Motor_Speed(2,CH2_Out);
	Set_Motor_Speed(3,CH3_Out);
	Set_Motor_Speed(4,CH4_Out);
}

	
#define COMPARE(MAX,B,MIN) if(B>MAX)MAX=B;else if(B<MIN)MIN=B

float Prams[Pram_Size];
/*
	将参数保存至扇区
	保存三轴陀螺仪的灵漂
*/


void Write_Prams(){
	int i;
	u32 temp;
	FLASH_Status FLASHStatus;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP|FLASH_FLAG_PGERR |FLASH_FLAG_WRPRTERR);
	FLASHStatus = FLASH_ErasePage(FLASH_Start); //擦除一页

	
	Prams[0]=pitch_target;
	Prams[1]=roll_target;
	Prams[2]=w_x_offset;
	Prams[3]=w_y_offset;
	Prams[4]=w_z_offset;	
	
	for(i=0;i<Pram_Size;++i){
		temp=*((u32 *)(Prams+i));
		FLASHStatus = FLASH_ProgramWord(FLASH_Start+i*4, temp);
		uprintf(USART,"write %f\r\n",Prams[i]);
	}
	
	FLASH_Lock();
}
u8 Pram_Error=0;
#define Error_Check(a,max_error) if(a>max_error||a<-max_error||isnan(a)){a=0;Pram_Error=1;}

void Load_Prams(){
	int i;
	for(i=0;i<Pram_Size;++i){
		Prams[i]=*(float* )(FLASH_Start+i*4);
		uprintf(USART,"Load Pram:%f\r\n",Prams[i]);
	}

	pitch_target=Prams[0];
	roll_target=Prams[1];
	w_x_offset=Prams[2];
	w_y_offset=Prams[3];
	w_z_offset=Prams[4];	
	
	Error_Check(pitch_target,5);
	Error_Check(roll_target,5);

	Error_Check(a_x_offset,2);
	Error_Check(a_y_offset,2);
	Error_Check(a_z_offset,2);
	
	if(Pram_Error){
		uprintf(USART,"load offset error!please adjust again!\r\n");
	}

}


