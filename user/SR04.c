#include "SR04.h"
#include "base.h"

float height;
float SR04_V;

Kal_Struct kal_height={1,0,1,20,0,1};
Kal_Struct kal_V={1,0,0.5,100,0,1};

float cap_time;
int cap_time_data[CAP_BUFFER_SIZE];
int cap_count;
void SR04_Init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;   //PA4 Trig1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

void SR04_Trig(){
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	delay_us(70);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}


/*
本函数已经废弃，迁移至main.c中的TIM3中断中。
*/
void Get_Height(){
	//height=avarge(cap_time_data,CAP_BUFFER_SIZE)*0.5*34000/1000000/2;   //单位为cm;
	float temp;
	temp=cap_time*0.5*34000/1000000/2; 
	height=temp;
//	if(temp<300){
//		height=temp;
//	}
	//height=KalMan(&kal_height,height);
	
}

