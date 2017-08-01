#include "SR04.h"
#include "base.h"

float height;

Kal_Struct kal_height={1,0,0.01,37.1160,0,1};

int cap_time;
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
	delay_us(15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}


void Get_Height(){
	//height=avarge(cap_time_data,CAP_BUFFER_SIZE)*0.5*34000/1000000/2;   //单位为cm;
	float temp;
	temp=cap_time*0.5*34000/1000000/2; 
	if(temp<300){
		height=temp;
	}
	height=KalMan(&kal_height,height);
	
}