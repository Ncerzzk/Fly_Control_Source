#ifndef __BASE_H
#define __BASE_H

#include "stm32f10x.h"
#include "outputdata.h"


void delay_init(int SYSCLK);
void delay_us(long nus);
float avarge(int *data,int count);

typedef struct _kal_struct{
	float A;   //一般为1
	float B;   //一般为0
	float Q;//系统过程噪声的协方差
	float R;//测量噪声的协方差
	
	float kal_out; //上一次卡尔曼的输出
	
	float cov; //上一次卡尔曼的输出的协方差
	
}Kal_Struct;

float KalMan(Kal_Struct *kal,float x);


#endif
