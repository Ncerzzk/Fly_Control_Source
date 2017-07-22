#include "base.h"


/*
	延时初始化。SYSCLK=72一般。
*/

int fac_us;

void delay_init(int SYSCLK)
{  
    SysTick->CTRL&=0xFFFFFFFB;  
    fac_us=SYSCLK/8;  
    //fac_ms=(u16t)(fac_us*1000);       
}  
  

/*1us初始化*/

void delay_us(long nus)  
{  
    long temp;  
    SysTick->LOAD=nus*fac_us;  
    SysTick->VAL=0x00;  
    SysTick->CTRL=0x01;  
    do  
    {  
        temp=SysTick->CTRL;  
    }  
    while(temp&0x01&&!(temp&0x10000));  
    SysTick->CTRL=0x00;  
    SysTick->VAL=0x00;     
}  

void send_wave(int data1,int data2,int data3,int data4){
	OutData[0]=data1;
	OutData[1]=data2;
	OutData[2]=data3;
	OutData[3]=data4;
	OutPut_Data();
}

void send_wave_shaobo(float data1,float data2,float data3,float data4){
	;
}