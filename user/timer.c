#include "timer.h"
#include "set.h"

void Tim2_Init(){
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* GPIOC clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* GPIOC Configuration:Pin6, 7, 8 and 9 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 2000;
	TIM_TimeBaseStructure.TIM_Prescaler = 35;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);

}

/*
	基本定时器的初始化
*/

void TIMER_Init(TIM_TypeDef *TIMER){
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	if(TIMER!=TIM6 && TIMER!=TIM7)
		return ;
	if(TIMER==TIM6){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
	}else if(TIMER==TIM7){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;		
	}
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 2000;
	TIM_TimeBaseStructure.TIM_Prescaler = 35;      //36分频，分完频率为2M HZ   0.5us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER, &TIM_TimeBaseStructure);


	TIM_ITConfig(TIMER, TIM_IT_Update, ENABLE);
	/* TIM2 enable counter */
	TIM_Cmd(TIMER, ENABLE);	
}


/*TIM3 超声波
*/

void TIM3_Cap_Init()  
{      
    TIM_ICInitTypeDef  TIM_ICInitStructure;  
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
    NVIC_InitTypeDef NVIC_InitStructure;  
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    //使能TIM5时钟  
      
    //TIM3的配置  
   // TIM_TimeBaseStructure.TIM_Period = 2000;     //重装载值  
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler =71;   //分频系数   //72分频，分完频率为1M HZ   
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;      
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数  
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);   
    
    //TIM3输入捕获配置  
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //我们用通道1  
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;    //上升沿捕获  
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1  
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   //无预分频器   
    TIM_ICInitStructure.TIM_ICFilter = 0x00; //IC1F=0000 ，无滤波器  
    TIM_ICInit(TIM3, &TIM_ICInitStructure);  
      
    //中断优先级配置  
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;    
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;   
    NVIC_Init(&NVIC_InitStructure);    
      
    TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC1,ENABLE);//打开更新中断和捕获中断  
      
    TIM_Cmd(TIM3,ENABLE );  //使能定时器3
     
}  

