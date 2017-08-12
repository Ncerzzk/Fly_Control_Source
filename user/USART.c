#include "USART.h"


void Usart_Init(USART_TypeDef * USARTx,int BaudRate){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//NVIC_InitStructure.NVIC_IRQChannel = USARTy_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);
	
	if(USARTx==USART1){
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		

			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
			GPIO_Init(GPIOA,&GPIO_InitStructure);
			

			GPIO_InitStructure.GPIO_Pin= GPIO_Pin_10;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
			GPIO_Init(GPIOA,&GPIO_InitStructure);
			

			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_Init(&NVIC_InitStructure);
	}else if(USARTx==UART4){
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4 , ENABLE);
		
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
			GPIO_Init(GPIOC,&GPIO_InitStructure);
			

			GPIO_InitStructure.GPIO_Pin= GPIO_Pin_11;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
			GPIO_Init(GPIOC,&GPIO_InitStructure);
			

			NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
			NVIC_Init(&NVIC_InitStructure);
	}

	USART_Init(USARTx,&USART_InitStructure);
	
	USART_Cmd(USARTx,ENABLE);
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); 
//	USART_ITConfig(USARTx, USART_IT_ERR, ENABLE); 
}


void uprintf(USART_TypeDef* USARTx, char *fmt, ...)
{

	char buffer[100 + 1];
	u8 i = 0;
	
	va_list arg_ptr;
	
	va_start(arg_ptr, fmt);  
	
	vsnprintf(buffer, 100 + 1, fmt, arg_ptr);
	
	USART_ClearFlag(USARTx, USART_FLAG_TXE);
	
	while ((i < 100) && buffer[i])
	{
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	  USART_SendData(USARTx, (u8)buffer[i++]); 
	}
	
	va_end(arg_ptr);
	
}