#ifndef __USART_H
#define __USART_H

#include <stdio.h>
#include <stdarg.h>
#include "stm32f10x.h"
#define USART 	UART4   //��ǰ����ʹ�õĴ���

void Usart_Init(USART_TypeDef * USARTx,int BaudRate);
void uprintf(USART_TypeDef* USARTx, char *fmt, ...);

#endif
