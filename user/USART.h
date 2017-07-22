#ifndef __USART_H
#define __USART_H

#include <stdio.h>
#include <stdarg.h>
#include "stm32f10x.h"
#define USART 	UART4   //当前程序使用的串口

void Usart_Init(USART_TypeDef * USARTx,int BaudRate);
void uprintf(USART_TypeDef* USARTx, char *fmt, ...);

#endif
