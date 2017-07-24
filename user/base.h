#ifndef __BASE_H
#define __BASE_H

#include "stm32f10x.h"
#include "outputdata.h"

void delay_init(int SYSCLK);
void delay_us(long nus);
float avarge(int *data,int count);
#endif
