


#ifndef _outputdata_H
#define _outputdata_H
#include "stm32f10x.h"

extern int OutData[4];
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);

void OutPut_Data(void);

#define UART_PORT 	UART4

#endif 
