#ifndef __SR04_H
#define __SR04_H
#include "stm32f10x.h"
extern float height;
void Get_Height(void);
extern int cap_time;
void SR04_Init(void);
#define CAP_BUFFER_SIZE 10
extern int cap_time_data[CAP_BUFFER_SIZE];
extern int cap_count;
extern int cap_time;


#endif
