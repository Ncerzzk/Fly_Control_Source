#ifndef __SR04_H
#define __SR04_H
#include "stm32f10x.h"
#include "base.h"
extern float height;
void Get_Height(void);
extern float cap_time;
void SR04_Init(void);
#define CAP_BUFFER_SIZE 10
extern int cap_time_data[CAP_BUFFER_SIZE];
extern int cap_count;

extern Kal_Struct kal_height;
extern float SR04_V;
extern Kal_Struct kal_V;

#endif
