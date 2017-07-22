#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f10x.h"

#define Set_Brush_Speed(CH,duty) TIM_SetCompare##CH(TIM2,(duty>96?96:duty)*4095/100)

void Brush_Init();


#endif