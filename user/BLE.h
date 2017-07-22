#ifndef __BLE_H
#define __BLE_H
#include "stm32f10x.h"

void BLE_Init();
void BLE_Wake();
void BLE_Send_Start();
void BLE_Send_Over();
void BLE_CMD_Open();
void BLE_CMD_Close();


#endif