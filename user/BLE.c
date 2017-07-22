#include "BLE.h"

#define SLEEP_Pin		GPIOC,GPIO_Pin_7
#define RST_Pin			GPIOC,GPIO_Pin_6
#define RTS_Pin			GPIOB,GPIO_Pin_15
#define CMD_Pin			GPIOB,GPIO_Pin_14
#define CTS_Pin			GPIOB,GPIO_Pin_13
#define CONN_Pin		GPIOB,GPIO_Pin_12

void BLE_Init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;	//SLEEP_PIN
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;		//RTS_PIN  //CMD_Pin
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;		//CTS_PIN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
}

void BLE_Wake(){
	GPIO_SetBits(SLEEP_Pin);
	delay_us(1000);
	GPIO_ResetBits(SLEEP_Pin);
}

void BLE_Send_Start(){
	GPIO_ResetBits(RTS_Pin);
}

void BLE_Send_Over(){
	GPIO_SetBits(RTS_Pin);
}

void BLE_CMD_Open(){
	GPIO_ResetBits(CMD_Pin);
}

void BLE_CMD_Close(){
	GPIO_SetBits(CMD_Pin);
}