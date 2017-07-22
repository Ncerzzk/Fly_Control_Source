#ifndef __LT8910_H
#define __LT8910_H
#include "stm32f10x.h"

#define RX_LENGTH 20




#define SPI_SS_GPIO				GPIO_Pin_12
#define SPI_SS						GPIOB,SPI_SS_GPIO

#define SPI_CLK_GPIO 			GPIO_Pin_13
#define SPI_CLK						GPIOB,SPI_CLK_GPIO

#define SPI_RST_GPIO			GPIO_Pin_6
#define SPI_RST						GPIOC,SPI_RST_GPIO

#define SPI_MOSI_GPIO			GPIO_Pin_15
#define SPI_MOSI					GPIOB,SPI_MOSI_GPIO

#define SPI_MISO_GPIO			GPIO_Pin_14
#define SPI_MISO					GPIOB,SPI_MISO_GPIO

#define PKT_GPIO					GPIO_Pin_15
#define PKT								GPIOA,PKT_GPIO


#define Set(n)		GPIO_SetBits(n)
#define	Reset(n)	GPIO_ResetBits(n)

#define Read(n)		GPIO_ReadInputDataBit(n)


void SPI_GPIOInit();
void SPI_WriteWord(u8 addr,u8 H,u8 L);
void SPI_ReadReg(u8 addr);
void LT8910_Init();
u8 RX_model();
void SPI_WriteByte(u8 add,u8 H);
void LT8910_Send(u8 * msg,u8 num);
void LT8910_Get();

extern u8 RegL,RegH;
extern u8 tx_buf[20];
extern u8 rx_buf[RX_LENGTH];

#endif