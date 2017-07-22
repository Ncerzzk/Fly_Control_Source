#include "Hspi.h"

void HSPI_Init(){
	 GPIO_InitTypeDef GPIO_InitStructure;
	 SPI_InitTypeDef   SPI_InitStructure;

   /* Configure SPI2 pins: SCK, MISO and MOSI ---------------------------------*/
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
   SPI_InitStructure.SPI_CRCPolynomial = 7;
   SPI_Init(SPI2, &SPI_InitStructure);
	 
	 SPI_Cmd(SPI2, ENABLE);
 
}



u8 HSPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte)
{
while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET){}//等待发送区空 
SPI_I2S_SendData(SPIx, Byte); //通过外设SPIx发送一个byte  数据
while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET){} //等待接收完
return SPI_I2S_ReceiveData(SPIx); //返回通过SPIx最近接收的数据   
}

void HSPI_WriteWord(SPI_TypeDef * SPIx,u8 addr,u8 H,u8 L){
	HSPI_WriteByte(SPIx,addr);
	HSPI_WriteByte(SPIx,H);
	HSPI_WriteByte(SPIx,L);
}


void HSPI_ReadWord(SPI_TypeDef * SPIx,u8 addr,u8 *a){
	HSPI_WriteByte(SPIx,addr);
	a[0]=HSPI_WriteByte(SPIx,0);
	a[1]=HSPI_WriteByte(SPIx,0);
}