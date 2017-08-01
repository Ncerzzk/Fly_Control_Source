#ifndef __HSPI_H
#define __HSPI_H

#include "stm32f10x.h"

#define SPI_ReadByte(SPIx) SPI_WriteByte(SPIx,0)
void HSPI_Init(void);

u8 HSPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte);
void HSPI_WriteWord(SPI_TypeDef * SPIx,u8 addr,u8 H,u8 L);

#define HSPI_ReadByte(SPIx) HSPI_WriteByte(SPIx,0)

#define SPI2_ReadByte(addr) HSPI_WriteByte(SPI2,addr)


#define SPI2_WriteWord(addr,H,L)	HSPI_WriteWord(SPI2,addr,H,L)

#define SPI2_WriteByte(B)	HSPI_WriteByte(SPI2,B)


#endif