#ifndef __I2C_H
#define __I2C_H

#include "base.h"

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"


typedef enum dataType{
	MSB_First=1,    //高位在前
	LSB_First=2			//低位在前
}dataType;



void I2C_init(I2C_TypeDef * I2Cn);
void I2C_ByteWrite(I2C_TypeDef * I2Cn,u8 slaveAddr, u8 pBuffer, u8 writeAddr);
void I2C_BufferRead(I2C_TypeDef * I2Cn,u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead);
u8 I2C_ByteRead(I2C_TypeDef * I2Cn,u8 slaveAddr,u8 readAddr);
s16 I2C_DoubleRead(I2C_TypeDef * I2Cn,u8 slaveAddr,u8 readAddr,dataType t);
void I2C_ForceEnd(I2C_TypeDef * I2Cn);


#endif