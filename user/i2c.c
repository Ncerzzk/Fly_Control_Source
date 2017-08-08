#include "i2c.h"


/*
	将I2C的两个引脚设为OD模式，手动进行高低电平的更改
*/
#define I2C1_SCL	GPIO_Pin_6      //GPIO_Pin_8 可复用为I2C1
#define I2C1_SDA	GPIO_Pin_7    	//GPIO_Pin_9 

#define I2C2_SCL	GPIO_Pin_10
#define I2C2_SDA	GPIO_Pin_11

void I2C_Pin2OD(I2C_TypeDef * I2Cn){
	GPIO_InitTypeDef GPIO_InitStructure;
	if(I2Cn==I2C2){
		GPIO_InitStructure.GPIO_Pin = I2C2_SCL | I2C2_SDA;	//I2C2
	}else{
		GPIO_InitStructure.GPIO_Pin = I2C1_SCL | I2C1_SDA;   //I2C1
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
	将I2C的两个引脚设为AF_OD模式，由I2C接口接管
*/
void I2C_Pin2AFOD(I2C_TypeDef * I2Cn){
	GPIO_InitTypeDef GPIO_InitStructure;
	if(I2Cn==I2C2){
		GPIO_InitStructure.GPIO_Pin = I2C2_SCL | I2C2_SDA;	//I2C2
	}else{
		GPIO_InitStructure.GPIO_Pin = I2C1_SCL | I2C1_SDA;   //I2C1
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
	I2C2 
	PB10 SCL
	PB11 SDA
	
*/



void I2C_init(I2C_TypeDef * I2Cn){
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	if(I2Cn==I2C2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	}else{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	}
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	/* Configure I2C2 pins: SCL and SDA ----------------------------------------*/
	if(I2Cn==I2C2){
		GPIO_InitStructure.GPIO_Pin = I2C2_SCL | I2C2_SDA;	//I2C2
	}else{
		GPIO_InitStructure.GPIO_Pin = I2C1_SCL | I2C1_SDA;   //I2C1
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure I2C2*/

	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 =0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	
	
	I2C_DeInit(I2Cn);
	I2C_Init(I2Cn, &I2C_InitStructure);
	I2C_Cmd(I2Cn, ENABLE);
	//I2C_CalculatePEC(I2Cn, ENABLE);  
	I2C_ITConfig(I2Cn, I2C_IT_EVT | I2C_IT_BUF, ENABLE);

	if(I2C_GetFlagStatus(I2Cn, I2C_FLAG_BUSY)){
		I2C_Pin2OD(I2Cn);
		I2C_ForceEnd(I2Cn);
		
	}
	I2C_Pin2AFOD(I2Cn);

}


void I2C_ForceEnd(I2C_TypeDef * I2Cn){
	u16 SDA_Pin;
	u16 SCL_Pin;
	if(I2Cn==I2C2){
		SDA_Pin=I2C2_SDA;
		SCL_Pin=I2C2_SCL;
	}else{
		SDA_Pin=I2C1_SDA;
		SCL_Pin=I2C1_SCL;		
	}
	while(!(GPIOB->ODR&SDA_Pin)){
		GPIO_SetBits(GPIOB,SCL_Pin);
		delay_us(10);
		GPIO_ResetBits(GPIOB,SCL_Pin);
		delay_us(10);
	}
	GPIO_SetBits(GPIOB,SCL_Pin);
	GPIO_ResetBits(GPIOB,SDA_Pin);
	delay_us(10);
	GPIO_SetBits(GPIOB,SDA_Pin);
	delay_us(10);
	I2C_SoftwareResetCmd(I2Cn,ENABLE);

}

void I2C_Test(){
	int i=50000;
	I2C_GenerateSTART(I2C_USE, ENABLE);
  while(!I2C_CheckEvent(I2C_USE, I2C_EVENT_MASTER_MODE_SELECT)){
		if(!--i)
			return ;
	}
	I2C_Send7bitAddress(I2C_USE, 0xA0, I2C_Direction_Transmitter);          // 发送 丛机地址 地址、状态（写）
	
	i=50000;
	while(!I2C_CheckEvent(I2C_USE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(!--i)
			return ;		
	}
	
	I2C_SendData(I2C_USE, 0x05);
	i=50000;
	while(!I2C_CheckEvent(I2C_USE, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(!--i)
			return ;		
	}
	I2C_GenerateSTOP(I2C_USE, ENABLE);          //发送结束信号
	
}
void I2C_ByteWrite(I2C_TypeDef * I2Cn,u8 slaveAddr, u8 pBuffer, u8 writeAddr)
{
	
  /* Send START condition */
  I2C_GenerateSTART(I2Cn, ENABLE);          //发送开始信号
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(I2Cn, slaveAddr, I2C_Direction_Transmitter);          // 发送 丛机地址 地址、状态（写）
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the MPU6050's internal address to write to */
  I2C_SendData(I2Cn, writeAddr);                   //发送 从机内部某个待写寄存器地址
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2Cn, pBuffer);                     //发送要写入的内容
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STOP condition */
  I2C_GenerateSTOP(I2Cn, ENABLE);          //发送结束信号
}

void I2C_BufferRead(I2C_TypeDef * I2Cn,u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
  /* While the bus is busy */  
  while(I2C_GetFlagStatus(I2Cn, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2Cn, ENABLE);
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for write */
  I2C_Send7bitAddress(I2Cn, slaveAddr, I2C_Direction_Transmitter); 
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2Cn, ENABLE);
  /* Send the MPU6050's internal address to write to */
  I2C_SendData(I2Cn, readAddr);
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

/* Send STRAT condition a second time */
  I2C_GenerateSTART(I2Cn, ENABLE);
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send MPU6050 address for read */
  I2C_Send7bitAddress(I2Cn, slaveAddr, I2C_Direction_Receiver);
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2Cn, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2Cn, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2Cn, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the MPU6050 */
      *pBuffer = I2C_ReceiveData(I2Cn);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2Cn, ENABLE);
}

u8 I2C_ByteRead(I2C_TypeDef * I2Cn,u8 slaveAddr,u8 readAddr){
	u8 temp;
	I2C_BufferRead(I2Cn,slaveAddr,&temp,readAddr,1);
	return temp;	
}

s16 I2C_DoubleRead(I2C_TypeDef * I2Cn,u8 slaveAddr,u8 readAddr,dataType t){
	u8 temp[2];
	I2C_BufferRead(I2Cn,slaveAddr,temp,readAddr,2);
	if(t==MSB_First)    //MPU6050
		return (temp[0]<<8)+temp[1];
	else			 //LSB_First MAG
	{
		return (temp[1]<<8)|temp[0]; 
	}
}

