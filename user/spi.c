#include "spi.h"
#include "USART.h"


 
void SPI2_Config(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  
	SPI_InitTypeDef SPI_InitStructure;    
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);               //使能SPI1时钟

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
     //  PB15--CLK PB13--MOSI  PB12 复用推挽 
  
   GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15|GPIO_Pin_13; 
   GPIO_InitStructure.GPIO_Speed =GPIO_Speed_10MHz; 
   GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP; 
   GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//B12 NSS
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP;	
	 GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	 GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	//CE
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	//IRQ 中断 C7
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
//	
//	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);
//	

//	
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	 
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //PPP外部中断线
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
     //PA6--MISO  输入浮空
   GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14; 
   GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING; 
   GPIO_Init(GPIOB, &GPIO_InitStructure);
   
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2; 
	 GPIO_InitStructure.GPIO_Speed =GPIO_Speed_10MHz; 
	 GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; 
   GPIO_Init(GPIOA, &GPIO_InitStructure);


   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  
   GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU; 
   GPIO_Init(GPIOA, &GPIO_InitStructure);

	 SPI_CE_LOW();
   SPI_NRF_CSN_HIGH();//拉高CSN失能片选
    
   
                                  //声明用来初始化的结构体
   SPI_InitStructure.SPI_Direction =SPI_Direction_2Lines_FullDuplex;//全双工
	 SPI_InitStructure.SPI_Mode =SPI_Mode_Master;                    //主模式
	 SPI_InitStructure.SPI_DataSize =SPI_DataSize_8b;                 //一次传输8位
	 SPI_InitStructure.SPI_CPOL =SPI_CPOL_Low;                       //空闲电平低电平
	 SPI_InitStructure.SPI_CPHA =SPI_CPHA_1Edge;                     //第一个上升沿采样
	 SPI_InitStructure.SPI_NSS =SPI_NSS_Soft;                        //NSS管理为软件件模式
	 SPI_InitStructure.SPI_BaudRatePrescaler =SPI_BaudRatePrescaler_8;//波特率预分频8  9MHz
	 SPI_InitStructure.SPI_FirstBit =SPI_FirstBit_MSB;      //数据传输低位在前
	 SPI_InitStructure.SPI_CRCPolynomial =7;                          //CRC校验方式
   SPI_Init(SPI2,&SPI_InitStructure);                                 //初始化
    
   //SPI_NSSInternalSoftwareConfig(SPI1,SPI_NSSInternalSoft_Set);
    
  
   SPI_Cmd(SPI2, ENABLE); //使能SPI1
}

u8 SPI_RW_Byte(SPI_TypeDef* SPIx,unsigned char Byte)
{
     while( SPI_I2S_GetFlagStatus(SPIx,SPI_I2S_FLAG_TXE) == RESET); //查发送缓冲器是否为空，空即可以发送
     SPI_I2S_SendData(SPIx,Byte);   //库函数：发送一个字节
     //当SPI接收缓冲器为空时等待
      while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) ==RESET);
		return SPI_I2S_ReceiveData(SPIx);
}//SPI_RW_Byte()

u8 SPI_NRF_Write(SPI_TypeDef* SPIx,char CMD,unsigned char*WBuff,unsigned char ByteNUM)
{
	int i;
	unsigned chari,status; 
	SPI_CE_LOW();
	SPI_NRF_CSN_LOW();//使能片选

	status=SPI_RW_Byte( SPIx , CMD);
	for(i=0;i<ByteNUM;++i)
	{
			SPI_RW_Byte( SPIx,*WBuff++);
		 //printf("写入第%d个数据\r\n",ByteNUM);   
	}
	SPI_NRF_CSN_HIGH();//
	return status;
}//SPI_NRF_Write()

u8 SPI_NRF_Read(SPI_TypeDef* SPIx,char CMD,unsigned char*RBuff,unsigned char ByteNUM)
{
	unsigned char i,status ;
		
	SPI_CE_LOW();
	SPI_NRF_CSN_LOW(); 
	status=SPI_RW_Byte( SPIx , CMD);
	for(i=0;i< ByteNUM ;i++)
	{   
			RBuff[i]=SPI_RW_Byte(SPIx,NOP);         // 取接收缓冲器，一个字节
		 //printf("读出第%d个数据\r\n",ByteNUM);   
	}
	SPI_NRF_CSN_HIGH(); 
	return status; 
}

void SPI_NRF_MOD_TX(void)
{
    u8 TX_Array[5];
	u8 _TX_RX_ADDR_[5]={0xB3,0xB4,0xB5,0xB6,0x05};
		
	SPI_CE_LOW();//CE=0待机模式 
		TX_Array[0]=0x03;//设置地址宽度11--5字节 10--4字节 01-3字节 00--不合法
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +SETUP_AW,TX_Array,1); 
		TX_Array[0]=0xf3;//建立自动重发间隔‘1111‘--等待4000+86us  15次
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +SETUP_RETR,TX_Array,1); 
		TX_Array[0]=0x02;//射频通道 X0000010
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_CH,TX_Array,1);
		TX_Array[0]=0x0f;//射频参数寄存器 00001111 2Mbps 发射功率 00-18dBm 01-12dBm 10-6dBm 11-0dBm 1--低噪声放大器增益
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_SETUP,TX_Array,1);
		TX_Array[0]=0x3f;//xx11 11110-5接收通道允许 
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_RXADDR,TX_Array,1);
		TX_Array[0]=0x3f;//xx11 11110-5通道允许自动应答
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_AA,TX_Array,1); 
		 

	SPI_NRF_Write(SPI2,NRF_WRITE_REG +TX_ADDR,_TX_RX_ADDR_,5);//写入接收发送数据的地址，这个地址是接收端收件的凭证
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RX_ADDR_P0,_TX_RX_ADDR_,5);//写入接收发送数据的地址，这个地址是接收端收件的凭证


	TX_Array[0]=0x0e;//中断全开 发送模式 PRIM_RX=0PWR_UP=1 
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +CONFIG,TX_Array,1);

	TX_Array[0]=0xfe;//1111 xxxx STATUS寄存器写‘1’清除所有标志
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +STATUS,TX_Array,1);

	SPI_CE_HIGH();//CE=1使能发射模式 
	delay_us(100);//CE拉高需要一定的延时才能进行发送 延时之后即可通过SPI接口发送TX_PLD
}

void SPI_NRF_MOD_RX(void)
{
  u8 TX_Array[5];
	u8 _TX_RX_ADDR_[5]={0xB3,0xB4,0xB5,0xB6,0x05};

	SPI_CE_LOW();//CE=0待机模式 
		TX_Array[0]=0x03;//允许接收通道00000011
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_RXADDR,TX_Array,1);
		TX_Array[0]=0x03;//设置地址宽度11--5字节 10--4字节 01-3字节 00--不合法
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +SETUP_AW,TX_Array,1); 
		TX_Array[0]=0x20;//射频通道 X0000010
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_CH,TX_Array,1);
		TX_Array[0]=0x0f;//射频参数寄存器 00001111 2Mbps 发射功率 00-18dBm 01-12dBm 10-6dBm 11-0dBm 1--低噪声放大器增益
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_SETUP,TX_Array,1);
		TX_Array[0]=0x3f;//xx11 11110-5通道允许自动应答
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_AA,TX_Array,1); 
		TX_Array[0]=0x04;//xx11 1111数据通道0 有效数据宽度 （1-32）字节
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RX_PW_P0,TX_Array,1);
		TX_Array[0]=0xfe;//1111 xxxxSTATUS寄存器 写‘1’清除所有标志
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +STATUS,TX_Array,1);

	SPI_NRF_Write(SPI2,NRF_WRITE_REG +TX_ADDR,_TX_RX_ADDR_,5);//写入接收发送数据的地址，这个地址是接收端收件的凭证
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RX_ADDR_P0,_TX_RX_ADDR_,5);//写入接收发送数据的地址，这个地址是接收端收件的凭证



	TX_Array[0]=0x0f;//接收模式 PRIM_RX=1 PWR_UP=1允许接收终端
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +CONFIG,TX_Array,1);

	SPI_CE_HIGH();//CE=1使能发射模式 
	delay_us(100);//CE拉高需要一定的延时才能进行发送 延时之后即可通过SPI接口发送TX_PLD
	//轮询中断24L01中断的到来 NRF_Read_IRQ()
}


u8 SPI_NRF_TX_DATAS(u8* TBuff,u8 ByteNUM)
{ 
   u8 Status[1];
	do{
		 SPI_CE_LOW();//拉低待机
		 SPI_NRF_Write(SPI2,WR_TX_PLOAD,TBuff,ByteNUM);//发送TBuff数组
		 SPI_CE_HIGH();//拉低待机
		}while(NRF_Read_IRQ()!=0);//中断产生时，IRQ引脚低电平

	SPI_NRF_Write(SPI2, FLUSH_TX,TBuff,0);
	SPI_CE_LOW();//拉低待机
	delay_us(100);
	SPI_NRF_Read(SPI2,NRF_READ_REG+STATUS,Status,1);//读取Status
	if(Status[0]&0x10)
	{
		Status[0]&=0x10;
		SPI_NRF_Write(SPI2,NRF_WRITE_REG +STATUS,Status,1);// 
		return 0;
	}
	else
	{
		Status[0]&=0x20;
		SPI_NRF_Write(SPI2,NRF_WRITE_REG +STATUS,Status,1);// 
		return 1;
	}
}

char RBuff[10];

u8  SPI_NRF_RX_DATAS(u8* RBuff)
{
   u8 RX_Status=1;
   u8 Status[1];
	while(NRF_Read_IRQ()!=0);//中断产生时，IRQ引脚低电平
	SPI_CE_LOW();//拉低待机，才能操作寄存器
	delay_us(100);
	SPI_NRF_Read(SPI2,NRF_READ_REG+STATUS,Status,1);//读取Status
	switch(Status[0]&0x0e) 
	{
		case 0x0e: 
			uprintf(USART,"Error\r\n");
			RX_Status=0;break; //RX_FIFO 空
		default :
			uprintf(USART,"OK");
			break;

	}

	SPI_NRF_Read(SPI2,RD_RX_PLOAD,RBuff,4);//读RX_FIFO
	SPI_NRF_Write(SPI2,NRF_READ_REG+STATUS,Status,1);//处理状态寄存器标志
	return RX_Status;
}

