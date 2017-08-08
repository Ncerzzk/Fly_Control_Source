#include "spi.h"
#include "USART.h"


 
void SPI2_Config(void)
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  
	SPI_InitTypeDef SPI_InitStructure;    
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);               //ʹ��SPI1ʱ��

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
     //  PB15--CLK PB13--MOSI  PB12 �������� 
  
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
	
	//IRQ �ж� C7
	
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
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; //PPP�ⲿ�ж���
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
     //PA6--MISO  ���븡��
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
   SPI_NRF_CSN_HIGH();//����CSNʧ��Ƭѡ
    
   
                                  //����������ʼ���Ľṹ��
   SPI_InitStructure.SPI_Direction =SPI_Direction_2Lines_FullDuplex;//ȫ˫��
	 SPI_InitStructure.SPI_Mode =SPI_Mode_Master;                    //��ģʽ
	 SPI_InitStructure.SPI_DataSize =SPI_DataSize_8b;                 //һ�δ���8λ
	 SPI_InitStructure.SPI_CPOL =SPI_CPOL_Low;                       //���е�ƽ�͵�ƽ
	 SPI_InitStructure.SPI_CPHA =SPI_CPHA_1Edge;                     //��һ�������ز���
	 SPI_InitStructure.SPI_NSS =SPI_NSS_Soft;                        //NSS����Ϊ�����ģʽ
	 SPI_InitStructure.SPI_BaudRatePrescaler =SPI_BaudRatePrescaler_8;//������Ԥ��Ƶ8  9MHz
	 SPI_InitStructure.SPI_FirstBit =SPI_FirstBit_MSB;      //���ݴ����λ��ǰ
	 SPI_InitStructure.SPI_CRCPolynomial =7;                          //CRCУ�鷽ʽ
   SPI_Init(SPI2,&SPI_InitStructure);                                 //��ʼ��
    
   //SPI_NSSInternalSoftwareConfig(SPI1,SPI_NSSInternalSoft_Set);
    
  
   SPI_Cmd(SPI2, ENABLE); //ʹ��SPI1
}

u8 SPI_RW_Byte(SPI_TypeDef* SPIx,unsigned char Byte)
{
     while( SPI_I2S_GetFlagStatus(SPIx,SPI_I2S_FLAG_TXE) == RESET); //�鷢�ͻ������Ƿ�Ϊ�գ��ռ����Է���
     SPI_I2S_SendData(SPIx,Byte);   //�⺯��������һ���ֽ�
     //��SPI���ջ�����Ϊ��ʱ�ȴ�
      while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) ==RESET);
		return SPI_I2S_ReceiveData(SPIx);
}//SPI_RW_Byte()

u8 SPI_NRF_Write(SPI_TypeDef* SPIx,char CMD,unsigned char*WBuff,unsigned char ByteNUM)
{
	int i;
	unsigned chari,status; 
	SPI_CE_LOW();
	SPI_NRF_CSN_LOW();//ʹ��Ƭѡ

	status=SPI_RW_Byte( SPIx , CMD);
	for(i=0;i<ByteNUM;++i)
	{
			SPI_RW_Byte( SPIx,*WBuff++);
		 //printf("д���%d������\r\n",ByteNUM);   
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
			RBuff[i]=SPI_RW_Byte(SPIx,NOP);         // ȡ���ջ�������һ���ֽ�
		 //printf("������%d������\r\n",ByteNUM);   
	}
	SPI_NRF_CSN_HIGH(); 
	return status; 
}

void SPI_NRF_MOD_TX(void)
{
    u8 TX_Array[5];
	u8 _TX_RX_ADDR_[5]={0xB3,0xB4,0xB5,0xB6,0x05};
		
	SPI_CE_LOW();//CE=0����ģʽ 
		TX_Array[0]=0x03;//���õ�ַ���11--5�ֽ� 10--4�ֽ� 01-3�ֽ� 00--���Ϸ�
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +SETUP_AW,TX_Array,1); 
		TX_Array[0]=0xf3;//�����Զ��ط������1111��--�ȴ�4000+86us  15��
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +SETUP_RETR,TX_Array,1); 
		TX_Array[0]=0x02;//��Ƶͨ�� X0000010
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_CH,TX_Array,1);
		TX_Array[0]=0x0f;//��Ƶ�����Ĵ��� 00001111 2Mbps ���书�� 00-18dBm 01-12dBm 10-6dBm 11-0dBm 1--�������Ŵ�������
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_SETUP,TX_Array,1);
		TX_Array[0]=0x3f;//xx11 11110-5����ͨ������ 
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_RXADDR,TX_Array,1);
		TX_Array[0]=0x3f;//xx11 11110-5ͨ�������Զ�Ӧ��
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_AA,TX_Array,1); 
		 

	SPI_NRF_Write(SPI2,NRF_WRITE_REG +TX_ADDR,_TX_RX_ADDR_,5);//д����շ������ݵĵ�ַ�������ַ�ǽ��ն��ռ���ƾ֤
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RX_ADDR_P0,_TX_RX_ADDR_,5);//д����շ������ݵĵ�ַ�������ַ�ǽ��ն��ռ���ƾ֤


	TX_Array[0]=0x0e;//�ж�ȫ�� ����ģʽ PRIM_RX=0PWR_UP=1 
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +CONFIG,TX_Array,1);

	TX_Array[0]=0xfe;//1111 xxxx STATUS�Ĵ���д��1��������б�־
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +STATUS,TX_Array,1);

	SPI_CE_HIGH();//CE=1ʹ�ܷ���ģʽ 
	delay_us(100);//CE������Ҫһ������ʱ���ܽ��з��� ��ʱ֮�󼴿�ͨ��SPI�ӿڷ���TX_PLD
}

void SPI_NRF_MOD_RX(void)
{
  u8 TX_Array[5];
	u8 _TX_RX_ADDR_[5]={0xB3,0xB4,0xB5,0xB6,0x05};

	SPI_CE_LOW();//CE=0����ģʽ 
		TX_Array[0]=0x03;//�������ͨ��00000011
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_RXADDR,TX_Array,1);
		TX_Array[0]=0x03;//���õ�ַ���11--5�ֽ� 10--4�ֽ� 01-3�ֽ� 00--���Ϸ�
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +SETUP_AW,TX_Array,1); 
		TX_Array[0]=0x20;//��Ƶͨ�� X0000010
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_CH,TX_Array,1);
		TX_Array[0]=0x0f;//��Ƶ�����Ĵ��� 00001111 2Mbps ���书�� 00-18dBm 01-12dBm 10-6dBm 11-0dBm 1--�������Ŵ�������
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RF_SETUP,TX_Array,1);
		TX_Array[0]=0x3f;//xx11 11110-5ͨ�������Զ�Ӧ��
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +EN_AA,TX_Array,1); 
		TX_Array[0]=0x04;//xx11 1111����ͨ��0 ��Ч���ݿ�� ��1-32���ֽ�
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RX_PW_P0,TX_Array,1);
		TX_Array[0]=0xfe;//1111 xxxxSTATUS�Ĵ��� д��1��������б�־
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +STATUS,TX_Array,1);

	SPI_NRF_Write(SPI2,NRF_WRITE_REG +TX_ADDR,_TX_RX_ADDR_,5);//д����շ������ݵĵ�ַ�������ַ�ǽ��ն��ռ���ƾ֤
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +RX_ADDR_P0,_TX_RX_ADDR_,5);//д����շ������ݵĵ�ַ�������ַ�ǽ��ն��ռ���ƾ֤



	TX_Array[0]=0x0f;//����ģʽ PRIM_RX=1 PWR_UP=1��������ն�
	SPI_NRF_Write(SPI2,NRF_WRITE_REG +CONFIG,TX_Array,1);

	SPI_CE_HIGH();//CE=1ʹ�ܷ���ģʽ 
	delay_us(100);//CE������Ҫһ������ʱ���ܽ��з��� ��ʱ֮�󼴿�ͨ��SPI�ӿڷ���TX_PLD
	//��ѯ�ж�24L01�жϵĵ��� NRF_Read_IRQ()
}


u8 SPI_NRF_TX_DATAS(u8* TBuff,u8 ByteNUM)
{ 
   u8 Status[1];
	do{
		 SPI_CE_LOW();//���ʹ���
		 SPI_NRF_Write(SPI2,WR_TX_PLOAD,TBuff,ByteNUM);//����TBuff����
		 SPI_CE_HIGH();//���ʹ���
		}while(NRF_Read_IRQ()!=0);//�жϲ���ʱ��IRQ���ŵ͵�ƽ

	SPI_NRF_Write(SPI2, FLUSH_TX,TBuff,0);
	SPI_CE_LOW();//���ʹ���
	delay_us(100);
	SPI_NRF_Read(SPI2,NRF_READ_REG+STATUS,Status,1);//��ȡStatus
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
	while(NRF_Read_IRQ()!=0);//�жϲ���ʱ��IRQ���ŵ͵�ƽ
	SPI_CE_LOW();//���ʹ��������ܲ����Ĵ���
	delay_us(100);
	SPI_NRF_Read(SPI2,NRF_READ_REG+STATUS,Status,1);//��ȡStatus
	switch(Status[0]&0x0e) 
	{
		case 0x0e: 
			uprintf(USART,"Error\r\n");
			RX_Status=0;break; //RX_FIFO ��
		default :
			uprintf(USART,"OK");
			break;

	}

	SPI_NRF_Read(SPI2,RD_RX_PLOAD,RBuff,4);//��RX_FIFO
	SPI_NRF_Write(SPI2,NRF_READ_REG+STATUS,Status,1);//����״̬�Ĵ�����־
	return RX_Status;
}

