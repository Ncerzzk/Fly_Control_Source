#include "lt8910.h"
#include "base.h"
#include "Hspi.h"


u8 RegL,RegH;
u8 tx_buf[20];
u8 rx_buf[RX_LENGTH];

#define delay  delay_us(1)
		
void SPI_GPIOInit(){
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = SPI_SS_GPIO|SPI_CLK_GPIO|SPI_MOSI_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =	SPI_MISO_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =	PKT_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =	SPI_RST_GPIO;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void RST_CS_GPIO_Init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = SPI_RST_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = SPI_SS_GPIO;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void SPI_WriteWord(u8 addr,u8 H,u8 L)
{
    int i;
		Reset(SPI_SS);
		delay;
    for(i = 0; i < 8; ++ i)
 {
		if(addr&0x80)
			Set(SPI_MOSI);
    else
			Reset(SPI_MOSI);
		Set(SPI_CLK);
		delay;
    Reset(SPI_CLK);
		delay;
    addr = addr << 1; 
 }
    for(i = 0; i < 8; ++i)
 {
		if(H&0x80)
			Set(SPI_MOSI);
		else
			Reset(SPI_MOSI);
		
    Set(SPI_CLK);
		delay;
		Reset(SPI_CLK);
		delay;
    H = H << 1;
 }
    for(i = 0; i < 8; ++i)
 {  
		if(L&0x80)
			Set(SPI_MOSI);
		else
			Reset(SPI_MOSI);
		Set(SPI_CLK);
		delay;
		
		Reset(SPI_CLK);
		delay;
    L = L << 1;
 }	
		delay;
    Set(SPI_SS);
}


void SPI_ReadReg(u8 addr)
{

    int i;	
		Reset(SPI_SS);
		delay;
    addr += 0x80;
    for(i = 0; i < 8; ++ i)
 {
		if(addr&0x80)
			Set(SPI_MOSI);
		else
			Reset(SPI_MOSI);
		
		Set(SPI_CLK);
		delay;
		Reset(SPI_CLK);
		delay;
    addr = addr << 1;
 }
    for(i = 0; i < 8; ++ i)
 {
		Set(SPI_CLK);
		delay;
		Reset(SPI_CLK);
		
    RegH = RegH << 1;  
    RegH |= Read(SPI_MISO);
		delay;
 }
    for(i = 0; i < 8; ++ i)
 {
		
		Set(SPI_CLK);
		delay;
		Reset(SPI_CLK);
    RegL = RegL << 1; 
    RegL |= Read(SPI_MISO);
		delay;
 }
		Set(SPI_SS);
}

void LT8910_Init(){
		
		SPI_GPIOInit();
		Reset(SPI_RST);
    delay_us(500);
		Set(SPI_RST);
		Reset(SPI_SS);
		
    delay_us(500);
		//Set(PKT);
    SPI_WriteWord( 0, 0x6f, 0xef );
    SPI_WriteWord( 1, 0x56, 0x81 );
    SPI_WriteWord( 2, 0x66, 0x17 );
    SPI_WriteWord( 4, 0x9c, 0xc9 );
    SPI_WriteWord( 5, 0x66, 0x37 );
    SPI_WriteWord( 7, 0x00, 0x00 );              //设置 2402Mhz
    SPI_WriteWord( 8, 0x6c, 0x90 );
    SPI_WriteWord( 9, 0x48, 0x00 );              //PA -12.2dbm
    SPI_WriteWord(10, 0x7f, 0xfd );
    SPI_WriteWord(11, 0x00, 0x08 );
    SPI_WriteWord(12, 0x00, 0x00 );
    SPI_WriteWord(13, 0x48, 0xbd );
    SPI_WriteWord(22, 0x00, 0xff );
    SPI_WriteWord(23, 0x80, 0x05 );
    SPI_WriteWord(24, 0x00, 0x67 );
    SPI_WriteWord(25, 0x16, 0x59 );
    SPI_WriteWord(26, 0x19, 0xe0 );
    SPI_WriteWord(27, 0x13, 0x00 );
    SPI_WriteWord(28, 0x18, 0x00 );
    SPI_WriteWord(32, 0x58, 0x00 );
    SPI_WriteWord(33, 0x3f, 0xc7 );
    SPI_WriteWord(34, 0x20, 0x00 );
    SPI_WriteWord(35, 0x0a, 0x00 );              //重发次数为9次 一共发送10次
    SPI_WriteWord(36, 0x02, 0x01 );
    SPI_WriteWord(37, 0x03, 0x01 );
    SPI_WriteWord(38, 0x04, 0x01 );
    SPI_WriteWord(39, 0x05, 0x01 );
    SPI_WriteWord(40, 0x44, 0x02 );
    SPI_WriteWord(41, 0xb8, 0x00 );              //CRC=ON;scramble=OFF;AUTO_ACK=ON
	
    SPI_WriteWord(42, 0xfd, 0xff );		//等待RX_ACK时间 255us  62.5KBPS
	
    SPI_WriteWord(43, 0x00, 0x0f );

    SPI_WriteWord(44, 0x10, 0x00);
    SPI_WriteWord(45, 0x05, 0x52);	
		
}

u8 RX_model(void)
{
				
		u8 i=0;
		u8 j=0;
	
		u8 flag=0;

		SPI_WriteWord(52, 0x80, 0x80);
		SPI_WriteWord(7, 0x00, 0x80 + 0x20);
		
    delay_us(500);	
		
    while(Read(PKT))                //等待数据jieshou完成	
		{
			;
		}
		
		SPI_ReadReg(48);
		if((RegH&0xC0))
			return 0;
		
    SPI_ReadReg(50);
		j=RegH;
		if(j>7)
			return ;
    while(i<j)
 {	
		
    SPI_ReadReg(50);
    rx_buf[i++]=RegH;
    rx_buf[i++]=RegL;	 
 }	
 
	return 1;
 
}


void SPI_WriteByte(u8 add,u8 H){
    u8 i;
		Reset(SPI_SS);
    for(i = 0; i < 8; ++ i)
 {
		if(add&0x80)
			Set(SPI_MOSI);
		else
			Reset(SPI_MOSI);
		
    Set(SPI_CLK);
		delay;
		Reset(SPI_CLK);
		delay;
    add = add << 1;
 }
 
    for(i = 0; i < 8; ++i) 
 {		
		if(H&0x80)
			Set(SPI_MOSI);
		else
			Reset(SPI_MOSI);
		Set(SPI_CLK);
		delay;
		Reset(SPI_CLK);
		delay;
    H = H << 1;
 }
		Set(SPI_SS);
}

void LT8910_Send(u8 * msg,u8 num){
    int i;
    SPI_WriteWord(52, 0x80, 0x80);		//清空FIFO
    SPI_WriteWord(7,  0x00, 0x00);		//进入发送模式
    SPI_WriteWord(50, num+1,0);			//表示发送总字节数
    i=0;
    while(i<num)
  {			                          //读取数据
		SPI_WriteByte(50,tx_buf[i++]);
  }
    SPI_WriteWord(7, 0x01, 0x20);      //开始发送
	
    while(!Read(PKT))                //等待数据发送完成	
		{
			;
		}
	
}

void LT8910_Get(){
		u8 i=0;
		u8 j=0;
    SPI_ReadReg(50);
		j=RegH;
    while(i<j)
 {
    SPI_ReadReg(50);
    rx_buf[i++]=RegH;
    rx_buf[i++]=RegL;	 
 }	
 //SPI_WriteWord(7, 0x00, 0x00);
}