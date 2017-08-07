/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "base.h"
#include "ms5611.h"
#include "i2c.h"
#include "set.h"
#include "USART.h"
#include "SR04.h"


// MS5611, Standard address 0x77
#define MS5611_ADDR                 0xEE    //0x77左移1位
// Autodetect: turn off BMP085 while initializing ms5611 and check PROM crc to confirm device
#define BMP085_OFF                  digitalLo(BARO_GPIO, BARO_PIN);
#define BMP085_ON                   digitalHi(BARO_GPIO, BARO_PIN);

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8

static void ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
static int8_t ms5611_crc(uint16_t *prom);
static uint32_t ms5611_read_adc(void);


static uint32_t ms5611_ut;  // static result of temperature measurement
static uint32_t ms5611_up;  // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;

int32_t pressure,temperature;

#define MS_Single_Write(address,data) I2C_ByteWrite(I2C_USE,MS5611_ADDR,data,address)

void ms5611_init(void){
	
		int i;
	
    ms5611_reset();
    // read all coefficients
    for (i = 0; i < PROM_NB; i++)
        ms5611_c[i] = ms5611_prom(i);	
	
}

int ms5611_cnt;
void ms5611_get_height(void){
	float temp;
	ms5611_cnt++;

	switch(ms5611_cnt){
		case 1:
			ms5611_start_up();
			break;
		case 10:
			ms5611_get_up();
			break;
		case 11:
			ms5611_start_ut();
			break;
		case 20:
			ms5611_get_ut();
			break;
		case 21:
			ms5611_calculate();
			temp=HEIGHT_CONSTANT-pressure;
			temp/=8;
			temp/=100;
			height=temp;
			uprintf(USART,"%f\r\n",height);
			//uprintf(USART,"p:%d     t:%d     height:%f\r\n",pressure,temperature,temp);
			break;
		case 25:
			ms5611_cnt=0;
			break;
		default:
			break;
	}
}

static void ms5611_reset(void)
{
		MS_Single_Write(CMD_RESET, 1);
    delay_us(2800*10);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };
		I2C_BufferRead(I2C_USE,MS5611_ADDR,rxbuf,CMD_PROM_RD + coef_num * 2,2);
    //i2cRead(MS5611_ADDR, CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command
    return rxbuf[0] << 8 | rxbuf[1];
}

static int8_t ms5611_crc(uint16_t *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t zero = 1;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    // if eeprom is all zeros, we're probably fucked - BUT this will return valid CRC lol
    for (i = 0; i < 8; i++) {
        if (prom[i] != 0)
            zero = 0;
    }
    if (zero)
        return -1;

    for (i = 0; i < 16; i++) {
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}

static uint32_t ms5611_read_adc(void)
{
    uint8_t rxbuf[3];
		I2C_BufferRead(I2C_USE,MS5611_ADDR,rxbuf,CMD_ADC_READ,3);
    //i2cRead(MS5611_ADDR, CMD_ADC_READ, 3, rxbuf); // read ADC
    return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

void ms5611_start_ut(void)
{
    MS_Single_Write(CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}

void ms5611_get_ut(void)
{
    ms5611_ut = ms5611_read_adc();
}

void ms5611_start_up(void)
{
    MS_Single_Write(CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr, 1); // D1 (pressure) conversion start!
}

void ms5611_get_up(void)
{
    ms5611_up = ms5611_read_adc();
}

void ms5611_calculate(void)
{
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int32_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    }
    press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;

        pressure = press;
        temperature = temp;
}
