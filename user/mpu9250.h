#ifndef __MPU9250_H
#define __MPU9250_H

#include "i2c.h"



#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x01(500Hz)
//采样频率=陀螺仪输出频率/(1+SMPLRT_DIV )    陀螺仪输出频率当低通滤波不开的时候为8KHZ，当低通开的时候为1K

#define	CONFIG			0x1A	//低通滤波频率，典型值：0x01(188Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define INT_PIN_CFG		0x37
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I			0x75	//IIC地址寄存器(默认数值0x68，只读)

#define GYRO_Range_Configure 0x8   //0x10
#define GYRO_Range 500

#define ACCEL_Range_Configure 0x1  
#define ACCEL_Range 2*9.8



#define MPU_Address   0xD0
#define MAG_Address 	0x18

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define MAG_CONTROL 	0x0A

#define MAG_ASAX			0x10
#define MAG_ASAY			0x11
#define MAG_ASAZ			0x12




#define BMP_OPEN	1
#define BMP_Address 												 0xEC  

#define BMP280_DIG_T1_LSB_REG                0x88  
#define BMP280_DIG_T1_MSB_REG                0x89  
#define BMP280_DIG_T2_LSB_REG                0x8A  
#define BMP280_DIG_T2_MSB_REG                0x8B  
#define BMP280_DIG_T3_LSB_REG                0x8C  
#define BMP280_DIG_T3_MSB_REG                0x8D  
#define BMP280_DIG_P1_LSB_REG                0x8E  
#define BMP280_DIG_P1_MSB_REG                0x8F  
#define BMP280_DIG_P2_LSB_REG                0x90  
#define BMP280_DIG_P2_MSB_REG                0x91  
#define BMP280_DIG_P3_LSB_REG                0x92  
#define BMP280_DIG_P3_MSB_REG                0x93  
#define BMP280_DIG_P4_LSB_REG                0x94  
#define BMP280_DIG_P4_MSB_REG                0x95  
#define BMP280_DIG_P5_LSB_REG                0x96  
#define BMP280_DIG_P5_MSB_REG                0x97  
#define BMP280_DIG_P6_LSB_REG                0x98  
#define BMP280_DIG_P6_MSB_REG                0x99  
#define BMP280_DIG_P7_LSB_REG                0x9A  
#define BMP280_DIG_P7_MSB_REG                0x9B  
#define BMP280_DIG_P8_LSB_REG                0x9C  
#define BMP280_DIG_P8_MSB_REG                0x9D  
#define BMP280_DIG_P9_LSB_REG                0x9E  
#define BMP280_DIG_P9_MSB_REG                0x9F  
  
#define BMP280_CHIPID_REG                    0xD0  /*Chip ID Register */  
#define BMP280_RESET_REG                     0xE0  /*Softreset Register */  
#define BMP280_STATUS_REG                    0xF3  /*Status Register */  
#define BMP280_CTRLMEAS_REG                  0xF4  /*Ctrl Measure Register */  
#define BMP280_CONFIG_REG                    0xF5  /*Configuration Register */  
#define BMP280_PRESSURE_MSB_REG              0xF7  /*Pressure MSB Register */  
#define BMP280_PRESSURE_LSB_REG              0xF8  /*Pressure LSB Register */  
#define BMP280_PRESSURE_XLSB_REG             0xF9  /*Pressure XLSB Register */  
#define BMP280_TEMPERATURE_MSB_REG           0xFA  /*Temperature MSB Reg */  
#define BMP280_TEMPERATURE_LSB_REG           0xFB  /*Temperature LSB Reg */  
#define BMP280_TEMPERATURE_XLSB_REG          0xFC  /*Temperature XLSB Reg */  

#define BMP_HEIGHT_CONSTANT				101325



#define MPU_Single_Read(address)   I2C_ByteRead(I2C_USE,MPU_Address,address)
#define MAG_Single_Read(address)   I2C_ByteRead(I2C_USE,MAG_Address,address)
#define BMP_Single_Read(address)   I2C_ByteRead(I2C_USE,BMP_Address,address)
#define MPU_Get_Data(address)				I2C_DoubleRead(I2C_USE,MPU_Address,address,MSB_First)
#define MAG_Get_Data(address)				I2C_DoubleRead(I2C_USE,MAG_Address,address,LSB_First)
//#define BMP_Get_Data(address)				I2C_DoubleRead(BMP_Address,address,LSB_First)
#define MPU_Single_Write(address,data) 		I2C_ByteWrite(I2C_USE,MPU_Address,data,address)
#define MAG_Single_Write(address,data)		I2C_ByteWrite(I2C_USE,MAG_Address,data,address)
#define BMP_Single_Write(address,data)		I2C_ByteWrite(I2C_USE,BMP_Address,data,address)


#define Get_Gyro_Y()		MPU_Get_Data(GYRO_YOUT_H)
#define Get_Gyro_X()		MPU_Get_Data(GYRO_XOUT_H)
#define Get_Gyro_Z()		MPU_Get_Data(GYRO_ZOUT_H)
#define Get_Accel_Z()		MPU_Get_Data(ACCEL_ZOUT_H)
#define Get_Accel_X()		MPU_Get_Data(ACCEL_XOUT_H)
#define Get_Accel_Y()		MPU_Get_Data(ACCEL_YOUT_H)

typedef enum{
	GYROX,
	GYROY,
	GYROZ
}GYRO;

void MPU_init();
float Gyro_Normalize(s16 gyro);
double BMP_Get_Data();

#endif

