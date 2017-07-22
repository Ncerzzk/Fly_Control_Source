#include "mpu9250.h"
#include "BMP280.h"



void MPU_init(){
	I2C_init(I2C_USE);
	
	delay_us(1000*500);
	
	MPU_Single_Write(PWR_MGMT_1,0x00);
	MPU_Single_Write(SMPLRT_DIV, 0x07);
	MPU_Single_Write(CONFIG, 0x06);

	MPU_Single_Write(GYRO_CONFIG, GYRO_Range_Configure);   //
	MPU_Single_Write(ACCEL_CONFIG, ACCEL_Range_Configure); 
	
	
	
}


//将陀螺仪直接读出的16位符号数归一为 °/s
float Gyro_Normalize(s16 gyro){
	return ((float)gyro)*GYRO_Range/32768.0;
}





