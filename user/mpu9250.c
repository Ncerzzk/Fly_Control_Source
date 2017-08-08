#include "mpu9250.h"
#include "BMP280.h"

struct bmp280_t bmp280;

signed char BMP280_I2C_bus_read(unsigned char device_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
{
	I2C_BufferRead(I2C_USE,device_addr,reg_data,reg_addr,cnt);
	return 0;
}

signed char BMP280_I2C_bus_write(unsigned char device_addr, unsigned char reg_addr, unsigned char *reg_data, unsigned char cnt)
{
	I2C_ByteWrite(I2C_USE,device_addr,*reg_data,reg_addr);
	return 0;
}

void BMP280_delay_msec(BMP280_U16_t msec) //delay in milliseconds
{
	BMP280_U32_t counter;
	for (counter = 0; counter < 2000*msec; counter++); // 2000 counts = 1 msec
}
void BMP_Init(){
	bmp280.bus_read = BMP280_I2C_bus_read;
	bmp280.bus_write = BMP280_I2C_bus_write;
	bmp280.delay_msec = BMP280_delay_msec;
	bmp280_init(&bmp280);
	bmp280_set_osrs_t(BMP280_OVERSAMPLING_1X);
	bmp280_set_osrs_p(BMP280_OVERSAMPLING_8X);
}

double BMP_Get_Data(){
	BMP280_S32_t upressure, utemperature;
	double compensated_temperature_d, compensated_pressure_d;
	bmp280_get_forced_uput(&upressure, &utemperature);
	compensated_temperature_d = bmp280_compensate_T_double(utemperature); //in DegC
	return bmp280_compensate_P_double(upressure); //in Pa	
}


void MPU_init(){
	I2C_init(I2C_USE);
	
	delay_us(1000*500);
	
//	while(1){
//		I2C_Test();
//	}
	//I2C_ByteWrite(I2C_USE,0xA0,0x01,0x02);    //测试I2C从机用
	
	MPU_Single_Write(PWR_MGMT_1,0x00);
	MPU_Single_Write(SMPLRT_DIV, 0x00);
	MPU_Single_Write(CONFIG, 0x02);  //之前延时为20ms(0x06，现在为3ms左右 0x02)

	MPU_Single_Write(GYRO_CONFIG, GYRO_Range_Configure);   //
	MPU_Single_Write(ACCEL_CONFIG, ACCEL_Range_Configure); 
	
	
	
}


//将陀螺仪直接读出的16位符号数归一为 °/s
float Gyro_Normalize(s16 gyro){
	return ((float)gyro)*GYRO_Range/32768.0;
}





