#ifndef __ANGLE_H
#define __ANGLE_H

#include "angle.h"
#include "math.h"
#include "mpu9250.h"
#include "base.h"
#include "USART.h"
#include "outputdata.h"

#define FLASH_Start 0x08019000
#define FLASH_Page_Size 0x400

#define BLE_CONTROL	1



typedef enum{
	START_PRE,
	FLYING,
	STAND,
	STOP,
	DROP,
	LAND,
	ADJUST,
	SLOW
}FLY_STATE;

typedef enum{
	FLY_FREE,
	FLY_SET,
	FLY_CONTROL
}FLY_MODE;

typedef enum{
	left,
	right,
	ahead,
	back
}DIRECTION;

float KalMan(Kal_Struct *kal,float x);
void Get_Accel_Angle();
void Get_Angle_Speed();
void Write_Prams();
void Load_Prams();
void Get_Mag_Angle();
void IMU_Update(float ax,float ay,float az,float wx,float wy,float wz);
void Get_Angle();
void Adjust_Gyro();
void Adjust_Acc();
extern float a_z_offset,a_x_offset,a_y_offset;

extern float angle_speed_X,angle_speed_Y,angle_speed_Z;   //陀螺仪归一化后的角速度
extern float accel_speed_X,accel_speed_Y,accel_speed_Z;   //加速度归一化后
extern double pitch,roll,yaw;
extern float roll_accel,pitch_accel;
extern u8 Adjust_Acc_State;
extern u8 Pram_Error;
#endif

