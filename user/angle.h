#ifndef __ANGLE_H
#define __ANGLE_H

#include "angle.h"
#include "math.h"
#include "mpu9250.h"
#include "base.h"
#include "USART.h"
#include "outputdata.h"



#define BLE_CONTROL	1




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
void Get_Accel_Angle(void);
void Get_Angle_Speed(void);

void Get_Mag_Angle(void);
void IMU_Update(float ax,float ay,float az,float wx,float wy,float wz);
void AHR_Update(float ax,float ay,float az,float wx,float wy,float wz,float mx,float my,float mz);  
void IMU_Update2(float ax,float ay,float az,float wx,float wy,float wz);
void Get_Angle(void);
void Adjust_Gyro(void);
void Adjust_Acc(void);
extern float a_z_offset,a_x_offset,a_y_offset;
extern float w_x_offset,w_y_offset,w_z_offset;

extern float angle_speed_X,angle_speed_Y,angle_speed_Z;   //陀螺仪归一化后的角速度
extern float accel_speed_X,accel_speed_Y,accel_speed_Z;   //加速度归一化后
extern double pitch,roll,yaw;
extern float roll_accel,pitch_accel;
extern u8 Adjust_Acc_State;
extern u8 Pram_Error;

#endif

