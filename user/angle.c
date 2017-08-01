#include "angle.h"

#define INTEGRAL_CONSTANT 0.005		//积分时间 5ms
#define HALF_T 0.0025
#define GRAVITY_CONSTANT 9.8			//重力加速度
#define Semig 1e-6
#define GY521 1


float a_x_offset=0;
float a_y_offset=0;
float a_z_offset=0;
float w_x_offset,w_y_offset,w_z_offset;


	
	
s16 gyro_X,gyro_Y,gyro_Z;     //陀螺仪读出的16位有符号数
s16 accel_X,accel_Y,accel_Z;  //加速度计读出的16位有符号数
float roll_accel,pitch_accel;
float angle_speed_X,angle_speed_Y,angle_speed_Z;   //陀螺仪归一化后的角速度
float accel_speed_X,accel_speed_Y,accel_speed_Z;   //加速度归一化后

float gravity_X,gravity_Y,gravity_Z; //重力加速度分量

float v_x,v_y,v_z;    //三个轴上的速度

float a_x,a_y,a_z;    //三个轴上的加速度（去除重力加速度分量）

float s_x,s_y,s_z;		//三个轴的距离

float q0=1;
float q1=0;
float q2=0;
float q3=0;

float IMU_P=2;
float IMU_I=0.005;
double pitch,roll,yaw;

float BMP_height;

void BMP_Get_Height(){
	float temp;
	float sub;
	if(!BMP_OPEN)
		return ;
	BMP_height=(float)BMP_Get_Data();
	BMP_height=BMP_HEIGHT_CONSTANT-BMP_height;
	BMP_height*=8;
	BMP_height/=100;
}




/*
	加速度计采集函数
*/

void Get_Accel_Angle(){
	accel_X=Get_Accel_X();
	accel_Y=Get_Accel_Y();
	accel_Z=Get_Accel_Z();
	
	accel_speed_X=((float)accel_X)*ACCEL_Range/32768.0;
	accel_speed_Y=((float)accel_Y)*ACCEL_Range/32768.0;
	accel_speed_Z=((float)accel_Z)*ACCEL_Range/32768.0;
	
}

/*
	陀螺仪采集函数
*/
void Get_Angle_Speed(){
	gyro_X=Get_Gyro_X();
	gyro_Y=Get_Gyro_Y();
	gyro_Z=Get_Gyro_Z();
	
	//卡尔曼之后，归一化
	angle_speed_X=((float)gyro_X)*GYRO_Range/32768.0/57.32-w_x_offset;
	angle_speed_Y=((float)gyro_Y)*GYRO_Range/32768.0/57.32-w_y_offset;
	angle_speed_Z=((float)gyro_Z)*GYRO_Range/32768.0/57.32-w_z_offset;
	
}

/*
	自检陀螺仪，获取零偏
*/




void Adjust_Gyro(){

	int i;
	float w_x_sum=0,w_y_sum=0,w_z_sum=0;
	
	TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);
	
	for(i=0;i<100;++i){
		
		Get_Angle_Speed();
		w_x_sum+=angle_speed_X+w_x_offset;
		w_y_sum+=angle_speed_Y+w_y_offset;
		w_z_sum+=angle_speed_Z+w_z_offset;
		delay_us(2000);  //delay_2ms
	}
	
	

	
	w_x_offset=w_x_sum/100;
	w_y_offset=w_y_sum/100;
	w_z_offset=w_z_sum/100;
	

	
	uprintf(USART,"adjust gyro successful! don't forget to write to the prams!\r\n");
	
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
}



//获取偏航角度（磁力计）
void Get_Mag_Angle(){
	s16 x,y;
	s8 asax,asay;
	float result=0;
	float adjustx,adjusty;
	
	MPU_Single_Write(INT_PIN_CFG,0x02);    //MPU6500 开启路过模式
	
	delay_us(100);
	
	asax=MAG_Single_Read(MAG_ASAX);
	asay=MAG_Single_Read(MAG_ASAY);
	
	adjustx=((asax-128)*0.5/128.0)+1;
	adjusty=((asay-128)*0.5/128.0)+1;
	
	x=MAG_Get_Data(MAG_XOUT_L);
	y=MAG_Get_Data(MAG_YOUT_L);
	
	MAG_Single_Write(MAG_CONTROL, 0x01);     //启动单次转换
	result=atan2(y*adjusty,(x*adjustx))*180/3.14;
	
	uprintf(USART1,"%f\r\n",result);
}

void Get_Angle(){
	float pitch_sub,roll_sub;
	
	pitch_sub=pitch_accel-pitch;
	roll_sub=roll_accel-roll;
	
	pitch+=(pitch_sub/0.8+angle_speed_X)*INTEGRAL_CONSTANT;
	roll+=(roll_sub/0.8-angle_speed_Y)*INTEGRAL_CONSTANT;
	
	
}



void IMU_Update(float ax,float ay,float az,float wx,float wy,float wz){
	float norm;
	float gbx,gby,gbz;
  float q0q0 = q0 * q0;                                                        
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q1q1 = q1 * q1;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;
	float q1q2=q1*q2;
	float q0q3=q0*q3;
	float ex,ey,ez;
	float exInt,eyInt,ezInt;
	norm=sqrt(ax*ax+ay*ay+az*az);
	if(norm<Semig)
		return ;
	ax/=norm;
	ay/=norm;
	az/=norm; 
	
	//计算重力加速度旋转到机体坐标系后的值,即以当前估计的姿态作为旋转矩阵
	gbx= 2*(q1q3 - q0q2);
	gby= 2*(q0q1 + q2q3);
	gbz= q0q0 - q1q1 - q2q2 + q3q3;
	
	//与实际加速度计测得的ax,ay,az做叉积，取误差
	
	
    ex = (ay*gbz - az*gby);                                                                
    ey = (az*gbx - ax*gbz);
    ez = (ax*gby - ay*gbx);	
	
	
		exInt += ex*IMU_I*INTEGRAL_CONSTANT;
		eyInt += ey*IMU_I*INTEGRAL_CONSTANT;
		ezInt += ez*IMU_I*INTEGRAL_CONSTANT;
		
		//补偿误差
		wx+=ex*IMU_P+exInt;
		wy+=ey*IMU_P+eyInt;
		wz+=ez*IMU_P+ezInt;
	
		//更新四元数
		q0=  q0 + (-q1*wx - q2*wy - q3*wz)*HALF_T;
		q1 = q1 + (q0*wx + q2*wz - q3*wy)*HALF_T;
		q2 = q2 + (q0*wy - q1*wz + q3*wx)*HALF_T;
		q3 = q3 + (q0*wz + q1*wy - q2*wx)*HALF_T; 
	
		//计算欧拉角
		norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
		if(norm<Semig)
			return ;
		q0/=norm;
		q1/=norm;
		q2/=norm;
		q3/=norm;
		
		pitch= asin(-2*q1*q3 + 2*q0*q2)* 57.3;
		roll= atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2+1)* 57.3; 
		yaw=atan2(2*q1q2 + 2*q0q3, -2*q2q2 - 2*q3q3+1)* 57.3; 
		
		if(GY521){
			if(roll>0){
				roll-=180;
			}else{
				roll+=180;
			}
		}
}


