#include "stm32f10x.h"

#include "main.h"
#include "base.h"
#include "USART.h"
#include "timer.h"
#include "mpu9250.h"
#include "lt8910.h"

#include "angle.h"
#include "control.h"

#include "BLE.h"


GPIO_InitTypeDef GPIO_InitStructure;

u8 Debug=0;

void main(){
	
	SystemInit();
	Usart_Init(USART,9600);
	
	uprintf(USART,"Hello,world!\r\n");

	delay_init(72);
	MPU_init();
	//Tim2_Init();
	TIMER_Init(TIM6);
	Brush_Init();
	//Load_Prams();
	uprintf(USART,"in  it ok!\r\n");
	
	//uprintf(USART1,"mode=%d\r\nstate=%d\r\n",mode,state);
	
	//BLE_Send_Over();

	
	
	while(1){	
			//send_wave((int)(gravity_Y*100),(int)(pitch),(int)(kal_ac_y.kal_out*100),(int)(a_y*100));
		//send_wave((int)roll,(int)height*10,(int)angle_speed_X,(int)pitch);
		//uprintf(USART1,"%f\r\n",fly_speed);
		//send_wave((int)kal_gy_z.kal_out,(int)angle_speed_Z,(int)yaw,(int)pitch);
		//send_wave((int)s_x,(int)s_y,(int)v_x,(int)yaw);
		//uprintf(USART1,"%f\r\n",height);
		//uprintf(USART,"%f\r\n",pitch);
		if(Debug){
			send_wave((int)pitch,(int)roll,0,0);
		}
	}
}


/*
void TIM2_IRQHandler(){
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);   //关中断
		switch(count){
			case 1:
				Get_Angle_Speed();
				Get_Accel_Angle();
				break;
			case 2:
				IMU_Update(accel_speed_X,accel_speed_Y,accel_speed_Z,angle_speed_X,angle_speed_Y,angle_speed_Z);
				//Get_Angle();
				break;
			default:
				break;
		}
		
		count=(count+1)%5;
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	}
}
*/
int count=0;
void TIM6_IRQHandler(void){
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET){
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);   //关中断
		switch(count){
			case 1:
				Get_Angle_Speed();
				Get_Accel_Angle();
				break;
			case 2:
				IMU_Update(accel_speed_X,accel_speed_Y,accel_speed_Z,angle_speed_X,angle_speed_Y,angle_speed_Z);
				//Get_Angle();
				break;
			case 3:
				uprintf(USART,"test\r\n");
				break;
			default:
				break;
		}
		
		count=(count+1)%5;
		TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	}	
}
void USART1_IRQHandler(void){
	char c;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {  
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
		c=USART_ReceiveData(USART1);
		switch(c){
			case 'a':
				break;
			case 'd':
				Debug=!Debug;
				break;
			default:
				break;
		}
	}
}
