#include "stm32f10x.h"

#include "main.h"
#include "base.h"
#include "USART.h"
#include "timer.h"
#include "mpu9250.h"
#include "lt8910.h"
#include "set.h"
#include "angle.h"
#include "control.h"

#include "BLE.h"
#include "SR04.h"
GPIO_InitTypeDef GPIO_InitStructure;



void main(){
	
	SystemInit();
	Usart_Init(USART,115200);
	
	uprintf(USART,"Hello,world!\r\n");

	delay_init(72);
	MPU_init();
	//Tim2_Init();
	TIMER_Init(TIM6);
	if(CAP_TIM==TIM3){
		TIM3_Cap_Init();
	}
	SR04_Init();
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
		//uprintf(USART,"%f\r\n",height);
		
		/*
		1、标定角度正方向  pitch前倾为负   roll左倾为正
		2、标定角速度正方向  X左倾为正   Y前倾为正 Z 顺时针为正
		3、根据此正方向，修改PID中参数的符号
		*/
			if(Send_Angle){
				send_wave((int)pitch,(int)roll,(int)height,(int)Height_PID.i);//标定角度正方向
				//send_wave((int)angle_speed_X,(int)angle_speed_Y,(int)angle_speed_Z,0);//标定角速度正方向
				
				//send_wave((int)Angle_Speed_X_Out,(int)Angle_Speed_Y_Out,0,0);
				//send_wave((int)CH1_Out,(int)CH2_Out,(int)CH3_Out,(int)CH4_Out);
			}
			//uprintf(USART,"Height:%f\r\n",height);
	}
}

u8 overflow;//溢出计数

typedef enum{
	WAIT_RISING,
	WAIT_FALLING
}cap_state;

cap_state state=WAIT_RISING;


void TIM3_IRQHandler(void)  
{   
		if (TIM_GetITStatus(CAP_TIM, TIM_IT_Update) != RESET)  {         
				++overflow;  
		}  
			
		if (TIM_GetITStatus(CAP_TIM, TIM_IT_CC1) != RESET)//捕获事件  
		{     
			if(state == WAIT_RISING)      //初始状态,此时已经捕捉到了上升沿。             
				{     
						overflow = 0;  
						TIM_SetCounter(CAP_TIM,0);  
						state = WAIT_FALLING;    
						TIM_OC1PolarityConfig(CAP_TIM,TIM_ICPolarity_Falling);     //设置为下降沿捕获  
				}      
				else if(state==WAIT_FALLING)// wait falling  
				{  
						cap_time = TIM_GetCapture1(CAP_TIM);  
						cap_time+= overflow*2000;  
						
					/*
						if(cap_count<CAP_BUFFER_SIZE){
							cap_time_data[cap_count]=cap_time;
							cap_count++;
						}
					*/
					
						overflow = 0;  
						TIM_SetCounter(CAP_TIM,0);  
						state = WAIT_RISING;  
						TIM_OC1PolarityConfig(CAP_TIM,TIM_ICPolarity_Rising); //设置为上升沿捕获  
							
				}  
		}                                                  
   
    TIM_ClearITPendingBit(CAP_TIM, TIM_IT_CC1|TIM_IT_Update); //清除中断  
}  
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
				Fly_Control();
				//uprintf(USART,"test\r\n");
				break;
			case 4:
				SR04_Trig();
				//if(cap_count>=CAP_BUFFER_SIZE){
				Get_Height();
					//cap_count=0;
				//}
				break;
			default:
				break;
		}
		
		count=(count+1)%5;
		TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	}	
}
void UART4_IRQHandler(void){
	char c;
	if(USART_GetITStatus(USART, USART_IT_RXNE) != RESET) {  
		USART_ClearITPendingBit(USART,USART_IT_RXNE); 
		c=USART_ReceiveData(USART);
		switch(c){
			case 't':
				Send_Angle=!Send_Angle;
				uprintf(USART,"SEND change!\r\n");
				break;
			case 'd':
				Debug=!Debug;
				uprintf(USART,"Debug=  %d\r\n",Debug);
				break;
			case 'f':
				Fly=1;
				uprintf(USART,"start fly!\r\n");
				break;
			case 's':
				Fly=0;
				uprintf(USART,"stop fly!\r\n");
				break;
			case 'p':
				Roll_PID.KP+=0.01;
				Pitch_PID.KP-=0.01;
				uprintf(USART,"pitch_P=%f    roll_P=%f \r\n",Pitch_PID.KP,Roll_PID.KP);
			
				break;
			case 'q':
				Roll_PID.KP-=0.01;
				Pitch_PID.KP+=0.01;
				uprintf(USART,"pitch_P=%f    roll_P=%f \r\n",Pitch_PID.KP,Roll_PID.KP);
				break;
			case 'a':
				base_duty+=0.3;
				uprintf(USART,"base_duty=%f\r\n",base_duty);
				break;
			case 'b':
				base_duty-=0.3;
				uprintf(USART,"base_duty=%f\r\n",base_duty);
				break;
			case 'i':
				pitch_target+=0.5;
				uprintf(USART,"pitch_target=%f\r\n",pitch_target);
				break;
			case 'o':
				pitch_target-=0.5;
				uprintf(USART,"pitch_target=%f\r\n",pitch_target);
				break;	
			case 'e':
				roll_target+=0.5;
				uprintf(USART,"roll_target=%f\r\n",roll_target);
				break;					
			case 'r':
				roll_target-=0.5;
				uprintf(USART,"roll_target=%f\r\n",roll_target);
				break;
			case 'z':
				Roll_PID.KI+=0.0001;
				Pitch_PID.KI-=0.0001;
				uprintf(USART,"angle_speed_ki=%f\r\n",Roll_PID.KI);
				break;
			case 'x':
				Roll_PID.KI-=0.0001;
				Pitch_PID.KI+=0.0001;
				uprintf(USART,"angle_speed_ki=%f\r\n",Roll_PID.KI);
				break;
			case 'c':
				ANGLE_SPEED_Z_PID.KP+=0.1;
				uprintf(USART,"yaw_KP=%f\r\n",ANGLE_SPEED_Z_PID.KP);
				break;
			case 'v':
				ANGLE_SPEED_Z_PID.KP-=0.1;
				uprintf(USART,"yaw_KP=%f\r\n",ANGLE_SPEED_Z_PID.KP);
				break;				
			case 'n':
				Height_PID.KI+=0.001;
				uprintf(USART,"height_KI=%f\r\n",Height_PID.KI);
				break;
			case 'm':
				Height_PID.KI-=0.001;
				uprintf(USART,"height_KI=%f\r\n",Height_PID.KI);
				break;
			case 'k':
				Height_PID.KP+=0.1;
				uprintf(USART,"Height_PID.KP=%f\r\n",Height_PID.KP);
				break;
			case 'l':
				Height_PID.KP-=0.1;
				uprintf(USART,"Height_PID.KP=%f\r\n",Height_PID.KP);
				break;				
			default:
				break;
		}
	}
}
