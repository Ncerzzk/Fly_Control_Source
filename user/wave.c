#include "wave.h"
#include "math.h"


typedef enum{
	Triangle,
	Squre
}Wave_Type;


typedef struct {
	Wave_Type type;
	u16 T;  //一个周期所代表的点，点数越多，精度越高
	u16 now_ptr; //当前在一个周期内的位置
}Wave;


void Wave_set(Wave *w,Wave_Type type,u16 T){
	w->type=type;
	w->T=T;
	w.now_ptr=0;
}

double Wave_next(Wave *w){
	u16 ptr=w->now_ptr;
	double result;
	switch(w->type){
		case Triangle:
			result=sin((ptr/w->T)*2*3.14);
			break;
		case Square:
			if(ptr>w->T/2){
				result=0;
			}else{
				result=1;
			}
			break;
	}
	w->now_ptr++;
	return result;
}

