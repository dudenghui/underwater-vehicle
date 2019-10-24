#include "Protocol.h"
#include "Tasks.h"

extern float duoji,light_c;
extern uint16_t depth_c,angle_c,forward_c,side_c,direct_c,direct_c1,direct_c2,duoji_c;
extern uint16_t ok,start;

uint16_t angle1, side1, forward1, depth1;

extern uint16_t depth_ctrl;

float THROTTLE,depth_E,direction,pump;
uint8_t key_Value[4];
uint8_t turn_l,turn_r; 

extern volatile uint8_t Rx_Buffer1[20];//串口1接收区


extern uint16_t lightt;

uint16_t jueduizhi(uint16_t shu1 , uint16_t shu2)
{
	uint16_t cha;
	cha= shu1-shu2;
	if(cha>0)
		return cha;
	if(cha<0)
		return (-1)*cha;
	else
		return 0;
}
/******************************************************************************
函数原型：	void Order_Get(void) 
功    能：	得到ROV控制指令
参    数：  无
*******************************************************************************/ 
void Order_Get(void)  
{
	if(Rx_Buffer1[6] == 0x59)//检验数据是否保存完毕
	{ 
		angle_c  =(uint16_t)(Rx_Buffer1[1]);
		depth_c  =(uint16_t)(Rx_Buffer1[2]);
		side_c   =(uint16_t)(Rx_Buffer1[3]);
		forward_c=(uint16_t)(Rx_Buffer1[4]);
		
		if((angle_c-angle1)>12)
			angle_c=angle1+12;
		else if((angle_c-angle1)<-12)
			angle_c=angle1-12;
		else
			angle_c=angle_c;
		
		if((side_c-side1)>2)
			side_c=side1+2;
		else if((side_c-side1)<-2)
			side_c=side1-2;
		else
			side_c=side_c;
		
		if((forward_c-forward1)>2)
			forward_c=forward1+2;
		else if((forward_c-forward1)<-2)
			forward_c=forward1-2;
		else
			forward_c=forward_c;
		
		if((depth_c-depth1)>4)
			depth_c=depth1+4;
		else if((depth_c-depth1)<-4)
			depth_c=depth1-4;
		else
			depth_c=depth_c;
		
		angle1  =angle_c  ;
		side1   =side_c   ;
		forward1=forward_c;
		depth1  =depth_c;
		
		if((Rx_Buffer1[5])==0x10)        //亮灯信号
		{
		   lightt=1;	
		}
		if((Rx_Buffer1[5])==0x0E)        //亮灯信号
		{
		   lightt=0;	
		}
		
		if((Rx_Buffer1[5])==0x0D)         //启动信号
		{
			start=1;
		}
		if((Rx_Buffer1[5])==0x0F)         //启动信号
		{
			start=0;
		}
		
		if((Rx_Buffer1[5])==0x09)         //全速  L2
		{
			direct_c=4;
		}
		else if((Rx_Buffer1[5])==0x0A)    //全速  R2
		{
			direct_c=3;
		}
        else if((Rx_Buffer1[5])==0x0B)     //半速  L1
		{
			direct_c=2;
		}
		else if((Rx_Buffer1[5])==0x0C)     //半速  R1
		{
			direct_c=1;
		}
        else
		{
			direct_c=0;
		}		
		
		
		
		if((Rx_Buffer1[5])==0x05)          //舵机 左转  UP
		{
			duoji_c=2;
		}
		else if((Rx_Buffer1[5])==0x07)     //舵机 右转  DOWN
		{
			duoji_c=1;
		}
		else
		{
			duoji_c=0;
		}	
		
		
		if((Rx_Buffer1[5])==0x08)          //定深 开  left
		{
			depth_ctrl=1;
		}
		if((Rx_Buffer1[5])==0x06)          //定深 关  right
		{
			depth_ctrl=0;
		}
		
	   /* direction = (((Rx_Buffer1[5])<<8)|(Rx_Buffer1[6]));     //方向
		if(direction<1800)
			 direction=0.2778*direction-500;
		else  if(direction>2300)
		           direction=0.2778*direction-639;  
		      else
				   direction=0; 
		
        THROTTLE  = (((Rx_Buffer1[7])<<8)|(Rx_Buffer1[8]));
		if(THROTTLE<1800)
			 THROTTLE=0.2778*THROTTLE+1000;

		else  if(THROTTLE>2300)
		           THROTTLE=0.2778*THROTTLE+861;  
		      else
				   THROTTLE=1500; 
		
	    pump  =  Rx_Buffer1[12];*/
	}


}












//		depth_E   = (((Rx_Buffer1[3])<<8)|(Rx_Buffer1[4]))*1.0/1000*2.4414;
//	    direction = (((Rx_Buffer1[5])<<8)|(Rx_Buffer1[6]))*0.044-90;
//        THROTTLE  = (((Rx_Buffer1[7])<<8)|(Rx_Buffer1[8]))*0.244+1000;
//	    pump      =  Rx_Buffer1[12];




