#include "IMU.h"
#include "math.h"
#include "Uart.h" 
#include "myiic.h"
#include "Control.h"

extern volatile uint8_t Rx_Buffer2[10];//串口3接收区

float ax,ay,az;
float gx,gy,gz;
float Qx,Qy,Qz;
float q_x[10],q_y[10],q_z[10];
float depth_R;

extern uint16_t ok,start;
uint16_t angle_f1[10];
uint8_t angle_f[10];

extern int16_t angle_actral;
/******************************************************************************
函数原型：	void IMU_Get(void) 
功    能：	得到ROV姿态数据
参    数：  无
*******************************************************************************/ 
void IMU_Get(void) 
{
	
	if(Rx_Buffer2[8] == 0x77)//检验数据是否保存完毕
	{ 
		if(Rx_Buffer2[1]==0x51)//加速度数据合成
	    {
			      ax=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*16*9.8;//
            ay=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*16*9.8;//
            az=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*16*9.8;//
		}                                                    
	                                                         
	    if(Rx_Buffer2[1]==0x52)//角速度数据合成
	    {
			      gx=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*2000;
            gy=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*2000;
            gz=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*2000;;
		}
	
        if(Rx_Buffer2[1]==0x53)//角度数据合成
	    {
		    Qx=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*180;
		      	if(Qx>180)   
							Qx=Qx-360;
			        
            Qy=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*180;
		      	if(Qy>180)   
							Qy=Qy-360;
			
            Qz=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*180;
			      if(Qz>180)   
							Qz=Qz-360;	   	
						
						
			Qx=Qx+90;    //将角度转换为可以发送的量
			Qy=Qy+90;
			Qz=Qz+180;
						
/*********************************滑动均值滤波**********************************************/
//uint16_t m,n;				  
//for(m=0;m<10;m++)				  
//{
//  
//	
//}

if(start==1)	
{	
q_x[0]=Qx;				                               //此为抖动滤波
if((q_x[0]-q_x[1])>20||(q_x[0]-q_x[1])<-20)			  
	q_x[0]=q_x[1];			  
q_x[1]=q_x[0];
Qx=q_x[0];
				  
q_y[0]=Qy;				  
if((q_y[0]-q_y[1])>20||(q_y[0]-q_y[1])<-20)			  
	q_y[0]=q_y[1];			  
q_y[1]=q_y[0];				  
Qy=q_y[0];				  
				  
q_z[0]=Qz;				  
if((q_z[0]-q_z[1])>20||(q_z[0]-q_z[1])<-20)			  
	q_z[0]=q_z[1];			  
q_z[1]=q_z[0];				  
Qz=q_z[0];				  
}			
else
{
  q_x[1]=Qx;
  q_y[1]=Qy;
  q_z[1]=Qz;	
}
				  
				  
				  
				  
				  
				  
/*********************************滑动均值滤波**********************************************/	
			angle_actral=90-Qx;//90作为平放时的补偿角度 左倾为负 右倾为正
						
			angle_f[0]=(uint8_t)((int16_t)(Qy*100)/200);
			angle_f[1]=(uint8_t)((int16_t)(Qy*100)%200);
			angle_f[2]=(uint8_t)((int16_t)(Qx*100)/200);
			angle_f[3]=(uint8_t)((int16_t)(Qx*100)%200);
			angle_f[4]=(uint8_t)((int16_t)(Qz*100)/200);
			angle_f[5]=(uint8_t)((int16_t)(Qz*100)%200);
						
						
			ax=(float)((float)angle_f[0]*2+(float)(angle_f[1])/100);
			ay=(float)((float)angle_f[2]*2+(float)(angle_f[3])/100);
			az=(float)((float)angle_f[4]*2+(float)(angle_f[5])/100);
		}
	
	}
    //depth_R = readMS5540();  //return depth 

}



























