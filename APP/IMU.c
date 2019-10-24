#include "IMU.h"
#include "math.h"
#include "Uart.h" 
#include "myiic.h"
#include "Control.h"

extern volatile uint8_t Rx_Buffer2[10];//����3������

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
����ԭ�ͣ�	void IMU_Get(void) 
��    �ܣ�	�õ�ROV��̬����
��    ����  ��
*******************************************************************************/ 
void IMU_Get(void) 
{
	
	if(Rx_Buffer2[8] == 0x77)//���������Ƿ񱣴����
	{ 
		if(Rx_Buffer2[1]==0x51)//���ٶ����ݺϳ�
	    {
			      ax=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*16*9.8;//
            ay=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*16*9.8;//
            az=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*16*9.8;//
		}                                                    
	                                                         
	    if(Rx_Buffer2[1]==0x52)//���ٶ����ݺϳ�
	    {
			      gx=(((short)Rx_Buffer2[3]<<8)|Rx_Buffer2[2])/32768.0*2000;
            gy=(((short)Rx_Buffer2[5]<<8)|Rx_Buffer2[4])/32768.0*2000;
            gz=(((short)Rx_Buffer2[7]<<8)|Rx_Buffer2[6])/32768.0*2000;;
		}
	
        if(Rx_Buffer2[1]==0x53)//�Ƕ����ݺϳ�
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
						
						
			Qx=Qx+90;    //���Ƕ�ת��Ϊ���Է��͵���
			Qy=Qy+90;
			Qz=Qz+180;
						
/*********************************������ֵ�˲�**********************************************/
//uint16_t m,n;				  
//for(m=0;m<10;m++)				  
//{
//  
//	
//}

if(start==1)	
{	
q_x[0]=Qx;				                               //��Ϊ�����˲�
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
				  
				  
				  
				  
				  
				  
/*********************************������ֵ�˲�**********************************************/	
			angle_actral=90-Qx;//90��Ϊƽ��ʱ�Ĳ����Ƕ� ����Ϊ�� ����Ϊ��
						
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



























