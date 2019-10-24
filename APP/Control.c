#include "IMU.h"
#include "Motor.h"
#include "Control.h"
#include "Protocol.h"
#include "myiic.h"

struct _pid Heading;
struct _pid depth;
struct _pid pos;
struct _pid zhi;

float throttle1,throttle2,throttle3,throttle4,throttle5,throttle6;
uint8_t depth_flag,Heading_flag;
float depth_ess,depth_integration,depth_eess,depth_ess1,motor_out;  //PID�������

int16_t angle_actral;
int16_t angle_ess,angle_integration,angle_eess,angle_ess1,angle_out;  //PID�������
int16_t angle_ess_max,angle_ess_min,angle_integration_max,angle_integration_min,angle_eess_max,angle_eess_min;//PID������޷�

int16_t fixed=0;   //ʹ����ˮ��Ư������Ҫ�Ĵ�ֱ����
float depth_blance=0;

float depth_l;

extern uint16_t depth_ctrl;

extern uint16_t ok,start;
extern float shendu_actual;
/******************************************************************************
����ԭ�ͣ�	void PID_Init(void)
��    �ܣ�	����PID����
*******************************************************************************/ 
void PID_Init(void)
{
	Heading.kp  = 4.9f;
	Heading.ki  = 0.02f;
	Heading.kd  = 8.0f;
	
	depth.kp = 20.0f;
	depth.ki = 0.0f;
	depth.kd = 0.0f;
	
	pos.kp = 1.1f;
	pos.ki = 0.02f;
	pos.kd = 0.5f;
	
	zhi.kp = 2.1f;
	zhi.ki = 0.0f;
	zhi.kd = 0.0f;
}



#define depth_max 	10.0f	
#define depth_integral_max 	1000.0f	

int16_t pid_count;
/******************************************************************************
����ԭ�ͣ�	void depth_control(void)
��    �ܣ�	��ȿ���PID
*******************************************************************************/ 
void depth_control(uint16_t depth_want,uint16_t angle_want,uint16_t p)
{
	  int16_t depth_want1,angle_want1;
	
	  depth_want1=128-depth_want;
	  angle_want1=128-angle_want;   //angle_want1��Χ��-128------128
	  //motor_out=depth_want1;l
	
	  if(depth_ctrl)                             //���ö���ģʽ
	  {
		 depth_blance=depth_l;
		 depth_ess=shendu_actual-depth_blance;   //����Ϊ��λ
		  
//	        if(depth_ess>-5&&depth_ess<5)
//				depth_ess=0;
//			else
//			{
//				if(depth_ess>0)
//				depth_ess=depth_ess-5;
//				if(depth_ess<0)
//				depth_ess=depth_ess+5;
//				}
			if(depth_ess>20)  depth_ess=20;
			if(depth_ess<-20)  depth_ess=-20;
				
			depth_integration+=depth_ess;          //������
		  
	        depth_eess=depth_ess-depth_ess1;       //΢����
	   	 	depth_ess1=depth_ess;
				
			motor_out=depth.kp*depth_ess+depth.ki*depth_integration+depth.kd*depth_eess;
				
	        if(motor_out>50)
				motor_out=50;
			if(motor_out<-50)
				motor_out=-50;
	  }
	  else                                      //�����ֶ�ң��ģʽ
	  {
		 depth_l=shendu_actual-1;
		 depth_ess=0;
		 motor_out=depth_want1;
	  }

	  
	  
      angle_ess=angle_want1-angle_actral*4;    //angle_actralҲӦ����-30-----30ת��Ϊ-128------128
	  if(angle_ess>-8&&angle_ess<8)
		  angle_ess=0;
	  else
	    {
		  if(angle_ess>0)
		  angle_ess=angle_ess-8;
		  if(angle_ess<0)
		  angle_ess=angle_ess+8;
		}
	  angle_integration+=angle_ess;            //������
	  if(angle_integration>500)
		  angle_integration=500;
	  if(angle_integration<-500)
		  angle_integration=-500;
	  
	  angle_eess=angle_ess-angle_ess1;         //΢����      �������ʾƫ��Ӵ������Ҫ����
	  angle_ess1=angle_ess;
	  
      angle_want1=pos.kp*angle_ess+pos.ki*angle_integration+pos.kd*angle_eess;
	  if(angle_want1>110)
				angle_want1=110;
	  if(angle_want1<-110)
				angle_want1=-110;
		
	  throttle5=(float)(motor_out*3+fixed-angle_want1);     //�޷�Ϊ-500----500
	  throttle6=(float)(motor_out*3+fixed+angle_want1);	 

	
}

#define angle_max 	 10.0f	
#define angle_integral_max 	1000.0f	

int16_t direct2;
/******************************************************************************
����ԭ�ͣ�	void Heading_control(void)
��    �ܣ�	����PID
*******************************************************************************/ 
void Heading_control(uint16_t forward,uint16_t p_f,uint16_t side,uint16_t p_s,uint16_t direct,uint16_t p_d)
{
	int16_t zhuanxiang,zhuanxiang1,qian_ess,qian_ess1,qian_eess,qian_intre;
	int16_t forward1;
	int16_t side1;
	int16_t direct1;
	
	forward1=128-forward;
    throttle1=forward1*p_f/10;
	throttle2=forward1*p_f/10;
	throttle3=forward1*p_f/10;
	throttle4=forward1*p_f/10;
	
	side1=128-side;
	throttle1+=side1*p_s/10;
	throttle2-=side1*p_s/10;
	throttle3+=side1*p_s/10;
	throttle4-=side1*p_s/10;

	if(direct==4)
		direct1=128;
	else if(direct==3)
		direct1=-128;
	else if(direct==2)
		direct1=64;
	else if(direct==1)
		direct1=-64;
	else
		direct1=0;
	
	if((direct1-direct2)>12)
		direct1=direct2+12;
	else if((direct1-direct2)<-12)
		direct1=direct2-12;
	else
		direct1=direct1;
		
	direct2=direct1;
/*************************��һ�ַ���************************************/	
// if(direct==0&&Qz>20&&Qz<340&&start==1)    //û��ת��������ͽ��бջ�
//	{
//	  qian_ess=zhuanxiang1-Qz;
//	  if(qian_ess>-20&&qian_ess<20)
//		  qian_ess=0;
//	  else
//	    {
//		  if(qian_ess>0)
//		  qian_ess=qian_ess-20;
//		  if(qian_ess<0)
//		  qian_ess=qian_ess+20;
//		}
//	  qian_intre+=qian_ess;
//	  qian_eess=qian_ess-qian_ess1;
//	  qian_ess1=qian_ess;
//	  direct1=zhi.kp*qian_ess+zhi.ki*qian_intre+zhi.kd*qian_eess;
//	}
//	else
//	{
//	  zhuanxiang1=Qz;
//      direct1=direct1;
//	}
//	
//	pid_count++;
//	if(pid_count>200)
//	{
//	   pid_count=0;
//	   zhuanxiang1=Qz;
//	}

/*************************�ڶ��ַ���************************************/	

//	if(direct==0)    //û��ת��������ͽ��бջ�
//	{
//	  zhuanxiang1=Qz;
//	  qian_ess=zhuanxiang-zhuanxiang1;
//		zhuanxiang=zhuanxiang1;
//		
//	  
//	  
//	  if(qian_ess>-5&&qian_ess<5)
//		  qian_ess=0;
//	  else
//	    {
//		  if(qian_ess>0)
//		  qian_ess=qian_ess-5;
//		  if(qian_ess<0)
//		  qian_ess=qian_ess+5;
//		}
//	
//	  if(qian_ess<-20)
//		  qian_ess=-20;
//	  if(qian_ess>20)
//		  qian_ess=20;
//	  
//	  qian_intre+=qian_ess;
//	  qian_eess=qian_ess-qian_ess1;
//	  qian_ess1=qian_ess;
//	  direct1=zhi.kp*qian_ess+zhi.ki*qian_intre+zhi.kd*qian_eess;	
//	  if(direct1>100)
//        direct1=100;
//	  if(direct1<-100)
//        direct1=-100;
//	}
//	else
//	{
//	  direct1=direct1;
//	}
	
	
	
	throttle1+=direct1*p_d/10;
	throttle2-=direct1*p_d/10;
	throttle3-=direct1*p_d/10;
	throttle4+=direct1*p_d/10;
}










































