#include "IMU.h"
#include "myiic.h"
#include "Tasks.h"
#include "Uart.h" 
#include "Control.h"
#include "Timer3.h"
#include "Motor.h"
#include "Protocol.h"
#include "delay.h"
#include "myiic.h"

uint8_t Bsp_Int_Ok = 0;
float duoji,duoji_text,light_c;
uint16_t depth_c,angle_c,forward_c,side_c,direct_c,direct_c1,direct_c2,duoji_c;
uint16_t ok,start;

uint16_t depth_ctrl=0;

uint16_t lightt;
void duoji_contro(uint16_t ss);
/******************************************************************************
����ԭ�ͣ�	void Nvic_Init(void)
��    �ܣ�	NVIC��ʼ��
*******************************************************************************/ 
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//NVIC_PriorityGroup 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//���ȼ�����
    //Timer3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	//����1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//����3�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//����2�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//	//�δ�ʱ���ж�
//	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	
}

/******************************************************************************
����ԭ�ͣ�	void BSP_Int(void)
��    �ܣ�	Ӳ��������ʼ��
*******************************************************************************/ 
void BSP_Int(void)
{
	Nvic_Init();         //�ж����ȼ���ʼ��	
	delay_init();
	
	Uart1_Init(115200);  //����1��ʼ����������115200��8λ���ݣ�1λֹͣλ��������żУ��   ��������
	Uart3_Init(115200);    //����3��ʼ��  �����Ǽ��ٶȼ�
	Uart2_Init(115200);  //����2��ʼ��  ��ѹģ��
	
	Motor_Init();        //PWM��ʼ��
	
	IIC_Init();          //��ȼ�ģ��
	MS5803_PROM_Read();
	
	PID_Init();          //PID������ʼ��
    Motor(0,0,0,0,0,0,0,0);   //�����������
	
	//delay_ms(500);
	//Timer3_Init(100);       //Timer3�ж�100HZ    ��ʱ���ж�
	Bsp_Int_Ok = 1;
	depth_ctrl=0;
}

/******************************************************************************
����ԭ�ͣ�	void Task_100HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ100HZ����
*******************************************************************************/ 
void Task_100HZ(void)
{
	Debug1_H;
	
	IMU_Get();
	Order_Get();
	MS5803_Pressure_ReadAndDMP();
	
	duoji_contro(duoji_c);      					      //�������
	Heading_control(forward_c,20,side_c,20,direct_c,10);  //�������
    depth_control(depth_c,angle_c,4);                     //���¿���
	
	//Heading_control(0x70,10,0x70,10,0x80,10);           //�������  
	//depth_control(0x70,0x70,3); 
	
	if(start==0)
	{
	  Motor(0,0,0,0,0,0,0,0);	
	}
	if(start)
	{
	  Motor(throttle1,throttle2,throttle3,throttle4,throttle5,throttle6,duoji,0);
	}
	
	/*******************�������****************/
	if(duoji>1900)	duoji=1900;
	if(duoji<1100)	duoji=1100;
	TIM3->CCR3 =duoji;            //A2
	
	TIM3->CCR4 = lightt*80000;          //A3
	delay_ms(10);
    Debug1_L;
}

/******************************************************************************
����ԭ�ͣ�	void Task_20HZ(void)
��    �ܣ�	��ѭ��������Ƶ��Ϊ20HZ����
*******************************************************************************/ 
void Task_20HZ(void)
{
	Debug2_H;

	//Order_Get();

	
	Debug2_L;
}


/******************************************************************************
����ԭ�ͣ�	void duoji_control(uint16_t ss)
��    �ܣ�	����PID����
*******************************************************************************/ 
void duoji_contro(uint16_t ss)
{
  int16_t data_c;
	if(ss==2)
		data_c=4;
	else if(ss==1)
		data_c=-4;
	else
		data_c=0;
	
	duoji=duoji+data_c;
	//duoji=duoji/2;
	
	if(duoji>1900)  //��5s������ɴ�����޵���С���޵ĵ���
		duoji=1900;
	if(duoji<1100)
		duoji=1100;
}




