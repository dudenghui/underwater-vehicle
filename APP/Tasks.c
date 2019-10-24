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
函数原型：	void Nvic_Init(void)
功    能：	NVIC初始化
*******************************************************************************/ 
void Nvic_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//NVIC_PriorityGroup 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//优先级分组
    //Timer3
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//先占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//从占优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	//串口1中断
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//串口3中断
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//串口2中断
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//	//滴答定时器中断
//	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
	
}

/******************************************************************************
函数原型：	void BSP_Int(void)
功    能：	硬件驱动初始化
*******************************************************************************/ 
void BSP_Int(void)
{
	Nvic_Init();         //中断优先级初始化	
	delay_init();
	
	Uart1_Init(115200);  //串口1初始化：波特率115200，8位数据，1位停止位，禁用奇偶校验   接受数据
	Uart3_Init(115200);    //串口3初始化  陀螺仪加速度计
	Uart2_Init(115200);  //串口2初始化  电压模块
	
	Motor_Init();        //PWM初始化
	
	IIC_Init();          //深度计模块
	MS5803_PROM_Read();
	
	PID_Init();          //PID参数初始化
    Motor(0,0,0,0,0,0,0,0);   //电机驱动归中
	
	//delay_ms(500);
	//Timer3_Init(100);       //Timer3中断100HZ    定时器中断
	Bsp_Int_Ok = 1;
	depth_ctrl=0;
}

/******************************************************************************
函数原型：	void Task_100HZ(void)
功    能：	主循环中运行频率为100HZ任务
*******************************************************************************/ 
void Task_100HZ(void)
{
	Debug1_H;
	
	IMU_Get();
	Order_Get();
	MS5803_Pressure_ReadAndDMP();
	
	duoji_contro(duoji_c);      					      //舵机控制
	Heading_control(forward_c,20,side_c,20,direct_c,10);  //方向控制
    depth_control(depth_c,angle_c,4);                     //上下控制
	
	//Heading_control(0x70,10,0x70,10,0x80,10);           //电机测试  
	//depth_control(0x70,0x70,3); 
	
	if(start==0)
	{
	  Motor(0,0,0,0,0,0,0,0);	
	}
	if(start)
	{
	  Motor(throttle1,throttle2,throttle3,throttle4,throttle5,throttle6,duoji,0);
	}
	
	/*******************舵机控制****************/
	if(duoji>1900)	duoji=1900;
	if(duoji<1100)	duoji=1100;
	TIM3->CCR3 =duoji;            //A2
	
	TIM3->CCR4 = lightt*80000;          //A3
	delay_ms(10);
    Debug1_L;
}

/******************************************************************************
函数原型：	void Task_20HZ(void)
功    能：	主循环中运行频率为20HZ任务
*******************************************************************************/ 
void Task_20HZ(void)
{
	Debug2_H;

	//Order_Get();

	
	Debug2_L;
}


/******************************************************************************
函数原型：	void duoji_control(uint16_t ss)
功    能：	设置PID参数
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
	
	if(duoji>1900)  //即5s即可完成从最大极限到最小极限的调整
		duoji=1900;
	if(duoji<1100)
		duoji=1100;
}




