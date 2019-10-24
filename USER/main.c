#include "stm32f10x.h"
#include "Timer3.h"
#include "IMU.h"
#include "Tasks.h"
#include "Control.h"
#include "Uart.h" 
#include "Motor.h"

extern u16 USART_RX_STA;
extern volatile uint8_t USART_TX_BUF[20];
extern uint8_t angle_f[10];
extern uint8_t com;

uint32_t text;
uint32_t t,len=15;
float energe;

int main(void)
{

	BSP_Int();	//底层驱动初始化
	
	while(1)
	{
//		if(Count_10ms>=1)
//		{	
//			Count_10ms = 0;
			Task_100HZ();
			text++;
//	      }
			
		if(text==10||com==1)
	 	{
			text=0;
			com=0;
			
			USART_TX_BUF[0]  = 0x68;
			USART_TX_BUF[1]  = angle_f[0];
			USART_TX_BUF[2]  = angle_f[1];
			USART_TX_BUF[3]  = angle_f[2];
			USART_TX_BUF[4]  = angle_f[3];
			USART_TX_BUF[5]  = angle_f[4];
			USART_TX_BUF[6]  = angle_f[5];
			USART_TX_BUF[7]  = angle_f[6];
			USART_TX_BUF[8]  = angle_f[7];
			USART_TX_BUF[9]  = angle_f[8];
      USART_TX_BUF[10] = 0x69;
			
			for(t=0;t<len;t++)
		  {
		    	USART_SendData(USART1, USART_TX_BUF[t]);//向串口1发送数据
		    	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		    }
      }
  	}
  }




