/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * 文件名  : UltrasonicWave.c
 * 描述    ：超声波测距模块，UltrasonicWave_Configuration（）函数
             初始化超声模块，UltrasonicWave_StartMeasure（）函数
			 启动测距，并将测得的数据通过串口1打印出来         
 * 实验平台：野火STM32开发板
 * 硬件连接：------------------
 *          | PE0  - TRIG      |
 *          | PE1  - ECHO      |
 *           ------------------
 * 库版本  ：ST3.5.0
 *
 * 作者    ：wildfire team 
 * 论坛    ：http://www.amobbs.com/forum-1008-1.html
 * 淘宝    ：http://firestm32.taobao.com
*********************************************************************************/
#include "UltrasonicWave.h"
//#include "bsp_usart1.h"
#include "GPIO_Config.h"
#include "led_pwm.h"
#include "stm32f10x_tim.h"

#define	TRIG_PORT      GPIOB		//TRIG      	超声波1
#define	ECHO_PORT      GPIOB		//ECHO 			超声波1
#define	TRIG_PIN       GPIO_Pin_0   //TRIG       	超声波1
#define	ECHO_PIN       GPIO_Pin_1	//ECHO   		超声波1

//#define	TRIG_PORT2      GPIOB		//TRIG       	超声波2
//#define	ECHO_PORT2      GPIOB		//ECHO 			超声波2
//#define	TRIG_PIN2       GPIO_Pin_2   //TRIG       	超声波2
//#define	ECHO_PIN2       GPIO_Pin_3	//ECHO 			超声波2

//#define	TRIG_PORT3      GPIOB		//TRIG       	超声波3
//#define	ECHO_PORT3      GPIOB		//ECHO 			超声波3
//#define	TRIG_PIN3       GPIO_Pin_4   //TRIG       	超声波3
//#define	ECHO_PIN3       GPIO_Pin_5	//ECHO 			超声波3

unsigned short int Distance;      //计算出的距离 
unsigned short int UltrasonicWave_Distance;      //计算出的距离
u8 Ultrasonic=0,Ultrasonic_OK=0;//开始，采集完成标志位
char U_num;//打开的号码
s32 u_time;//超时

/*
 * 函数名：DelayTime_us
 * 描述  ：1us延时函数
 * 输入  ：Time   	延时的时间 US
 * 输出  ：无	
 */
void DelayTime_us(int Time)    
{
   unsigned char i;
   for ( ; Time>0; Time--)
     for ( i = 0; i < 72; i++ );
}

/*
 * 函数名：UltrasonicWave_Configuration
 * 描述  ：超声波模块的初始化
 * 输入  ：无
 * 输出  ：无	
 */
void UltrasonicWave_Configuration(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	
						 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			
		GPIO_InitStructure.GPIO_Pin = TRIG_PIN;//|TRIG_PIN2;//;|TRIG_PIN3;					 //PC8接TRIG
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		     //设为推挽输出模式
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
		GPIO_Init(TRIG_PORT,&GPIO_InitStructure);	                 //初始化外设GPIO 

//		GPIO_InitStructure.GPIO_Pin = ECHO_PIN;//|ECHO_PIN2|ECHO_PIN3;				     //PC9接ECH0
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		 //设为输入
//		GPIO_Init(ECHO_PORT,&GPIO_InitStructure);						 //初始化GPIOC
}

/*
 * 函数名：UltrasonicWave_CalculateTime
 * 描述  ：计算距离
 * 输入  ：无
 * 输出  ：无	
 */
void UltrasonicWave_CalculateTime(void)
{
   UltrasonicWave_Distance=TIM_GetCounter(TIM7)*5*34/2000;
}

/*
 * 函数名：UltrasonicWave_StartMeasure
 * 描述  ：开始测距，发送一个>10us的脉冲，然后测量返回的高电平时间
 * 输入  ：无
 * 输出  ：无	
 */
void UltrasonicWave_StartMeasure(void)
{
//	if(!Ultrasonic)  //没有超声波在工作
	if(!Ultrasonic_OK)  //没有超声波在工作
	{
		GPIO_SetBits(TRIG_PORT,TRIG_PIN); 		  //送>10US的高电平
		DelayTime_us(20);		                      //延时20US
		GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
  }
}

void UltrasonicWave_OK(void)
{	
	if(Ultrasonic_OK)
	{
		UltrasonicWave_CalculateTime();									 //取时间
		TIM_SetCounter(TIM7,0);
		Distance=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//超声波的距离
	  Ultrasonic_OK=0;//计算完成
//		printf("1号距离为:%d cm\t",Distance);

		
	}
}
//void UltrasonicWave_StartMeasure(void)
//{
//	int i=0;
//  GPIO_SetBits(TRIG_PORT,TRIG_PIN); 		  //送>10US的高电平
//  DelayTime_us(20);		                      //延时20US
//  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
//  
//  while(!GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN));	             //等待高电平
//  TIM_Cmd(TIM2, ENABLE);                                             //开启时钟
//  while(GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN)&&(i<0x7afff))
//  i++;	                 //等待低电平
//  TIM_Cmd(TIM2, DISABLE);			                                 //定时器2失能
//  UltrasonicWave_CalculateTime();									 //计算距离
//  TIM_SetCounter(TIM2,0);
////	
//	Distance=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//超声波的距离
////  printf("\r\ndistance:%d%d cm\r\n",UltrasonicWave_Distance/256,UltrasonicWave_Distance%256);	
////	distance=Distance; 
//	printf("\r左边:%d cm\r\t",Distance);
//  	  
//}

//void UltrasonicWave_StartMeasure_2(void)
//{
//	int i=0;
//  GPIO_SetBits(TRIG_PORT2,TRIG_PIN2); 		  //送>10US的高电平
//  DelayTime_us(20);		                      //延时20US
//  GPIO_ResetBits(TRIG_PORT2,TRIG_PIN2);
//  
//  while(!GPIO_ReadInputDataBit(ECHO_PORT2,ECHO_PIN2));	             //等待高电平
//  TIM_Cmd(TIM2, ENABLE);                                             //开启时钟
//  while(GPIO_ReadInputDataBit(ECHO_PORT2,ECHO_PIN2)&&(i<0x7af))
//  i++;	                 //等待低电平
//  TIM_Cmd(TIM2, DISABLE);			                                 //定时器2失能
//  UltrasonicWave_CalculateTime();									 //计算距离
//  TIM_SetCounter(TIM2,0);
//	
//	Distance2=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//超声波的距离
//	//printf("\r\ndistance:%d%d cm\r\n",UltrasonicWave_Distance/256,UltrasonicWave_Distance%256);	
//	distance2=Distance2; 
//	printf("\r前面:%d cm\r\t",Distance2);
//  	  
//}

//void UltrasonicWave_StartMeasure_3(void)
//{
//	int i=0;
//  GPIO_SetBits(TRIG_PORT3,TRIG_PIN3); 		  //送>10US的高电平
//  DelayTime_us(20);		                      //延时20US
//  GPIO_ResetBits(TRIG_PORT3,TRIG_PIN3);
//  
//  while(!GPIO_ReadInputDataBit(ECHO_PORT3,ECHO_PIN3));	             //等待高电平
//  TIM_Cmd(TIM2, ENABLE);                                             //开启时钟
//  while(GPIO_ReadInputDataBit(ECHO_PORT3,ECHO_PIN3)&&(i<0x7af))
//  i++;	                 //等待低电平
//  TIM_Cmd(TIM2, DISABLE);			                                 //定时器2失能
//  UltrasonicWave_CalculateTime();									 //计算距离
//  TIM_SetCounter(TIM2,0);
//	
//	Distance3=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//超声波的距离
//	//printf("\r\ndistance:%d%d cm\r\n",UltrasonicWave_Distance/256,UltrasonicWave_Distance%256);	
//	distance3=Distance3; 
//	printf("\r右边:%d cm\r\t",Distance3);
//  	  
//}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
