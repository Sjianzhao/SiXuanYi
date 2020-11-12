/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * �ļ���  : UltrasonicWave.c
 * ����    �����������ģ�飬UltrasonicWave_Configuration��������
             ��ʼ������ģ�飬UltrasonicWave_StartMeasure��������
			 ������࣬������õ�����ͨ������1��ӡ����         
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�------------------
 *          | PE0  - TRIG      |
 *          | PE1  - ECHO      |
 *           ------------------
 * ��汾  ��ST3.5.0
 *
 * ����    ��wildfire team 
 * ��̳    ��http://www.amobbs.com/forum-1008-1.html
 * �Ա�    ��http://firestm32.taobao.com
*********************************************************************************/
#include "UltrasonicWave.h"
//#include "bsp_usart1.h"
#include "GPIO_Config.h"
#include "led_pwm.h"
#include "stm32f10x_tim.h"

#define	TRIG_PORT      GPIOB		//TRIG      	������1
#define	ECHO_PORT      GPIOB		//ECHO 			������1
#define	TRIG_PIN       GPIO_Pin_0   //TRIG       	������1
#define	ECHO_PIN       GPIO_Pin_1	//ECHO   		������1

//#define	TRIG_PORT2      GPIOB		//TRIG       	������2
//#define	ECHO_PORT2      GPIOB		//ECHO 			������2
//#define	TRIG_PIN2       GPIO_Pin_2   //TRIG       	������2
//#define	ECHO_PIN2       GPIO_Pin_3	//ECHO 			������2

//#define	TRIG_PORT3      GPIOB		//TRIG       	������3
//#define	ECHO_PORT3      GPIOB		//ECHO 			������3
//#define	TRIG_PIN3       GPIO_Pin_4   //TRIG       	������3
//#define	ECHO_PIN3       GPIO_Pin_5	//ECHO 			������3

unsigned short int Distance;      //������ľ��� 
unsigned short int UltrasonicWave_Distance;      //������ľ���
u8 Ultrasonic=0,Ultrasonic_OK=0;//��ʼ���ɼ���ɱ�־λ
char U_num;//�򿪵ĺ���
s32 u_time;//��ʱ

/*
 * ��������DelayTime_us
 * ����  ��1us��ʱ����
 * ����  ��Time   	��ʱ��ʱ�� US
 * ���  ����	
 */
void DelayTime_us(int Time)    
{
   unsigned char i;
   for ( ; Time>0; Time--)
     for ( i = 0; i < 72; i++ );
}

/*
 * ��������UltrasonicWave_Configuration
 * ����  ��������ģ��ĳ�ʼ��
 * ����  ����
 * ���  ����	
 */
void UltrasonicWave_Configuration(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;	
						 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			
		GPIO_InitStructure.GPIO_Pin = TRIG_PIN;//|TRIG_PIN2;//;|TRIG_PIN3;					 //PC8��TRIG
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		     //��Ϊ�������ģʽ
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
		GPIO_Init(TRIG_PORT,&GPIO_InitStructure);	                 //��ʼ������GPIO 

//		GPIO_InitStructure.GPIO_Pin = ECHO_PIN;//|ECHO_PIN2|ECHO_PIN3;				     //PC9��ECH0
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		 //��Ϊ����
//		GPIO_Init(ECHO_PORT,&GPIO_InitStructure);						 //��ʼ��GPIOC
}

/*
 * ��������UltrasonicWave_CalculateTime
 * ����  ���������
 * ����  ����
 * ���  ����	
 */
void UltrasonicWave_CalculateTime(void)
{
   UltrasonicWave_Distance=TIM_GetCounter(TIM7)*5*34/2000;
}

/*
 * ��������UltrasonicWave_StartMeasure
 * ����  ����ʼ��࣬����һ��>10us�����壬Ȼ��������صĸߵ�ƽʱ��
 * ����  ����
 * ���  ����	
 */
void UltrasonicWave_StartMeasure(void)
{
//	if(!Ultrasonic)  //û�г������ڹ���
	if(!Ultrasonic_OK)  //û�г������ڹ���
	{
		GPIO_SetBits(TRIG_PORT,TRIG_PIN); 		  //��>10US�ĸߵ�ƽ
		DelayTime_us(20);		                      //��ʱ20US
		GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
  }
}

void UltrasonicWave_OK(void)
{	
	if(Ultrasonic_OK)
	{
		UltrasonicWave_CalculateTime();									 //ȡʱ��
		TIM_SetCounter(TIM7,0);
		Distance=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//�������ľ���
	  Ultrasonic_OK=0;//�������
//		printf("1�ž���Ϊ:%d cm\t",Distance);

		
	}
}
//void UltrasonicWave_StartMeasure(void)
//{
//	int i=0;
//  GPIO_SetBits(TRIG_PORT,TRIG_PIN); 		  //��>10US�ĸߵ�ƽ
//  DelayTime_us(20);		                      //��ʱ20US
//  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
//  
//  while(!GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN));	             //�ȴ��ߵ�ƽ
//  TIM_Cmd(TIM2, ENABLE);                                             //����ʱ��
//  while(GPIO_ReadInputDataBit(ECHO_PORT,ECHO_PIN)&&(i<0x7afff))
//  i++;	                 //�ȴ��͵�ƽ
//  TIM_Cmd(TIM2, DISABLE);			                                 //��ʱ��2ʧ��
//  UltrasonicWave_CalculateTime();									 //�������
//  TIM_SetCounter(TIM2,0);
////	
//	Distance=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//�������ľ���
////  printf("\r\ndistance:%d%d cm\r\n",UltrasonicWave_Distance/256,UltrasonicWave_Distance%256);	
////	distance=Distance; 
//	printf("\r���:%d cm\r\t",Distance);
//  	  
//}

//void UltrasonicWave_StartMeasure_2(void)
//{
//	int i=0;
//  GPIO_SetBits(TRIG_PORT2,TRIG_PIN2); 		  //��>10US�ĸߵ�ƽ
//  DelayTime_us(20);		                      //��ʱ20US
//  GPIO_ResetBits(TRIG_PORT2,TRIG_PIN2);
//  
//  while(!GPIO_ReadInputDataBit(ECHO_PORT2,ECHO_PIN2));	             //�ȴ��ߵ�ƽ
//  TIM_Cmd(TIM2, ENABLE);                                             //����ʱ��
//  while(GPIO_ReadInputDataBit(ECHO_PORT2,ECHO_PIN2)&&(i<0x7af))
//  i++;	                 //�ȴ��͵�ƽ
//  TIM_Cmd(TIM2, DISABLE);			                                 //��ʱ��2ʧ��
//  UltrasonicWave_CalculateTime();									 //�������
//  TIM_SetCounter(TIM2,0);
//	
//	Distance2=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//�������ľ���
//	//printf("\r\ndistance:%d%d cm\r\n",UltrasonicWave_Distance/256,UltrasonicWave_Distance%256);	
//	distance2=Distance2; 
//	printf("\rǰ��:%d cm\r\t",Distance2);
//  	  
//}

//void UltrasonicWave_StartMeasure_3(void)
//{
//	int i=0;
//  GPIO_SetBits(TRIG_PORT3,TRIG_PIN3); 		  //��>10US�ĸߵ�ƽ
//  DelayTime_us(20);		                      //��ʱ20US
//  GPIO_ResetBits(TRIG_PORT3,TRIG_PIN3);
//  
//  while(!GPIO_ReadInputDataBit(ECHO_PORT3,ECHO_PIN3));	             //�ȴ��ߵ�ƽ
//  TIM_Cmd(TIM2, ENABLE);                                             //����ʱ��
//  while(GPIO_ReadInputDataBit(ECHO_PORT3,ECHO_PIN3)&&(i<0x7af))
//  i++;	                 //�ȴ��͵�ƽ
//  TIM_Cmd(TIM2, DISABLE);			                                 //��ʱ��2ʧ��
//  UltrasonicWave_CalculateTime();									 //�������
//  TIM_SetCounter(TIM2,0);
//	
//	Distance3=(UltrasonicWave_Distance%256)+100*(UltrasonicWave_Distance/256);	//�������ľ���
//	//printf("\r\ndistance:%d%d cm\r\n",UltrasonicWave_Distance/256,UltrasonicWave_Distance%256);	
//	distance3=Distance3; 
//	printf("\r�ұ�:%d cm\r\t",Distance3);
//  	  
//}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
