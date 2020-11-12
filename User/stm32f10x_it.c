/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_SysTick.h"
#include "GPIO_Config.h"
#include "nR24L01_Reg.h"
#include "irda.h"
#include "LED_config.h"
#include "mpu6050.h"
#include "BMP_280.h"
#include "PID_control.h"

char rx_num;//������յĴ���
extern void TimingDelay_Decrement(void);
extern char num;//����һ��ʱ���ڽ��ղŴ���
extern volatile u32 time,times;
extern u8 Ultrasonic,Ultrasonic_OK;


extern uint32_t frame_data;
extern uint8_t  frame_cnt;
extern uint8_t  frame_flag;

uint8_t isr_cnt;  /* ���ڼ�����˶��ٴ��ж� */
uint16_t TIME_AGM=0,TIME_PID=0;//�ɼ�ʱ���Լ�PID ��ʱ��



uint8_t DATA_START;//��ͨ�ɼ���־λ
uint8_t PRESS_START;//��ѹ�ɼ���־λ
uint8_t Ultrasonic_START;//�������ɼ���־λ
uint8_t AGM_START;//�ǶȲɼ���־λ
uint8_t TIME_20ms;//20ms��־λ

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM6 , TIM_IT_Update) != RESET ) 
	{	
//		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);
		
		TIME_AGM++;  //�����ǲɼ�ʱ��		
		if(TIME_AGM==5) //5ms�ж�һ��  �����Ǽ��ٶȵش�
		{
//			  PWM_control();//PWM�������
			
				TIME_AGM=0;
			
			  AGM_START=1;//�ɼ���־λ			
			
		}
		
		TIME_PID++;  //PID����ʱ��
		time++;
		
    if(time==18)
		{
			get_nrf_sta();				//��ȡ״̬��־
//			if(TX_DS)			//���ͳɹ�������ѭ��
//			{	
//			}			
			if(RX_DR == 1)				//���ճɹ�
			{	 
				nrf_read(RxBuf);		//���ճɹ��󣬽�NRF24L01���յ������ݶ�����Ƭ����RxBuf�����С�
				num++;//����
//				printf("ok\n");
			}
			nrf_TxMod();          //����ģʽ
			nrf_trans(TxBuf);			//�������͵�����д��NRF24L01
		}
		else if(time==20)//20ms
		{
			TIME_20ms=1;
//			PID_control();//PID����
			PWM_control();//PWM�������
			
			
			nrf_RxMod();          //����ģʽ
			time=0;
			times++;
		
			if(times==25)//500ms��������ܵĴ���
			{
				times=0;
				rx_num=num;
	//			printf("һ���ӽ��� %d ��\n",num);			
				 num=0;
	//			LED0_TOGGLE;
			}			
		}
		
		if(TIME_PID==50) //100ms�ж�һ�� ��ѹ ������ PID
		{
				PRESS_START=1; //��ѹ�򿪲���
						  			
		}
		else if(TIME_PID==100) //100ms�ж�һ�� ��ѹ ������ PID
		{
				TIME_PID=0;
			
			  DATA_START=1;//����ͨ���ݲɼ�	
		}
		
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);  		 
	}		 	
}

void EXTI1_IRQHandler(void)
{ 
	if(EXTI_GetITStatus(EXTI_Line1) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1))  //1�ų������������ź�
		{
				TIM_Cmd(TIM7, ENABLE);  //������ʱ��
//				Ultrasonic=1; //��������־ ��ֹ�ٴ�����
				Ultrasonic_OK=0;//��࿪ʼ
//			printf("��\n");
		}	
		else
		{
			  TIM_Cmd(TIM7, DISABLE);   //ֹͣ��ʱ��
				Ultrasonic_OK=1;//������   �Ѿ���ɲ��
//				Ultrasonic=0;//�رղ���־  ������Կ�ʼ�´β��
//			printf("��/n");
		}
		
		EXTI_ClearITPendingBit(EXTI_Line1);     //����жϱ�־λ
	 } 
}

// IO ���жϣ��жϿ�ΪPB11 �Ӻ������ͷ�����ݹܽ�
void EXTI15_10_IRQHandler(void)
{
	uint8_t pulse_time = 0;
  uint8_t leader_code_flag = 0; /* �������־λ�������������ʱ����ʾһ֡���ݿ�ʼ */
  uint8_t irda_data = 0;        /* �����ݴ�λ */
  
  if(EXTI_GetITStatus(EXTI_Line15) != RESET) /* ȷ���Ƿ������EXTI Line�ж� */
	{   
    while(1)
    {
      if( IrDa_DATA_IN()== SET )        /* ֻ�����ߵ�ƽ��ʱ�� */
      {       
        pulse_time = Get_Pulse_Time();
        
        /* >=5ms ���������ź� �����ָ��Ż���������ʱ��Ҳ��break����while(1)ѭ�� */
        if( pulse_time >= 250 )                
        {
          break; /* ����while(1)ѭ�� */
        }
        
        if(pulse_time>=200 && pulse_time<250)         /* ���ǰ��λ 4ms~4.5ms */
        {
          leader_code_flag = 1;
        }
        else if(pulse_time>=10 && pulse_time<50)      /* 0.56ms: 0.2ms~1ms */
        {
          irda_data = 0;
        }
        else if(pulse_time>=50 && pulse_time<100)     /* 1.68ms��1ms~2ms */
        {
          irda_data =1 ; 
        }        
        else if( pulse_time>=100 && pulse_time<=200 ) /* 2.1ms��2ms~4ms */
        {/* �����룬�ڵڶ����жϳ��� */
          frame_flag = 1;               /* һ֡���ݽ������ */
          frame_cnt++;                  /* ����������1 */
          isr_cnt ++;                   /* ���ж�һ�μ�1 */
          break;                        /* ����while(1)ѭ�� */
        }
        
        if( leader_code_flag == 1 )
        {/* �ڵ�һ���ж������ */
          frame_data <<= 1;
          frame_data += irda_data;
          frame_cnt = 0;
          isr_cnt = 1;
        }
      }      
    }// while(1)   
		EXTI_ClearITPendingBit(EXTI_Line15);     //����жϱ�־λ
    //LED2_TOGGLE;
	}  
}
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
