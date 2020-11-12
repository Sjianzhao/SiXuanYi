/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ��3.5.0�汾�⽨�Ĺ���ģ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� iSO STM32 ������ 
  * ��̳    :http://www.chuxue123.com
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f10x.h"
#include "GPIO_Config.h"
#include "nR24L01_Reg.h"
#include "bsp_SysTick.h"
#include "bsp_TiMbase.h"
#include "bsp_exti.h"
#include "pwm_config.h"
#include "oled.h"

#include "bsp_i2c.h"
#include "i2c.h"
#include "mpu6050.h"
#include "BMP_280.h"
#include "ADC_DMA.h"

#include "TIM_time.h"

#include "led_pwm.h"
#include "UltrasonicWave.h"

#include "irda.h"

#include "init_set.h"//����
#include "data_exchang.h"// �������߽���
#include "LED_config.h"
#include "FLASH_RW.h"
#include "AHRS.h"



#define CLI()      __set_PRIMASK(1)		/* �ر����ж� */  
#define SEI() __set_PRIMASK(0)				/* �������ж� */



//����****************
extern uint8_t  frame_flag;
extern uint8_t  isr_cnt;
extern uint8_t  frame_cnt;

extern __IO uint16_t ADC_ConvertedValue;

// �ֲ����������ڱ���ת�������ĵ�ѹֵ 	 
double ADC_ConvertedValueLocal; 
extern uint8_t AGM_START;//�ǶȲɼ���־λ

char num=0;//����һ��ʱ���ڽ��ղŴ���

__IO uint32_t DeviceID = 0;
extern uchar  nrf_sta;
extern uint8_t SendBuff[SENDBUFF_SIZE];//����������  ����DMA����
volatile u32 time = 0,times=0; // ms ��ʱ����
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{	
//	    uint8_t key_val;
	
			CLI();
			SEI();
	
	    SysTick_Init();       //�����δ�
			NVIC_SetPriority (SysTick_IRQn, 0);
			USART1_Config();  		//���ڳ�ʼ��
			USART1_DMA_Config(); 	//����DMA����
	
//			USART3_Config();
	
//			USART_SendData(USART3, 0x12);
//			while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	     
			SPI_DATA_Init();			//Ӳ��spi��ʼ��
//	  I2C_EE_Init();    		//Ӳ��iic оƬ���ȱ�ݵ������׿���
	    gpio_i2c();           //ģ��iic 
	
			MPU6050_Init();       //�����Ǽ��شŴ�������ʼ��
			BMP280_Init();				//��ʼ����ѹ������
	    init_NRF24L01();			//��ʼ��2.4G		
			
			LED_GPIO_Config();			
					
				
			
			OLED_Init();			   //��ʼ��OLED  
									
//			TIM2_init();
			
			TIM3_init();
//			TIM3_Mode_PWM1_Config(1950);
			 //��ʱ������
			ADC1_Init();
			
      EXTI_PB1_Config();	//��ʼ���շ��ж����� 		
			TIM7_Configuration();		//��ʱ��2��ʼ����
			UltrasonicWave_Configuration();//������ģ�����ų�ʼ��	
			
			
			
			Flash_data_init();//У׼���ݳ�ʼ��
			
			TIM3->CCR1=3900;
			TIM3->CCR2=3900;
			TIM3->CCR3=3900;
			TIM3->CCR4=3900;
			Delay_ms(3000);  //�ȴ�3�� ��-��-�������ȷ��
      TIM3->CCR1=1950;
			TIM3->CCR2=1950;
			TIM3->CCR3=1950;
			TIM3->CCR4=1950;
			Delay_ms(4000); //��ؽ������١���������͵�ȷ��
//			while(1);
//			TIM3_Mode_PWM1_Config(1950);
//			TIM3_Mode_PWM2_Config(1950);
//			TIM3_Mode_PWM3_Config(1950);
//			TIM3_Mode_PWM4_Config(1950);
//			Delay_ms(5000);
//      while(1);
//			init_quaternion();
//			
			OLED_Clear();        //����
			OLED_ShowCHinese(40,3,21);//�� 
			OLED_ShowCHinese(72,3,22);//��
			
			AGM_START=1; //��һ������ʱ��Ҫ����			
			TIM6_Configuration();	// TIM6 ��ʱ���� 		
			TIM6_NVIC_Configuration(); /* ʵս��ʱ�����ж����ȼ� */	
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE); /* TIM6 ���¿�ʱ�ӣ���ʼ��ʱ */ 
			
			YAW_data_init();//��ʼ������
			
			IrDa_Init();//�����ʼ��
			 while(1) 
			{ 
				set_data();
				
        data_init();//��ȡ����������  
				
//				change();	//�����˲�
				
				data_chang();//���ͼ�����
//				LED0_TOGGLE;
				
			}
}			
								
//				LED0_TOGGLE;
				

/*********************************************END OF FILE**********************/

