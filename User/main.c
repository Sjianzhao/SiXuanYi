/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   用3.5.0版本库建的工程模板
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
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

#include "init_set.h"//设置
#include "data_exchang.h"// 数据无线交换
#include "LED_config.h"
#include "FLASH_RW.h"
#include "AHRS.h"



#define CLI()      __set_PRIMASK(1)		/* 关闭总中断 */  
#define SEI() __set_PRIMASK(0)				/* 开放总中断 */



//红外****************
extern uint8_t  frame_flag;
extern uint8_t  isr_cnt;
extern uint8_t  frame_cnt;

extern __IO uint16_t ADC_ConvertedValue;

// 局部变量，用于保存转换计算后的电压值 	 
double ADC_ConvertedValueLocal; 
extern uint8_t AGM_START;//角度采集标志位

char num=0;//计算一段时间内接收才次数

__IO uint32_t DeviceID = 0;
extern uchar  nrf_sta;
extern uint8_t SendBuff[SENDBUFF_SIZE];//待发送数据  利用DMA发送
volatile u32 time = 0,times=0; // ms 计时变量
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
//	    uint8_t key_val;
	
			CLI();
			SEI();
	
	    SysTick_Init();       //开启滴答
			NVIC_SetPriority (SysTick_IRQn, 0);
			USART1_Config();  		//串口初始化
			USART1_DMA_Config(); 	//串口DMA配置
	
//			USART3_Config();
	
//			USART_SendData(USART3, 0x12);
//			while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	     
			SPI_DATA_Init();			//硬件spi初始化
//	  I2C_EE_Init();    		//硬件iic 芯片设计缺陷导致容易卡死
	    gpio_i2c();           //模拟iic 
	
			MPU6050_Init();       //陀螺仪及地磁传感器初始化
			BMP280_Init();				//初始化气压传感器
	    init_NRF24L01();			//初始化2.4G		
			
			LED_GPIO_Config();			
					
				
			
			OLED_Init();			   //初始化OLED  
									
//			TIM2_init();
			
			TIM3_init();
//			TIM3_Mode_PWM1_Config(1950);
			 //延时两秒钟
			ADC1_Init();
			
      EXTI_PB1_Config();	//初始化收发中断引脚 		
			TIM7_Configuration();		//定时器2初始配置
			UltrasonicWave_Configuration();//超声波模块引脚初始化	
			
			
			
			Flash_data_init();//校准数据初始化
			
			TIM3->CCR1=3900;
			TIM3->CCR2=3900;
			TIM3->CCR3=3900;
			TIM3->CCR4=3900;
			Delay_ms(3000);  //等待3秒 哔-哔-油门最高确认
      TIM3->CCR1=1950;
			TIM3->CCR2=1950;
			TIM3->CCR3=1950;
			TIM3->CCR4=1950;
			Delay_ms(4000); //电池节数，哔——油门最低点确认
//			while(1);
//			TIM3_Mode_PWM1_Config(1950);
//			TIM3_Mode_PWM2_Config(1950);
//			TIM3_Mode_PWM3_Config(1950);
//			TIM3_Mode_PWM4_Config(1950);
//			Delay_ms(5000);
//      while(1);
//			init_quaternion();
//			
			OLED_Clear();        //清屏
			OLED_ShowCHinese(40,3,21);//设 
			OLED_ShowCHinese(72,3,22);//置
			
			AGM_START=1; //第一次运行时候要计算			
			TIM6_Configuration();	// TIM6 定时配置 		
			TIM6_NVIC_Configuration(); /* 实战定时器的中断优先级 */	
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE); /* TIM6 重新开时钟，开始计时 */ 
			
			YAW_data_init();//初始化数据
			
			IrDa_Init();//红外初始化
			 while(1) 
			{ 
				set_data();
				
        data_init();//读取传感器数据  
				
//				change();	//互补滤波
				
				data_chang();//发送及接受
//				LED0_TOGGLE;
				
			}
}			
								
//				LED0_TOGGLE;
				

/*********************************************END OF FILE**********************/

