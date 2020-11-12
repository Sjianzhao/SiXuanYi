#include "LED_config.h"

void LED_GPIO_Config(void)
{		
			/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*开启GPIOB和GPIOB的外设时钟*/
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

		/*选择要控制的GPIOB引脚*/															   
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	

		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIOB0*/
		GPIO_Init(GPIOA, &GPIO_InitStructure);			
				  
		/* 关闭所有led灯	*/
		GPIO_SetBits(GPIOA, GPIO_Pin_0);	 
}
