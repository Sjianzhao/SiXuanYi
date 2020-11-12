#include "stm32f10x.h"


#define	digitalHi(p,i)			{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)			{p->BRR=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态

#define LED0_TOGGLE		digitalToggle(GPIOA,GPIO_Pin_0)
#define LED0_OFF		digitalHi(GPIOA,GPIO_Pin_0)
#define LED0_ON			digitalLo(GPIOA,GPIO_Pin_0)

void LED_GPIO_Config(void);
