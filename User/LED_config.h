#include "stm32f10x.h"


#define	digitalHi(p,i)			{p->BSRR=i;}			//����Ϊ�ߵ�ƽ		
#define digitalLo(p,i)			{p->BRR=i;}				//����͵�ƽ
#define digitalToggle(p,i)		{p->ODR ^=i;}			//�����ת״̬

#define LED0_TOGGLE		digitalToggle(GPIOA,GPIO_Pin_0)
#define LED0_OFF		digitalHi(GPIOA,GPIO_Pin_0)
#define LED0_ON			digitalLo(GPIOA,GPIO_Pin_0)

void LED_GPIO_Config(void);
