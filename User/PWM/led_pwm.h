#include "stm32f10x.h" 

static void TIM3_GPIO_Config(void);
void TIM2_init(void);
void TIM2_Mode_PWM1_Config(u32 CCR1_Val);
void TIM2_Mode_PWM2_Config(u32 CCR1_Val);
void TIM2_Mode_PWM3_Config(u32 CCR1_Val);
void TIM2_Mode_PWM4_Config(u32 CCR1_Val);
