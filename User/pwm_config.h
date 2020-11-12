#include "stm32f10x.h" 

static void TIM3_GPIO_Config(void);
void TIM3_init(void);
void TIM3_Mode_PWM1_Config(u32 CCR1_Val);
void TIM3_Mode_PWM2_Config(u32 CCR1_Val);
void TIM3_Mode_PWM3_Config(u32 CCR1_Val);
void TIM3_Mode_PWM4_Config(u32 CCR1_Val);
