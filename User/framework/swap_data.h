#include "stm32f10x.h"
#include "GPIO_Config.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	

void Data_Send_Status(void);//��̬�ش�
void sent_data_com(void);//ʵʱ����
void send_line(int16_t b1,int16_t b2,int16_t b3,int16_t b4,int16_t b5,int16_t b6);//ɽ����λ��
