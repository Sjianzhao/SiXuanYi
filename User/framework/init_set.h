#include "stm32f10x.h"
#include "irda.h"
#include "oled.h"
#include "bsp_SysTick.h"

void set_data(void);					//���˵�
void MENU_1(void);						//��һ���˵� 1
void MENU_2(void);						//��һ���˵� 2
void MENU_3(void);						//��һ���˵� 3

void MENU_1_1(void);          //�Ӷ����˵� 1-1  //�ڻ�PID����
void MENU_1_2(void);          //�Ӷ����˵� 1-2  //�⻷PID����
void MENU_1_3(void);          //�Ӷ����˵� 1-3  //YAWPID����

void MENU_1_3_1(void);          //�Ӷ����˵� 1-3-1  //YAWPID�ڻ�����
void MENU_1_3_2(void);          //�Ӷ����˵� 1-3-2  //YAWPID�⻷����
               
void MENU_3_1(void);          //�Ӷ����˵� 3-1
void MENU_3_2(void);          //�Ӷ����˵� 3-2

void complete_data(void);			//�������˵� 3-1-1 ������� �Ƿ񱣴�����
void complete_GA_data(void);  //�������˵� 3-2-2 ������� �Ƿ񱣴�����

#define   RED_Error 	  0   	//����

#define   RED_POWER  		162 	//��Դ
#define   RED_MENU 			226 	//�˵�
#define   RED_TEST 			34  	//����
#define   RED_RETURN 		194 	//����

#define   RED_Rise 			2			//����
#define   RED_Left 		  224 	//���
#define   RED_OK        168
#define   RED_Right     144		//�ұ�
#define   RED_Down      152		//�½�

#define   RED_C         176 	//���
#define   RED_0         104
#define   RED_1         48
#define   RED_2         24
#define   RED_3         122
#define   RED_4         16
#define   RED_5         56
#define   RED_6         90
#define   RED_7         66
#define   RED_8         74
#define   RED_9         82
