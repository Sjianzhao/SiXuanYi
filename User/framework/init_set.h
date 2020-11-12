#include "stm32f10x.h"
#include "irda.h"
#include "oled.h"
#include "bsp_SysTick.h"

void set_data(void);					//主菜单
void MENU_1(void);						//子一级菜单 1
void MENU_2(void);						//子一级菜单 2
void MENU_3(void);						//子一级菜单 3

void MENU_1_1(void);          //子二级菜单 1-1  //内环PID设置
void MENU_1_2(void);          //子二级菜单 1-2  //外环PID设置
void MENU_1_3(void);          //子二级菜单 1-3  //YAWPID设置

void MENU_1_3_1(void);          //子二级菜单 1-3-1  //YAWPID内环设置
void MENU_1_3_2(void);          //子二级菜单 1-3-2  //YAWPID外环设置
               
void MENU_3_1(void);          //子二级菜单 3-1
void MENU_3_2(void);          //子二级菜单 3-2

void complete_data(void);			//子三级菜单 3-1-1 修正完成 是否保存数据
void complete_GA_data(void);  //子三级菜单 3-2-2 修正完成 是否保存数据

#define   RED_Error 	  0   	//错误

#define   RED_POWER  		162 	//电源
#define   RED_MENU 			226 	//菜单
#define   RED_TEST 			34  	//测试
#define   RED_RETURN 		194 	//返回

#define   RED_Rise 			2			//上升
#define   RED_Left 		  224 	//左边
#define   RED_OK        168
#define   RED_Right     144		//右边
#define   RED_Down      152		//下降

#define   RED_C         176 	//清空
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
