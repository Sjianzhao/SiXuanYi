#include "init_set.h"
#include "pwm_config.h"
#include "PID_control.h"


//红外****************
extern uint8_t  frame_flag;
extern uint8_t  isr_cnt;
extern uint8_t  frame_cnt;

extern uint8_t clean_value;//积分清零标志位

uint8_t key_val; //存储接收的数据
uint8_t screen;//显示标志位

extern int8_t ERR_STOP;     //倾斜过大

/************************/
/*********主菜单**********/
/************************/
uint8_t POWER_set=0;//电源设置标志位

void set_data(void)
{    	
		
		uint8_t mode_set;//模式选择
	
		
	
		if( frame_flag == 1 )      // 一帧红外数据接收完成 
		{
			  key_val = IrDa_Process();
				if(key_val==RED_POWER)
				{
					POWER_set=1;//电源设置打开
					mode_set=1;
					screen=0;//显示屏清零
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , DISABLE); /* TIM6 重新开时钟，开始计时 */
					TIM3_Mode_PWM1_Config(1950);
					TIM3_Mode_PWM2_Config(1950);
					TIM3_Mode_PWM3_Config(1950);
					TIM3_Mode_PWM4_Config(1950);
					
				}					
				while(POWER_set)//POWER 
				{
					
					if(!screen)//显示
					{
						  screen=1; //显示
						
							OLED_Clear();//清屏
						
							OLED_ShowCHinese(40,0,21);//设 
							OLED_ShowCHinese(72,0,22);//置
							 
							OLED_ShowCHinese(32,2,23);//电 
							OLED_ShowCHinese(48,2,24);//调
							OLED_ShowCHinese(64,2,21);//设 
							OLED_ShowCHinese(80,2,22);//置
							 
							OLED_ShowCHinese(32,4,25);//无 
							OLED_ShowCHinese(48,4,26);//线
							OLED_ShowCHinese(64,4,21);//设 
							OLED_ShowCHinese(80,4,22);//置
							
							OLED_ShowCHinese(32,6,27);//地 
							OLED_ShowCHinese(48,6,28);//磁
							OLED_ShowCHinese(64,6,21);//设 
							OLED_ShowCHinese(80,6,22);//置
							
							//显示现在的选择
								switch( mode_set )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
					}
					
					if( frame_flag == 1 )      // 一帧红外数据接收完成
					{	
						key_val = IrDa_Process();//获取按键
						if(key_val==RED_Down)//"-"
						{
							 mode_set++;
								if(mode_set==4)
									mode_set=1;
						}
						else if(key_val==RED_Rise)//"+"
						{
							 mode_set--;
								if(mode_set==0)
									mode_set=3;
						}
												
						else if(key_val==RED_OK)// ">"  进入
						{
							  if(mode_set==1)    //进入模式1设置
								{
										MENU_1();//进入子一级菜单1
								}
								else if(mode_set==2)   //进入模式2设置
								{
										MENU_2();//进入子一级菜单2									
								}
								else if(mode_set==3)   //进入模式3设置
								{
										
									  MENU_3();//进入子一级菜单3	
								}
						}					
					
						if(key_val==RED_RETURN)//RETURN返回
						{
							  OLED_Clear();//清屏
								OLED_ShowCHinese(40,3,21);//设 
								OLED_ShowCHinese(72,3,22);//置
								if(ERR_STOP)
									OLED_ShowString(32,6,"LOCK"); //锁定
								screen=0;//显示清除
								POWER_set=0;
							  key_val=255;
								clean_value=1;//积分清零标志位
							
								RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE); /* TIM6 重新开时钟，开始计时 */
						}
						else
						{
								//显示现在的选择
								switch( mode_set )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
						}
					}					
			}
		}

}

/***************************/
/*******子一级菜单 1 ********/
/***************************/

void MENU_1(void)
{
		uint8_t mode_set_1; //子一级菜单 1 标志位
									
		mode_set_1=1;
		screen=0;//显示清除
								
		 while(mode_set_1)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowString(32,0,"PID_SET");//显示PID
						OLED_ShowString(16,2,"INT_PID");//显示PID
						OLED_ShowString(16,4,"OUT_PID");//显示PID
						OLED_ShowString(16,6,"YAW_PID");//显示PID
					
						//显示现在的选择
								switch( mode_set_1 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
					
						if(key_val==RED_Down)//"-"
						{
							 mode_set_1++;
								if(mode_set_1==4)
									mode_set_1=1;
						}
						else if(key_val==RED_Rise)//"+"
						{
							 mode_set_1--;
								if(mode_set_1==0)
									mode_set_1=3;
						}
					  else if(key_val==RED_OK)// ">"  进入
						{
							  if(mode_set_1==1)    //进入模式1设置
								{
										MENU_1_1();
								}
								else if(mode_set_1==2)   //进入模式2设置
								{
										MENU_1_2();									
								}
								else if(mode_set_1==3)   //进入模式2设置
								{
										MENU_1_3();									
								}/**/								
						}
					
					
						if(key_val==RED_RETURN)//RETURN返回
						{						
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_1=0;
							  key_val=255;							
								while(frame_flag)//退出子一级菜单 防误按设置
								{
									
									frame_flag = 0;//标志位0
									Delay_us(800000);
								}
								key_val=255;
								
						}
						else
						{
								//显示现在的选择
								switch( mode_set_1 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
						}
				}
		 }
}

/***************************/
/*******子一级菜单 2 ********/
/***************************/


void MENU_2(void)
{
		uint8_t mode_set_2; //子一级菜单 2 标志位
									
		mode_set_2=1;
		screen=0;//显示清除
								
		 while(mode_set_2)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowCHinese(32,0,25);//无 
						OLED_ShowCHinese(48,0,26);//线
						OLED_ShowCHinese(64,0,21);//设 
						OLED_ShowCHinese(80,0,22);//置	
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
					key_val = IrDa_Process();//获取按键				
				
						if(key_val==RED_RETURN)//RETURN返回
						{						
								screen=0;//显示清除
								mode_set_2=0; //模式清零	
							
								while(frame_flag)//退出子一级菜单 防误按设置
								{
									
									frame_flag = 0;//标志位0
									
									Delay_us(800000);
								}
								key_val=255;
								ERR_STOP=0;//姿态意外 停车，临时数据
						}
				}
		 }
}

/***************************/
/*******子一级菜单 3 ********/
/***************************/


/*

*/

void MENU_3(void)
{
	  uint8_t mode_set_3; //子子子一级菜单 311 标志位
									
		mode_set_3=1; //默认第一种模式（地磁场校准）
		screen=0;//显示清除
								
		 while(mode_set_3)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
					
						OLED_ShowCHinese(48,0,30);//校
						OLED_ShowCHinese(64,0,31);//准

						OLED_ShowCHinese(24,3,27);//地 
						OLED_ShowCHinese(40,3,28);//磁
						OLED_ShowCHinese(56,3,29);//场
						OLED_ShowCHinese(72,3,30);//校 
						OLED_ShowCHinese(88,3,31);//准

						OLED_ShowCHinese(24,5,32);//陀 
						OLED_ShowCHinese(40,5,33);//螺
						OLED_ShowCHinese(56,5,34);//仪
						OLED_ShowCHinese(72,5,30);//校 
						OLED_ShowCHinese(88,5,31);//准

						//显示现在的选择
						switch( mode_set_3 )
						{
							case 1:
										OLED_ShowChar(8,3,'*');
										OLED_ShowChar(8,5,' ');

							break;
							
							case 2:
										OLED_ShowChar(8,3,' ');
										OLED_ShowChar(8,5,'*');
							break;
							
							default:       
							break;
						}
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
				
						if(key_val==RED_Down)//"-"
						{
							 mode_set_3++;
								if(mode_set_3==3)
									mode_set_3=1;
						}
						else if(key_val==RED_Rise)//"+"
						{
							 mode_set_3--;
								if(mode_set_3==0)
									mode_set_3=2;
						}
												
						else if(key_val==RED_OK)// ">"  进入
						{   
								RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE); /* TIM6 重新开时钟，开始计时 */
									
							  if(mode_set_3==1)    //进入模式1设置
								{
									
									MENU_3_1();
								}
								else if(mode_set_3==2)   //进入模式2设置
								{
										MENU_3_2();									
								}								
						}					
					
						if(key_val==RED_RETURN)//RETURN返回
						{
							  OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_3=0;
							  key_val=255;
						}
						else
						{
								//显示现在的选择
								switch( mode_set_3 )
								{
									case 1:
												OLED_ShowChar(8,3,'*');
												OLED_ShowChar(8,5,' ');

									break;
									
									case 2:
												OLED_ShowChar(8,3,' ');
												OLED_ShowChar(8,5,'*');
									break;
									
									default:       
									break;
								}
						}
				}
		 }
		 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , DISABLE); /* TIM6 重新开时钟，开始计时 */
}

/******************************/
/*******子二级菜单 3-1 ********/
/******************************/

#include "data_exchang.h"
#include "AHRS.h"
#include "FLASH_RW.h"

void MENU_3_1(void)
{
		uint8_t mode_set_3_1; //子子一级菜单 31 标志位
		extern uint8_t AGM_START;//角度采集标志位
    extern short Mag[3];
		extern short Xscope,Yscope,Zscope;  //范围
	  extern T_float_xyz Mag_corr;//校准后
	
		extern int FLASH_MAG_X_DATA;
		extern int FLASH_MAG_Y_DATA;          //地磁原点校准
		extern int FLASH_MAG_Z_DATA;
	
	  extern short Xoffset,Yoffset,Zoffset;  //补偿
	
		uint8_t scope;
	
		mode_set_3_1=1; //没有选择，只有校准完成进入保存选择，或者直接退出
		screen=0;//显示清除
		
			Xscope=0;
			Yscope=0;
			Zscope=0;
	
			Mag_corr.X=0;
			Mag_corr.Y=0;          //存放到数据里面
			Mag_corr.Z=0;
	
		 while(mode_set_3_1)
		 {			 							 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowCHinese(24,0,27);//地 
						OLED_ShowCHinese(40,0,28);//磁
						OLED_ShowCHinese(56,0,29);//场
						OLED_ShowCHinese(72,0,30);//校 
						OLED_ShowCHinese(88,0,31);//准

				}
				
			   /*在这里采集数据运算的数据*/
				
				if(AGM_START) //50ms采集一次
				{
						AGM_START=0;//清除标志位
						MPU9250ReadMag(Mag);//采集数据
						calibration_Mag(Mag[0],Mag[1],Mag[2]); //对三轴进行校准
											  
						scope=((float)Xscope/120)*100.0;
						if(Xscope>=120)
						{
							scope=100;
						}
						OLED_ShowNum(64,2,scope,3,16);
						scope=((float)Yscope/120)*100.0;
						if(Yscope>=120)
						{
							scope=100;
						}
					  OLED_ShowNum(64,4,scope,3,16);
						scope=((float)Zscope/120)*100.0;
						if(Zscope>=120)
						{
							scope=100;
						}
						OLED_ShowNum(64,6,scope,3,16);
				}
				
				 
				
				if((Xscope>=120)&&(Yscope>=120)&&(Zscope>=120))//RETURN返回  
				{
						complete_data();//进入保存界面
						
						OLED_Clear();//清屏
						screen=0;//显示清除
						mode_set_3_1=0;	
						key_val=255;
				}  
								
				
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
						
				/************************************/
            if(key_val==RED_MENU)//menu模拟采集完成
						{
								complete_data();//进入保存界面
							
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_3_1=0;	
							  key_val=255;
            }
				/************************************/
						
						if(key_val==RED_RETURN)//RETURN返回
						{
							  OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_3_1=0;

								read_flash_data();   //读取数据									
								Xoffset=FLASH_MAG_X_DATA;
								Yoffset=FLASH_MAG_Y_DATA;          //存放到数据里面
								Zoffset=FLASH_MAG_Z_DATA;	
							
							  key_val=255;
						}
				}
		 }
}

/******************************/
/*******子二级菜单 3-2 ********/
/******************************/

#include "data_exchang.h"
#include "AHRS.h"
#include "FLASH_RW.h"

void MENU_3_2(void)
{
		uint8_t mode_set_3_2; //子子一级菜单 31 标志位
		extern uint8_t AGM_START;//角度采集标志位
    extern short Acel[3];    // 加速度
    extern short Gyro[3];    // 角速度

	  extern short offset_GX,offset_GY,offset_GZ;  //补偿
		extern short offset_AX,offset_AY,offset_AZ;  //补偿
	
		extern int FLASH_Gyro_X_DATA;
		extern int FLASH_Gyro_Y_DATA;         //角度原点校准
		extern int FLASH_Gyro_Z_DATA;

		extern int FLASH_Acel_X_DATA;
		extern int FLASH_Acel_Y_DATA;         //加速度原点校准
		extern int FLASH_Acel_Z_DATA;
	
		extern uint8_t calibration_GA_time; //校准采集次数
	
		mode_set_3_2=1; //没有选择，只有校准完成进入保存选择，或者直接退出
		screen=0;//显示清除
		calibration_GA_time=0;//计数器清零
		
		offset_GX=0;
		offset_GY=0;
		offset_GZ=0;
		offset_AX=0;
		offset_AY=0;
		offset_AZ=0;
			
		 while(mode_set_3_2)
		 {			 							 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowCHinese(24,0,32);//陀 
						OLED_ShowCHinese(40,0,33);//螺
						OLED_ShowCHinese(56,0,34);//仪
						OLED_ShowCHinese(72,0,30);//校 
						OLED_ShowCHinese(88,0,31);//准

				}
				
			   /*在这里采集数据运算的数据*/
				
				if(AGM_START) //20ms采集一次
				{
						AGM_START=0;//清除标志位
					
						MPU6050ReadAcc(Acel);	//采集数据
						MPU6050ReadGyro(Gyro);//采集数据
						
						calibration_GA(Gyro[0],Gyro[1],Gyro[2],Acel[0],Acel[1],Acel[2]);		
					
						OLED_ShowNum(64,6,calibration_GA_time,3,16);    //完成度
				}
				
				if(calibration_GA_time==255) //校准错误
				{
						/*显示错误*/
					
						OLED_Clear();//清屏
						screen=0;//显示清除
						mode_set_3_2=0;
					
						read_flash_data();   //读取数据									
						offset_GX=FLASH_Gyro_X_DATA;
						offset_GY=FLASH_Gyro_Y_DATA;         //角度原点校准
						offset_GZ=FLASH_Gyro_Z_DATA;

						offset_AX=FLASH_Acel_X_DATA;
						offset_AY=FLASH_Acel_Y_DATA;         //加速度原点校准
						offset_AZ=FLASH_Acel_Z_DATA;
				}
				
				if(calibration_GA_time==100)//RETURN返回  
				{
						complete_GA_data();//进入保存界面
						
						OLED_Clear();//清屏
						screen=0;//显示清除
						mode_set_3_2=0;	
						key_val=255;
				}  
								
				
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
						
				/************************************/
            if(key_val==RED_MENU)//menu模拟采集完成
						{
								complete_GA_data();//进入保存界面
							
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_3_2=0;	
							  key_val=255;
            }
				/************************************/
						
						if(key_val==RED_RETURN)//RETURN返回
						{
							  OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_3_2=0;

								read_flash_data();   //读取数据									
								offset_GX=FLASH_Gyro_X_DATA;
								offset_GY=FLASH_Gyro_Y_DATA;         //角度原点校准
								offset_GZ=FLASH_Gyro_Z_DATA;

								offset_AX=FLASH_Acel_X_DATA;
								offset_AY=FLASH_Acel_Y_DATA;         //加速度原点校准
								offset_AZ=FLASH_Acel_Z_DATA;	
							
							  key_val=255;
						}
				}
		 }
}





/*********************************/
/**校准采集完成自动进入确认选择函数***/
/******* 子三级菜单 3-1-1  ********/
/*********************************/




void complete_data(void)//修正完成 是否保存数据
{
		uint8_t mode_set_3_1_1; //子子子一级菜单 311 标志位
	
		extern int FLASH_MAG_X_DATA;
		extern int FLASH_MAG_Y_DATA;          //地磁原点校准
		extern int FLASH_MAG_Z_DATA;
	
//		extern T_float_xyz Mag_corr;//校准后
		extern short Xoffset,Yoffset,Zoffset;  //补偿
									
		mode_set_3_1_1=2; //默认第二种模式（取消）
		screen=0;//显示清除
								
		 while(mode_set_3_1_1)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowCHinese(24,0,27);//地 
						OLED_ShowCHinese(40,0,28);//磁
						OLED_ShowCHinese(56,0,29);//场
						OLED_ShowCHinese(72,0,30);//校 
						OLED_ShowCHinese(88,0,31);//准

						//显示现在的选择
						switch( mode_set_3_1_1 )
						{
							case 1:
									OLED_ShowCHinese_inverse(24,3,35);//保反色 
									OLED_ShowCHinese_inverse(40,3,36);//存反色
									OLED_ShowCHinese(56,3,37);//取
									OLED_ShowCHinese(72,3,38);//消 
							break;
							
							case 2:
									OLED_ShowCHinese(24,3,35);//保 
									OLED_ShowCHinese(40,3,36);//存
									OLED_ShowCHinese_inverse(56,3,37);//取
									OLED_ShowCHinese_inverse(72,3,38);//消
							break;
							
							default:       
							break;
						}
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
				
						if(key_val==RED_Down || key_val==RED_Right)//"-"
						{
							 mode_set_3_1_1++;
								if(mode_set_3_1_1==3)
									mode_set_3_1_1=1;
						}
						else if(key_val==RED_Rise || key_val==RED_Left)//"+"
						{
							 mode_set_3_1_1--;
								if(mode_set_3_1_1==0)
									mode_set_3_1_1=2;
						}
												
						else if(key_val==RED_OK)// ">"  进入
						{
							  if(mode_set_3_1_1==1)    //进入模式1设置
								{
										/*执行选择1*/
									
//									  OLED_ShowString(32,2,"reading");
									  printf("补偿：%d,\t%d,\t%d\n\n",Xoffset,Yoffset,Zoffset);
									  read_flash_data();   //读取数据									
									  FLASH_MAG_X_DATA=Xoffset;  //补偿;
										FLASH_MAG_Y_DATA=Yoffset;          //存放到数据里面
										FLASH_MAG_Z_DATA=Zoffset;										
										write_flash_data();  //写入数据
									
										mode_set_3_1_1=0;
								}
								else if(mode_set_3_1_1==2)   //进入模式2设置
								{
										/*执行选择2*/
										mode_set_3_1_1=0;
								}								
						}					
					
						if(key_val==RED_RETURN)//RETURN返回
						{
							  OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_3_1_1=0;
								key_val=255;
						}
						else
						{
								//显示现在的选择
								switch( mode_set_3_1_1 )
								{
									case 1:
											OLED_ShowCHinese_inverse(24,3,35);//保反色 
											OLED_ShowCHinese_inverse(40,3,36);//存反色
											OLED_ShowCHinese(56,3,37);//取
											OLED_ShowCHinese(72,3,38);//消 
									break;
									
									case 2:
											OLED_ShowCHinese(24,3,35);//保 
											OLED_ShowCHinese(40,3,36);//存
											OLED_ShowCHinese_inverse(56,3,37);//取
									    OLED_ShowCHinese_inverse(72,3,38);//消
									break;
									
									default:       
									break;
								}
						}
				}
		 }
}


/*********************************/
/**校准采集完成自动进入确认选择函数***/
/******* 子三级菜单 3-2-2  ********/
/*********************************/




void complete_GA_data(void)//修正完成 是否保存数据
{
		uint8_t mode_set_3_2_2; //子子子一级菜单 311 标志位
	
		extern int FLASH_Gyro_X_DATA;
		extern int FLASH_Gyro_Y_DATA;         //角度原点校准
		extern int FLASH_Gyro_Z_DATA;

		extern int FLASH_Acel_X_DATA;
		extern int FLASH_Acel_Y_DATA;         //加速度原点校准
		extern int FLASH_Acel_Z_DATA;
	
	  extern short offset_GX,offset_GY,offset_GZ;  //补偿
		extern short offset_AX,offset_AY,offset_AZ;  //补偿
	
									
		mode_set_3_2_2=2; //默认第二种模式（取消）
		screen=0;//显示清除
								
		 while(mode_set_3_2_2)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowCHinese(24,0,32);//陀 
						OLED_ShowCHinese(40,0,33);//螺
						OLED_ShowCHinese(56,0,34);//仪
						OLED_ShowCHinese(72,0,30);//校 
						OLED_ShowCHinese(88,0,31);//准

						//显示现在的选择
						switch( mode_set_3_2_2 )
						{
							case 1:
									OLED_ShowCHinese_inverse(24,3,35);//保反色 
									OLED_ShowCHinese_inverse(40,3,36);//存反色
									OLED_ShowCHinese(56,3,37);//取
									OLED_ShowCHinese(72,3,38);//消 
							break;
							
							case 2:
									OLED_ShowCHinese(24,3,35);//保 
									OLED_ShowCHinese(40,3,36);//存
									OLED_ShowCHinese_inverse(56,3,37);//取
									OLED_ShowCHinese_inverse(72,3,38);//消
							break;
							
							default:       
							break;
						}
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
				
						if(key_val==RED_Down || key_val==RED_Right)//"-"
						{
							 mode_set_3_2_2++;
								if(mode_set_3_2_2==3)
									mode_set_3_2_2=1;
						}
						else if(key_val==RED_Rise || key_val==RED_Left)//"+"
						{
							 mode_set_3_2_2--;
								if(mode_set_3_2_2==0)
									mode_set_3_2_2=2;
						}
												
						else if(key_val==RED_OK)// ">"  进入
						{
							  if(mode_set_3_2_2==1)    //进入模式1设置
								{
										/*执行选择1*/
									
//									  OLED_ShowString(32,2,"reading");
//									  printf("角 度补偿：%d,\t%d,\t%d\n\n",offset_GX,offset_GY,offset_GZ);
//										printf("加速度补偿：%d,\t%d,\t%d\n\n",offset_AX,offset_AY,offset_AZ);
									  read_flash_data();   //读取数据		
									
										FLASH_Gyro_X_DATA=offset_GX;
										FLASH_Gyro_Y_DATA=offset_GY;         //角度原点校准
										FLASH_Gyro_Z_DATA=offset_GZ;

										FLASH_Acel_X_DATA=offset_AX;
										FLASH_Acel_Y_DATA=offset_AY;         //加速度原点校准
										FLASH_Acel_Z_DATA=offset_AZ;								
										write_flash_data();  //写入数据
									
										mode_set_3_2_2=0;
								}
								else if(mode_set_3_2_2==2)   //进入模式2设置
								{
										/*执行选择2*/
										mode_set_3_2_2=0;
								}								
						}					
					
						if(key_val==RED_RETURN)//RETURN返回
						{
							  OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_3_2_2=0;
								key_val=255;
						}
						else
						{
								//显示现在的选择
								switch( mode_set_3_2_2 )
								{
									case 1:
											OLED_ShowCHinese_inverse(24,3,35);//保反色 
											OLED_ShowCHinese_inverse(40,3,36);//存反色
											OLED_ShowCHinese(56,3,37);//取
											OLED_ShowCHinese(72,3,38);//消 
									break;
									
									case 2:
											OLED_ShowCHinese(24,3,35);//保 
											OLED_ShowCHinese(40,3,36);//存
											OLED_ShowCHinese_inverse(56,3,37);//取
									    OLED_ShowCHinese_inverse(72,3,38);//消
									break;
									
									default:       
									break;
								}
						}
				}
		 }
}
/***************************/
/*******子一级菜单 1-1 ******/
/***************************/

void MENU_1_1(void)
{
		uint8_t mode_set_1_1; //子一级菜单 1 标志位
		
		extern int interior_KP; 			//内部
		extern int interior_KI;
		extern int interior_KD;
	
		extern int FLASH_INPID_P_DATA;
		extern int FLASH_INPID_I_DATA;       //内环PID
		extern int FLASH_INPID_D_DATA;

		extern float interior_KP_f; 			//内部
		extern float interior_KI_f;
		extern float interior_KD_f;
	
		mode_set_1_1=1;
		screen=0;//显示清除
		
		Flash_data_init(); //获取内存数据
	
		 while(mode_set_1_1)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowString(32,0,"PID INT");//显示
					
						switch( mode_set_1_1 )
						{
							case 1:
								OLED_ShowChar(8,2,'*');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 2:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,'*');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 3:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,'*');
							break;
							
							default:       
							break;
						}
						OLED_ShowNum(64,2,interior_KP,5,16);    //KP的值
						OLED_ShowNum(64,4,interior_KI,5,16);    //KI的值
						OLED_ShowNum(64,6,interior_KD,5,16);    //KD的值
					
						/*显示现在的选择*/		
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
					
//					
					/*1进位*/	
					  if(key_val==RED_Down)
						{
								if(mode_set_1_1==1)  //KP
								{
										interior_KP--;
								}
								else if(mode_set_1_1==2)
								{
										interior_KI--;
								}
								else if(mode_set_1_1==3)
								{
										interior_KD--;
								}
						}
						else if(key_val==RED_Rise)
						{
							   if(mode_set_1_1==1)  //KP
								{
										interior_KP++;
								}
								else if(mode_set_1_1==2)
								{
										interior_KI++;
								}
								else if(mode_set_1_1==3)
								{
										interior_KD++;
								}
						}
						/*十进位*/
						else if(key_val==RED_1)
						{
								if(mode_set_1_1==1)  //KP
								{
										interior_KP-=10;
								}
								else if(mode_set_1_1==2)
								{
										interior_KI-=10;
								}
								else if(mode_set_1_1==3)
								{
										interior_KD-=10;
								}
						}
						else if(key_val==RED_0)
						{
							   if(mode_set_1_1==1)  //KP
								{
										interior_KP+=10;
								}
								else if(mode_set_1_1==2)
								{
										interior_KI+=10;
								}
								else if(mode_set_1_1==3)
								{
										interior_KD+=10;
								}
						}
						/*百进位*/
						else if(key_val==RED_3)
						{
								if(mode_set_1_1==1)  //KP
								{
										interior_KP-=100;
								}
								else if(mode_set_1_1==2)
								{
										interior_KI-=100;
								}
								else if(mode_set_1_1==3)
								{
										interior_KD-=100;
								}
						}
						else if(key_val==RED_C)
						{
							   if(mode_set_1_1==1)  //KP
								{
										interior_KP+=100;
								}
								else if(mode_set_1_1==2)
								{
										interior_KI+=100;
								}
								else if(mode_set_1_1==3)
								{
										interior_KD+=100;
								}
						}
						
						
						/****************************/
						else if(key_val==RED_Right)//"右"
						{
							 mode_set_1_1++;
								if(mode_set_1_1==4)
									mode_set_1_1=1;
						}
						else if(key_val==RED_Left)//"+"
						{
							 mode_set_1_1--;
								if(mode_set_1_1==0)
									mode_set_1_1=3;
						}
//					  else if(key_val==RED_OK)// ">"  进入
//						{
//							  if(mode_set_1_1==1)    //进入模式1设置
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_1==2)   //进入模式2设置
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_1==3)   //进入模式2设置
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN返回
						{						
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_1_1=0;
							  key_val=255;							
								while(frame_flag)//退出子一级菜单 防误按设置
								{
									
									frame_flag = 0;//标志位0
									Delay_us(800000);
								}
								key_val=255;
								
										FLASH_INPID_P_DATA=interior_KP;
										FLASH_INPID_I_DATA=interior_KI;      //内环PID
										FLASH_INPID_D_DATA=interior_KD;
								
										interior_KP_f=(float)interior_KP/1000; 			//内环 数据
										interior_KI_f=(float)interior_KI/1000;
										interior_KD_f=(float)interior_KD/1000;
								
								write_flash_data();  //写入数据
						}
						else
						{
								//显示现在的选择
								switch( mode_set_1_1 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
								OLED_ShowNum(64,2,interior_KP,5,16);    //KP的值99.999
								OLED_ShowNum(64,4,interior_KI,5,16);    //KI的值
								OLED_ShowNum(64,6,interior_KD,5,16);    //KD的值
						}
				}
		 }
}

/***************************/
/*******子一级菜单 1-2 ******/
/***************************/

void MENU_1_2(void)
{
		uint8_t mode_set_1_2; //子一级菜单 1 标志位
		
	  extern int external_KP; 			//外环
		extern int external_KI;
		extern int external_KD;

		extern int FLASH_OUTPID_P_DATA;
		extern int FLASH_OUTPID_I_DATA;     //外环PID
		extern int FLASH_OUTPID_D_DATA;
	
	  extern float external_KP_f; 			//外环
		extern float external_KI_f;
		extern float external_KD_f;
	
		mode_set_1_2=1;
		screen=0;//显示清除
		
		Flash_data_init(); //获取内存数据
	
		 while(mode_set_1_2)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowString(32,0,"PID OUT");//显示
					
						switch( mode_set_1_2 )
						{
							case 1:
								OLED_ShowChar(8,2,'*');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 2:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,'*');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 3:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,'*');
							break;
							
							default:       
							break;
						}
						OLED_ShowNum(64,2,external_KP,6,16);    //KP的值
						OLED_ShowNum(64,4,external_KI,6,16);    //KI的值
						OLED_ShowNum(48,6,external_KD,8,16);    //KD的值
					
						/*显示现在的选择*/		
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
					
//						
					  if(key_val==RED_Down)
						{
								if(mode_set_1_2==1)  //KP
								{
										external_KP--;
								}
								else if(mode_set_1_2==2)
								{
										external_KI--;
								}
								else if(mode_set_1_2==3)
								{
										external_KD--;
								}
						}
						else if(key_val==RED_Rise)
						{
							   if(mode_set_1_2==1)  //KP
								{
										external_KP++;
								}
								else if(mode_set_1_2==2)
								{
										external_KI++;
								}
								else if(mode_set_1_2==3)
								{
										external_KD++;
								}
						}
						else if(key_val==RED_1)
						{
								if(mode_set_1_2==1)  //KP
								{
										external_KP-=10;
								}
								else if(mode_set_1_2==2)
								{
										external_KI-=10;
								}
								else if(mode_set_1_2==3)
								{
										external_KD-=10;
								}
						}
						else if(key_val==RED_0)
						{
							   if(mode_set_1_2==1)  //KP
								{
										external_KP+=10;
								}
								else if(mode_set_1_2==2)
								{
										external_KI+=10;
								}
								else if(mode_set_1_2==3)
								{
										external_KD+=10;
								}
						}
						/*百进位*/
						else if(key_val==RED_3)
						{
								if(mode_set_1_2==1)  //KP
								{
										external_KP-=100;
								}
								else if(mode_set_1_2==2)
								{
										external_KI-=100;
								}
								else if(mode_set_1_2==3)
								{
										external_KD-=100;
								}
						}
						else if(key_val==RED_C)
						{
							   if(mode_set_1_2==1)  //KP
								{
										external_KP+=100;
								}
								else if(mode_set_1_2==2)
								{
										external_KI+=100;
								}
								else if(mode_set_1_2==3)
								{
										external_KD+=100;
								}
						}
						/*千进位*/
						else if(key_val==RED_4)
						{
								if(mode_set_1_2==1)  //KP
								{
										external_KP-=1000;
								}
								else if(mode_set_1_2==2)
								{
										external_KI-=1000;
								}
								else if(mode_set_1_2==3)
								{
										external_KD-=1000;
								}
						}
						else if(key_val==RED_7)
						{
							   if(mode_set_1_2==1)  //KP
								{
										external_KP+=1000;
								}
								else if(mode_set_1_2==2)
								{
										external_KI+=1000;
								}
								else if(mode_set_1_2==3)
								{
										external_KD+=1000;
								}
						}
						/*万进位*/
						else if(key_val==RED_5)
						{
								if(mode_set_1_2==1)  //KP
								{
										external_KP-=10000;
								}
								else if(mode_set_1_2==2)
								{
										external_KI-=10000;
								}
								else if(mode_set_1_2==3)
								{
										external_KD-=10000;
								}
						}
						else if(key_val==RED_8)
						{
							   if(mode_set_1_2==1)  //KP
								{
										external_KP+=10000;
								}
								else if(mode_set_1_2==2)
								{
										external_KI+=10000;
								}
								else if(mode_set_1_2==3)
								{
										external_KD+=10000;
								}
						}/*十万进位*/
						else if(key_val==RED_6)
						{
								if(mode_set_1_2==1)  //KP
								{
										external_KP-=100000;
								}
								else if(mode_set_1_2==2)
								{
										external_KI-=100000;
								}
								else if(mode_set_1_2==3)
								{
										external_KD-=100000;
								}
						}
						else if(key_val==RED_9)
						{
							   if(mode_set_1_2==1)  //KP
								{
										external_KP+=100000;
								}
								else if(mode_set_1_2==2)
								{
										external_KI+=100000;
								}
								else if(mode_set_1_2==3)
								{
										external_KD+=100000;
								}
						}
						
						else if(key_val==RED_Right)//"右"
						{
							 mode_set_1_2++;
								if(mode_set_1_2==4)
									mode_set_1_2=1;
						}
						else if(key_val==RED_Left)//"+"
						{
							 mode_set_1_2--;
								if(mode_set_1_2==0)
									mode_set_1_2=3;
						}
//					  else if(key_val==RED_OK)// ">"  进入
//						{
//							  if(mode_set_1_2==1)    //进入模式1设置
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_2==2)   //进入模式2设置
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_2==3)   //进入模式2设置
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN返回
						{						
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_1_2=0;
							  key_val=255;							
								while(frame_flag)//退出子一级菜单 防误按设置
								{
									
									frame_flag = 0;//标志位0
									Delay_us(800000);
								}
								key_val=255;
								
								
										FLASH_OUTPID_P_DATA=external_KP;
										FLASH_OUTPID_I_DATA=external_KI;     //外环PID
										FLASH_OUTPID_D_DATA=external_KD;

								    external_KP_f=(float)external_KP/1000.0; 			//外环
										external_KI_f=(float)external_KI/1000.0;
										external_KD_f=(float)external_KD/100.0;

								
								write_flash_data();  //写入数据
						}
						else
						{
								//显示现在的选择
								switch( mode_set_1_2 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
								OLED_ShowNum(64,2,external_KP,6,16);    //KP的值99.999
								OLED_ShowNum(64,4,external_KI,6,16);    //KI的值
								OLED_ShowNum(48,6,external_KD,8,16);    //KD的值
						}
				}
		 }
}

/***************************/
/*******子一级菜单 1 ********/
/***************************/

void MENU_1_3(void)
{
		uint8_t mode_set_1_3; //子一级菜单 1 标志位
									
		mode_set_1_3=1;
		screen=0;//显示清除
								
		 while(mode_set_1_3)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowString(32,0,"YAW_SET");//显示PID
						OLED_ShowString(16,2,"INT_PID");//显示PID
						OLED_ShowString(16,4,"OUT_PID");//显示PID
//						OLED_ShowString(16,6,"YAW_PID");//显示PID
					
						//显示现在的选择
								switch( mode_set_1_3 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
//									case 3:
//										OLED_ShowChar(8,2,' ');
//										OLED_ShowChar(8,4,' ');
//										OLED_ShowChar(8,6,'*');
//									break;
									
									default:       
									break;
								}
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
					
						if(key_val==RED_Down)//"-"
						{
							 mode_set_1_3++;
								if(mode_set_1_3==3)
									mode_set_1_3=1;
						}
						else if(key_val==RED_Rise)//"+"
						{
							 mode_set_1_3--;
								if(mode_set_1_3==0)
									mode_set_1_3=2;
						}
					  else if(key_val==RED_OK)// ">"  进入
						{
							  if(mode_set_1_3==1)    //进入模式1设置
								{
										MENU_1_3_1();
								}
								else if(mode_set_1_3==2)   //进入模式2设置
								{
										MENU_1_3_2();									
								}
//								else if(mode_set_1==3)   //进入模式2设置
//								{
//										MENU_1_3();									
//								}/**/								
						}
					
					
						if(key_val==RED_RETURN)//RETURN返回
						{						
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_1_3=0;
							  key_val=255;							
								while(frame_flag)//退出子一级菜单 防误按设置
								{
									
									frame_flag = 0;//标志位0
									Delay_us(800000);
								}
								key_val=255;
								
						}
						else
						{
								//显示现在的选择
								switch( mode_set_1_3 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
//									case 3:
//										OLED_ShowChar(8,2,' ');
//										OLED_ShowChar(8,4,' ');
//										OLED_ShowChar(8,6,'*');
//									break;
									
									default:       
									break;
								}
						}
				}
		 }
}
/***************************/
/*******子一级菜单 1-3-1 ******/
/***************************/

void MENU_1_3_1(void)
{
		uint8_t mode_set_1_3_1; //子一级菜单 1 标志位
		
		extern int YAWinterior_KP; 			//内部
		extern int YAWinterior_KI;
		extern int YAWinterior_KD;
	
		extern int FLASH_YAWINPID_P_DATA;
		extern int FLASH_YAWINPID_I_DATA;       //内环PID
		extern int FLASH_YAWINPID_D_DATA;

		extern float YAWinterior_KP_f; 			//内部
		extern float YAWinterior_KI_f;
		extern float YAWinterior_KD_f;
	
		mode_set_1_3_1=1;
		screen=0;//显示清除
		
		Flash_data_init(); //获取内存数据
	
		 while(mode_set_1_3_1)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowString(32,0,"YAW INT");//显示
					
						switch( mode_set_1_3_1 )
						{
							case 1:
								OLED_ShowChar(8,2,'*');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 2:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,'*');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 3:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,'*');
							break;
							
							default:       
							break;
						}
						OLED_ShowNum(64,2,YAWinterior_KP,5,16);    //KP的值
						OLED_ShowNum(64,4,YAWinterior_KI,5,16);    //KI的值
						OLED_ShowNum(64,6,YAWinterior_KD,5,16);    //KD的值
					
						/*显示现在的选择*/		
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
					
//					
					/*1进位*/	
					  if(key_val==RED_Down)
						{
								if(mode_set_1_3_1==1)  //KP
								{
										YAWinterior_KP--;
								}
								else if(mode_set_1_3_1==2)
								{
										YAWinterior_KI--;
								}
								else if(mode_set_1_3_1==3)
								{
										YAWinterior_KD--;
								}
						}
						else if(key_val==RED_Rise)
						{
							   if(mode_set_1_3_1==1)  //KP
								{
										YAWinterior_KP++;
								}
								else if(mode_set_1_3_1==2)
								{
										YAWinterior_KI++;
								}
								else if(mode_set_1_3_1==3)
								{
										YAWinterior_KD++;
								}
						}
						/*十进位*/
						else if(key_val==RED_1)
						{
								if(mode_set_1_3_1==1)  //KP
								{
										YAWinterior_KP-=10;
								}
								else if(mode_set_1_3_1==2)
								{
										YAWinterior_KI-=10;
								}
								else if(mode_set_1_3_1==3)
								{
										YAWinterior_KD-=10;
								}
						}
						else if(key_val==RED_0)
						{
							   if(mode_set_1_3_1==1)  //KP
								{
										YAWinterior_KP+=10;
								}
								else if(mode_set_1_3_1==2)
								{
										YAWinterior_KI+=10;
								}
								else if(mode_set_1_3_1==3)
								{
										YAWinterior_KD+=10;
								}
						}
						/*百进位*/
						else if(key_val==RED_3)
						{
								if(mode_set_1_3_1==1)  //KP
								{
										YAWinterior_KP-=100;
								}
								else if(mode_set_1_3_1==2)
								{
										YAWinterior_KI-=100;
								}
								else if(mode_set_1_3_1==3)
								{
										YAWinterior_KD-=100;
								}
						}
						else if(key_val==RED_C)
						{
							   if(mode_set_1_3_1==1)  //KP
								{
										YAWinterior_KP+=100;
								}
								else if(mode_set_1_3_1==2)
								{
										YAWinterior_KI+=100;
								}
								else if(mode_set_1_3_1==3)
								{
										YAWinterior_KD+=100;
								}
						}
						
						
						/****************************/
						else if(key_val==RED_Right)//"右"
						{
							 mode_set_1_3_1++;
								if(mode_set_1_3_1==4)
									mode_set_1_3_1=1;
						}
						else if(key_val==RED_Left)//"+"
						{
							 mode_set_1_3_1--;
								if(mode_set_1_3_1==0)
									mode_set_1_3_1=3;
						}
//					  else if(key_val==RED_OK)// ">"  进入
//						{
//							  if(mode_set_1_1==1)    //进入模式1设置
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_1==2)   //进入模式2设置
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_1==3)   //进入模式2设置
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN返回
						{						
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_1_3_1=0;
							  key_val=255;							
								while(frame_flag)//退出子一级菜单 防误按设置
								{
									
									frame_flag = 0;//标志位0
									Delay_us(800000);
								}
								key_val=255;
								
										FLASH_YAWINPID_P_DATA=YAWinterior_KP;
										FLASH_YAWINPID_I_DATA=YAWinterior_KI;      //内环PID
										FLASH_YAWINPID_D_DATA=YAWinterior_KD;
								
										YAWinterior_KP_f=(float)YAWinterior_KP/1000; 			//内环 数据
										YAWinterior_KI_f=(float)YAWinterior_KI/1000;
										YAWinterior_KD_f=(float)YAWinterior_KD/1000;
								
								write_flash_data();  //写入数据
						}
						else
						{
								//显示现在的选择
								switch( mode_set_1_3_1 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
								OLED_ShowNum(64,2,YAWinterior_KP,5,16);    //KP的值99.999
								OLED_ShowNum(64,4,YAWinterior_KI,5,16);    //KI的值
								OLED_ShowNum(64,6,YAWinterior_KD,5,16);    //KD的值
						}
				}
		 }
}

/***************************/
/*******子一级菜单 1-3-2 ******/
/***************************/

void MENU_1_3_2(void)
{
		uint8_t mode_set_1_3_2; //子一级菜单 1 标志位
		
	  extern int YAWexternal_KP; 			//外环
		extern int YAWexternal_KI;
		extern int YAWexternal_KD;

		extern int FLASH_YAWOUTPID_P_DATA;
		extern int FLASH_YAWOUTPID_I_DATA;     //外环PID
		extern int FLASH_YAWOUTPID_D_DATA;
	
	  extern float YAWexternal_KP_f; 			//外环
		extern float YAWexternal_KI_f;
		extern float YAWexternal_KD_f;
	
		mode_set_1_3_2=1;
		screen=0;//显示清除
		
		Flash_data_init(); //获取内存数据
	
		 while(mode_set_1_3_2)
		 {			 
				if(!screen)//显示
				{
						screen=1; //显示
					
						OLED_Clear();//清屏
						OLED_ShowString(32,0,"PID OUT");//显示
					
						switch( mode_set_1_3_2 )
						{
							case 1:
								OLED_ShowChar(8,2,'*');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 2:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,'*');
								OLED_ShowChar(8,6,' ');
							break;
							
							case 3:
								OLED_ShowChar(8,2,' ');
								OLED_ShowChar(8,4,' ');
								OLED_ShowChar(8,6,'*');
							break;
							
							default:       
							break;
						}
						OLED_ShowNum(64,2,YAWexternal_KP,6,16);    //KP的值
						OLED_ShowNum(64,4,YAWexternal_KI,6,16);    //KI的值
						OLED_ShowNum(48,6,YAWexternal_KD,8,16);    //KD的值
					
						/*显示现在的选择*/		
				}
			 
				if( frame_flag == 1 )      // 一帧红外数据接收完成
				{	
						key_val = IrDa_Process();//获取按键				
					
//						
					  if(key_val==RED_Down)
						{
								if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP--;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI--;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD--;
								}
						}
						else if(key_val==RED_Rise)
						{
							   if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP++;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI++;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD++;
								}
						}
						else if(key_val==RED_1)
						{
								if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP-=10;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI-=10;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD-=10;
								}
						}
						else if(key_val==RED_0)
						{
							   if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP+=10;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI+=10;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD+=10;
								}
						}
						/*百进位*/
						else if(key_val==RED_3)
						{
								if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP-=100;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI-=100;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD-=100;
								}
						}
						else if(key_val==RED_C)
						{
							   if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP+=100;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI+=100;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD+=100;
								}
						}
						/*千进位*/
						else if(key_val==RED_4)
						{
								if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP-=1000;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI-=1000;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD-=1000;
								}
						}
						else if(key_val==RED_7)
						{
							   if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP+=1000;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI+=1000;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD+=1000;
								}
						}
						/*万进位*/
						else if(key_val==RED_5)
						{
								if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP-=10000;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI-=10000;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD-=10000;
								}
						}
						else if(key_val==RED_8)
						{
							   if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP+=10000;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI+=10000;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD+=10000;
								}
						}/*十万进位*/
						else if(key_val==RED_6)
						{
								if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP-=100000;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI-=100000;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD-=100000;
								}
						}
						else if(key_val==RED_9)
						{
							   if(mode_set_1_3_2==1)  //KP
								{
										YAWexternal_KP+=100000;
								}
								else if(mode_set_1_3_2==2)
								{
										YAWexternal_KI+=100000;
								}
								else if(mode_set_1_3_2==3)
								{
										YAWexternal_KD+=100000;
								}
						}
						
						else if(key_val==RED_Right)//"右"
						{
							 mode_set_1_3_2++;
								if(mode_set_1_3_2==4)
									mode_set_1_3_2=1;
						}
						else if(key_val==RED_Left)//"+"
						{
							 mode_set_1_3_2--;
								if(mode_set_1_3_2==0)
									mode_set_1_3_2=3;
						}
//					  else if(key_val==RED_OK)// ">"  进入
//						{
//							  if(mode_set_1_3_2==1)    //进入模式1设置
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_3_2==2)   //进入模式2设置
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_3_2==3)   //进入模式2设置
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN返回
						{						
								OLED_Clear();//清屏
								screen=0;//显示清除
								mode_set_1_3_2=0;
							  key_val=255;							
								while(frame_flag)//退出子一级菜单 防误按设置
								{
									
									frame_flag = 0;//标志位0
									Delay_us(800000);
								}
								key_val=255;
								
								
										FLASH_YAWOUTPID_P_DATA=YAWexternal_KP;
										FLASH_YAWOUTPID_I_DATA=YAWexternal_KI;     //外环PID
										FLASH_YAWOUTPID_D_DATA=YAWexternal_KD;

								    YAWexternal_KP_f=(float)YAWexternal_KP/1000.0; 			//外环
										YAWexternal_KI_f=(float)YAWexternal_KI/1000.0;
										YAWexternal_KD_f=(float)YAWexternal_KD/100.0;

								
								write_flash_data();  //写入数据
						}
						else
						{
								//显示现在的选择
								switch( mode_set_1_3_2 )
								{
									case 1:
										OLED_ShowChar(8,2,'*');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 2:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,'*');
										OLED_ShowChar(8,6,' ');
									break;
									
									case 3:
										OLED_ShowChar(8,2,' ');
										OLED_ShowChar(8,4,' ');
										OLED_ShowChar(8,6,'*');
									break;
									
									default:       
									break;
								}
								OLED_ShowNum(64,2,YAWexternal_KP,6,16);    //KP的值99.999
								OLED_ShowNum(64,4,YAWexternal_KI,6,16);    //KI的值
								OLED_ShowNum(48,6,YAWexternal_KD,8,16);    //KD的值
						}
				}
		 }
}
