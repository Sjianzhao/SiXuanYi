#include "PID_control.h"
#include "AHRS.h"
#include "pwm_config.h"
#include "data_exchang.h"
#include "swap_data.h"
#include "init_set.h"

#define PWM_MAX 130.0
#define PWM_MIN -130.0

#define SET_MAX 23.0f    //限制意外角度
#define SET_MIN -23.0f

int8_t ERR_STOP=0;    //意外停车

//#define external_KP 0.0f 			//外环
//#define external_KI 0.0f
//#define external_KD 0.0f

//#define interior_KP 0.0f 			//内部
//#define interior_KI 0.0f
//#define interior_KD 0.0f

int external_KP; 			//外环
int external_KI;
int external_KD;

int interior_KP; 			//内部
int interior_KI;
int interior_KD;

float external_KP_f; 			//外环
float external_KI_f;
float external_KD_f;

float interior_KP_f; 			//内部
float interior_KI_f;
float interior_KD_f;
/*YAW*/
int YAWexternal_KP; 			//外环
int YAWexternal_KI;
int YAWexternal_KD;

int YAWinterior_KP; 			//内部
int YAWinterior_KI;
int YAWinterior_KD;

float YAWexternal_KP_f; 			//外环
float YAWexternal_KI_f;
float YAWexternal_KD_f;

float YAWinterior_KP_f; 			//内部
float YAWinterior_KI_f;
float YAWinterior_KD_f;

float roll_PWM=0; 	//俯仰
float pitch_PWM=0;	//横滚

short roll_PWM_OUT=0;
short pitch_PWM_OUT=0;
short yaw_PWM_OUT=0;


int yaw_PWM=0;		//偏航

extern T_float_xyz averag_Acel; //卡尔曼滤波后的加速度数据
extern T_float_xyz averag_gyro;
extern T_float_xyz origin_Acel;//修正后的加速度值
extern T_float_xyz radian_gyro;   //修正后角度值
extern  T_float_xyz value_gyro; //校准后的数据

uint8_t clean_value=0;//积分清零标志位

extern short accelerator_IN;  //油门 接收到的油门数据
extern short roll_IN,pitch_IN;//俯仰，横滚

float START_YAW; //初始偏差


int cheshi;
float cheshi4,cheshi1,cheshi2,cheshi3,cheshi5;
/*俯仰PID*/
static float spend_err_roll_external[3] = {0,0,0};
static float spend_err_roll_interior[3] = {0,0,0};

void PID_roll_Control(float roll_set)
{
      //外环
			static float roll_external;

			static float  errP = 0,errD = 0,errI = 0;
	 
			//内环
//			static int roll_interior;

			static float errP_int = 0,errD_int = 0,errI_int = 0;
			
			roll_set+=(float)(roll_IN*0.01); //俯仰
	
			if(roll_set>SET_MAX)
			{roll_set=SET_MAX;ERR_STOP=1;OLED_ShowString(16,6,"LOCK");}
			if(roll_set<(SET_MIN))
			{roll_set=(SET_MIN);ERR_STOP=1;OLED_ShowString(16,6,"LOCK");}
//			if(ERR_STOP)
//				OLED_ShowString(16,6,"LOCK"); //锁定
//			else
//				OLED_ShowString(32,6,"     ");
			
			if(clean_value)    //积分清零标志位
			{
//					clean_value=0;
					roll_external=0;
					spend_err_roll_external[2]=0;
					spend_err_roll_interior[2]=0;
					spend_err_roll_interior[1]=0;
					spend_err_roll_interior[0]=0;
					roll_PWM=0;
			}
			
			
			
//	    cheshi1=averag_gyro_OUT_Y(roll_set);
			
			
	 
//			cheshi=roll_set;
//			 cheshi1=averag_gyro_OUT_X(roll_set);
			cheshi1=roll_set;
			spend_err_roll_external[2] = spend_err_roll_external[1]+spend_err_roll_external[2]; 	//历史所有的偏差
			spend_err_roll_external[1] = spend_err_roll_external[0];             	//保存上一次偏差
			spend_err_roll_external[0] =-roll_set;//cheshi1;					//偏差值=期望值-现实值

			
			if(spend_err_roll_external[2] >  20) spend_err_roll_external[2] = 20;
			if(spend_err_roll_external[2] < -20) spend_err_roll_external[2] = -20;   //积分限幅
			
			
			errP=spend_err_roll_external[0];                            		//此时的偏差
			errI=spend_err_roll_external[0]+spend_err_roll_external[1]+spend_err_roll_external[2];    	//历史以来所有的偏差
			errD=spend_err_roll_external[0]-spend_err_roll_external[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
			roll_external = (float)(errP*external_KP_f+errI*external_KI_f+errD*external_KD_f);        //PID计算 输出控制量
			
//			roll_external
//      cheshi2= roll_external;
	 /*控制量限幅*/
	 
			spend_err_roll_interior[2] += spend_err_roll_interior[0]; 	//历史所有的偏差
			spend_err_roll_interior[1] = spend_err_roll_interior[0];             	//保存上一次偏差
			
			cheshi4=averag_gyro_X(value_gyro.X);
//				cheshi4=value_gyro.X;
			                                    
			spend_err_roll_interior[0] =  roll_external - cheshi4;//cheshi4;					//与角速度做差
			                            

			
			if(spend_err_roll_interior[2] >  20.0) spend_err_roll_interior[2] = 20.0;
			if(spend_err_roll_interior[2] < -20.0) spend_err_roll_interior[2] = -20.0;   //积分限幅
			
//       cheshi3=spend_err_roll_interior[0];
			
			errP_int=spend_err_roll_interior[0];                            		//此时的偏差
			errI_int=spend_err_roll_interior[0]+spend_err_roll_interior[2];    	//历史以来所有的偏差
			errD_int=spend_err_roll_interior[0]-spend_err_roll_interior[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
//			roll_PWM += (float)(errP_int*interior_KP_f+errI_int*interior_KI_f+errD_int*interior_KD_f);
			roll_PWM = (float)(errP_int*interior_KP_f+errI_int*interior_KI_f+errD_int*interior_KD_f);
			

			
			if(roll_PWM>=PWM_MAX )
			{
					roll_PWM=PWM_MAX ;
			}
			else if(roll_PWM<=PWM_MIN)
			{
					roll_PWM=PWM_MIN;
			}
	    roll_PWM_OUT=(short)(roll_PWM+0.5);//类型转换加校正
		
}

/*横滚PID*/
static float spend_err_pitch_external[3] = {0,0,0};
static float spend_err_pitch_interior[3] = {0,0,0};

void PID_pitch_Control( float pitch_set)
{
      //外环
			static float pitch_external;

			static float  errP = 0,errD = 0,errI = 0;
	 
			//内环
//			static int pitch_interior;

			static float errP_int = 0,errD_int = 0,errI_int = 0;
			
	    pitch_set+=(float)(pitch_IN*0.01); //横滚
	
			if(pitch_set>SET_MAX)
			{pitch_set=SET_MAX;ERR_STOP=1;OLED_ShowString(16,6,"LOCK");}
			if(pitch_set<(SET_MIN))
			{pitch_set=(SET_MIN);ERR_STOP=1;OLED_ShowString(16,6,"LOCK");}
			
			
			if(clean_value)    //积分清零标志位
			{
					clean_value=0;
					pitch_external=0;
					spend_err_pitch_external[2]=0;
					spend_err_pitch_interior[2]=0;
					spend_err_pitch_interior[1]=0;
					spend_err_pitch_interior[0]=0;
					pitch_PWM=0;
			}
			
			
	    
			spend_err_pitch_external[2] = spend_err_pitch_external[1]+spend_err_pitch_external[2]; 	//历史所有的偏差I 积分
			spend_err_pitch_external[1] = spend_err_pitch_external[0];             	//保存上一次偏差 D微分
//			                                      cheshi3=averag_gyro_OUT_Y(pitch_set);//(float)pitch_set ;					//偏差值=期望值-现实值P
			                                      cheshi3=-pitch_set;
			
			spend_err_pitch_external[0] = cheshi3;

			
			if(spend_err_pitch_external[2] >  20.0) spend_err_pitch_external[2] = 20.0;
			if(spend_err_pitch_external[2] < -20.0) spend_err_pitch_external[2] = -20.0;   //积分限幅
			
			
			errP=spend_err_pitch_external[0];                            		//此时的偏差
			errI=spend_err_pitch_external[0]+spend_err_pitch_external[1]+spend_err_pitch_external[2];    	//历史以来所有的偏差
			errD=spend_err_pitch_external[0]-spend_err_pitch_external[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
			pitch_external = (float)(errP*external_KP_f+errI*external_KI_f+errD*external_KD_f);        //PID计算 输出控制量
			
//			pitch_external
//      cheshi2= pitch_external;
	 /*控制量限幅*/
	 
			spend_err_pitch_interior[2] += spend_err_pitch_interior[0]; 	//历史所有的偏差
			spend_err_pitch_interior[1] = spend_err_pitch_interior[0];             	//保存上一次偏差
			
			
			spend_err_pitch_interior[0] = pitch_external - averag_gyro_Y(value_gyro.Y);					//与角速度做差
			
			if(spend_err_pitch_interior[2] >  20) spend_err_pitch_interior[2] = 20;
			if(spend_err_pitch_interior[2] < -20) spend_err_pitch_interior[2] = -20;   //积分限幅
			
//       cheshi3=spend_err_pitch_interior[0];
			
			errP_int=spend_err_pitch_interior[0];                            		//此时的偏差
			errI_int=spend_err_pitch_interior[0]+spend_err_pitch_interior[2];    	//历史以来所有的偏差
			errD_int=spend_err_pitch_interior[0]-spend_err_pitch_interior[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
//			pitch_PWM += (float)(errP_int*interior_KP_f+errI_int*interior_KI_f+errD_int*interior_KD_f);
			pitch_PWM = (float)(errP_int*interior_KP_f+errI_int*interior_KI_f+errD_int*interior_KD_f);
	 
		  if(pitch_PWM>=PWM_MAX )
			{
					pitch_PWM=PWM_MAX ;
			}
			else if(pitch_PWM<=PWM_MIN)
			{
					pitch_PWM=PWM_MIN;
			}
			
			 pitch_PWM_OUT=(short)(pitch_PWM+0.5);
}

/*偏航PID*/
static float spend_err_yaw_external[3] = {0,0,0};
static float spend_err_yaw_interior[3] = {0,0,0};

void PID_yaw_Control( float yaw_set)
{
//      //外环
//			static int yaw_external;
//			static int spend_err_yaw_external[3] = {0,0,0};
//			static int  errP = 0,errD = 0,errI = 0;
//	 
//			//内环
////			static int yaw_interior;
//			static float spend_err_yaw_interior[3] = {0,0,0};
//			static float errP_int = 0,errD_int = 0,errI_int = 0;
//	 
//			spend_err_yaw_external[2] = spend_err_yaw_external[1]+spend_err_yaw_external[2]; 	//历史所有的偏差
//			spend_err_yaw_external[1] = spend_err_yaw_external[0];             	//保存上一次偏差
//			spend_err_yaw_external[0] =  yaw_set ;					//偏差值=期望值-现实值

//			errP=spend_err_yaw_external[0];                            		//此时的偏差
//			errI=spend_err_yaw_external[0]+spend_err_yaw_external[1]+spend_err_yaw_external[2];    	//历史以来所有的偏差
//			errD=spend_err_yaw_external[0]-spend_err_yaw_external[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
//			yaw_external = (int)(errP*external_KP_f+errI*external_KI_f+errD*external_KD_f);;        //PID计算 输出控制量
//   
//	 /*控制量限幅*/
//	 
//			spend_err_yaw_interior[2] = spend_err_yaw_interior[1]+spend_err_yaw_interior[2]; 	//历史所有的偏差
//			spend_err_yaw_interior[1] = spend_err_yaw_interior[0];             	//保存上一次偏差
//			spend_err_yaw_interior[0] =  yaw_external ;					//输入角速度

//			errP_int=spend_err_yaw_interior[0];                            		//此时的偏差
//			errI_int=spend_err_yaw_interior[0]+spend_err_yaw_interior[1]+spend_err_yaw_interior[2];    	//历史以来所有的偏差
//			errD_int=spend_err_yaw_interior[0]-spend_err_yaw_interior[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
//			yaw_PWM = (int)(errP_int*interior_KP_f+errI_int*interior_KI_f+errD_int*interior_KD_f);
	 
	 
	       //外环
			static float yaw_external;

			static float  errP = 0,errD = 0,errI = 0;
	 
			//内环
//			static int yaw_interior;

			static float errP_int = 0,errD_int = 0,errI_int = 0;
			
//			yaw_set+=(float)(yaw_IN*0.01); //俯仰
	
//			if(yaw_set>SET_MAX)
//			{yaw_set=SET_MAX;ERR_STOP=1;OLED_ShowString(16,6,"LOCK");}
//			if(yaw_set<(SET_MIN))
//			{yaw_set=(SET_MIN);ERR_STOP=1;OLED_ShowString(16,6,"LOCK");}
//			if(ERR_STOP)
//				OLED_ShowString(16,6,"LOCK"); //锁定
//			else
//				OLED_ShowString(32,6,"     ");
			
			if(clean_value)    //积分清零标志位
			{
//					clean_value=0;
					yaw_external=0;
					spend_err_yaw_external[2]=0;
					spend_err_yaw_interior[2]=0;
					spend_err_yaw_interior[1]=0;
					spend_err_yaw_interior[0]=0;
					yaw_PWM=0;
			}
			
			
			
//	    cheshi1=averag_gyro_OUT_Y(yaw_set);
			
			
	 
//			cheshi=yaw_set;
//			 cheshi1=averag_gyro_OUT_X(yaw_set);
//			 cheshi1=yaw_set;//
			if(yaw_set>START_YAW+180.0)
			{
					yaw_set=yaw_set-START_YAW-360.0;
			}
			else if(yaw_set<START_YAW-180.0)
			{
					yaw_set=360.0-yaw_set+START_YAW;
			}
			else
			{
					yaw_set= yaw_set - START_YAW;//偏航偏差
			}
			
			
			cheshi2=yaw_set;
			spend_err_yaw_external[2] = spend_err_yaw_external[1]+spend_err_yaw_external[2]; 	//历史所有的偏差
			spend_err_yaw_external[1] = spend_err_yaw_external[0];             	//保存上一次偏差
			spend_err_yaw_external[0] = yaw_set;					//偏差值=期望值-现实值

			
			if(spend_err_yaw_external[2] >  20) spend_err_yaw_external[2] = 20;
			if(spend_err_yaw_external[2] < -20) spend_err_yaw_external[2] = -20;   //积分限幅
			
			
			errP=spend_err_yaw_external[0];                            		//此时的偏差
			errI=spend_err_yaw_external[0]+spend_err_yaw_external[1]+spend_err_yaw_external[2];    	//历史以来所有的偏差
			errD=spend_err_yaw_external[0]-spend_err_yaw_external[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
			yaw_external = (float)(errP*YAWexternal_KP_f+errI*YAWexternal_KI_f+errD*YAWexternal_KD_f);        //PID计算 输出控制量
			
//			yaw_external
//      cheshi2= yaw_external;
	 /*控制量限幅*/
	 
			spend_err_yaw_interior[2] += spend_err_yaw_interior[0]; 	//历史所有的偏差
			spend_err_yaw_interior[1] = spend_err_yaw_interior[0];             	//保存上一次偏差
			
//			cheshi4=averag_gyro_X(value_gyro.X);
////				cheshi4=value_gyro.X;
//			                                     cheshi2= yaw_external - cheshi4;//cheshi4;					//与角速度做差
			spend_err_yaw_interior[0] =yaw_external - averag_gyro_Z(value_gyro.Z);;
			

			
			if(spend_err_yaw_interior[2] >  20.0) spend_err_yaw_interior[2] = 20.0;
			if(spend_err_yaw_interior[2] < -20.0) spend_err_yaw_interior[2] = -20.0;   //积分限幅
			
//       cheshi3=spend_err_yaw_interior[0];
			
			errP_int=spend_err_yaw_interior[0];                            		//此时的偏差
			errI_int=spend_err_yaw_interior[0]+spend_err_yaw_interior[2];    	//历史以来所有的偏差
			errD_int=spend_err_yaw_interior[0]-spend_err_yaw_interior[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
			yaw_PWM = (float)(errP_int*YAWinterior_KP_f+errI_int*YAWinterior_KI_f+errD_int*YAWinterior_KD_f);
			

			
			if(yaw_PWM>=PWM_MAX )
			{
					yaw_PWM=PWM_MAX ;
			}
			else if(yaw_PWM<=PWM_MIN)
			{
					yaw_PWM=PWM_MIN;
			}
	    yaw_PWM_OUT=(short)(yaw_PWM+0.5);//类型转换加校正
		
}

extern uint8_t TIME_20ms;
void PID_control(void)
{   
		extern T_float_angle angle;
	
		PID_roll_Control	(angle.roll);
		PID_pitch_Control (angle.pitch);
		PID_yaw_Control		(angle.yaw); 
//		send_line(cheshi1*10,-cheshi4*10,roll_PWM*10,cheshi2*10,cheshi3*10,pitch_PWM*10);//回传PID数据 山外
	
			
		if(TIME_20ms )
		{
				TIME_20ms=0;
			  send_line(cheshi1*10,cheshi2,value_gyro.X,yaw_PWM*10,START_YAW,value_gyro.Z);//pitch_PWM*10);//回传PID数据 山外
		}
}

extern uint8_t POWER_set;//电源设置标志位

void PWM_control(void)
{
		if(accelerator_IN>10&&!POWER_set&&!ERR_STOP)
		{
//				TIM3_Mode_PWM1_Config(2050+accelerator_IN+pitch_PWM_OUT+roll_PWM_OUT);//1950起飞油门		
//				TIM3_Mode_PWM2_Config(2050+accelerator_IN-pitch_PWM_OUT-roll_PWM_OUT);
//				TIM3_Mode_PWM3_Config(2050+accelerator_IN-pitch_PWM_OUT+roll_PWM_OUT);
//				TIM3_Mode_PWM4_Config(2050+accelerator_IN+pitch_PWM_OUT-roll_PWM_OUT);
//					TIM3->CCR1=1950;
//					TIM3->CCR2=1950;
//					TIM3->CCR1=(uint16_t)(2150+accelerator_IN+roll_PWM_OUT+pitch_PWM_OUT);
//					TIM3->CCR2=(uint16_t)(2150+accelerator_IN-roll_PWM_OUT-pitch_PWM_OUT);
//					TIM3->CCR3=(uint16_t)(2150+accelerator_IN+roll_PWM_OUT-pitch_PWM_OUT);
//					TIM3->CCR4=(uint16_t)(2150+accelerator_IN-roll_PWM_OUT+pitch_PWM_OUT);
					TIM3->CCR1=(uint16_t)(2150+accelerator_IN+(short)(pitch_PWM	+roll_PWM+yaw_PWM+0.5));
					TIM3->CCR2=(uint16_t)(2150+accelerator_IN+(short)(-pitch_PWM-roll_PWM+yaw_PWM+0.5));
					TIM3->CCR3=(uint16_t)(2150+accelerator_IN+(short)(-pitch_PWM+roll_PWM-yaw_PWM+0.5));
					TIM3->CCR4=(uint16_t)(2150+accelerator_IN+(short)(pitch_PWM	-roll_PWM-yaw_PWM+0.5));
//					TIM3->CCR1=2150+accelerator_IN+roll_PWM_OUT;
//					TIM3->CCR2=2150+accelerator_IN-roll_PWM_OUT;
//					TIM3->CCR3=2150+accelerator_IN+roll_PWM_OUT;
//					TIM3->CCR4=2150+accelerator_IN-roll_PWM_OUT;
//					TIM3->CCR1=2150+accelerator_IN+pitch_PWM_OUT;
//					TIM3->CCR2=2150+accelerator_IN-pitch_PWM_OUT;
//					TIM3->CCR3=2150+accelerator_IN-pitch_PWM_OUT;
//					TIM3->CCR4=2150+accelerator_IN+pitch_PWM_OUT;
	  }
		else
		{
//				TIM3_Mode_PWM1_Config(1950);	
//				TIM3_Mode_PWM2_Config(1950);
//				TIM3_Mode_PWM3_Config(1950);
//				TIM3_Mode_PWM4_Config(1950);
				TIM3->CCR1=1950;
				TIM3->CCR2=1950;
				TIM3->CCR3=1950;
				TIM3->CCR4=1950;
			  
			
		}
//		TIM3_Mode_PWM1_Config(2050+accelerator_IN+(int)roll_PWM_OUT);//1950起飞油门		
//		TIM3_Mode_PWM2_Config(2050+accelerator_IN-(int)roll_PWM_OUT);
//		TIM3_Mode_PWM3_Config(2050+accelerator_IN+(int)roll_PWM_OUT);
//		TIM3_Mode_PWM4_Config(2050+accelerator_IN-(int)roll_PWM_OUT);
	
	
}
/*		最 低 油 门 ：    1950          */
/*		最 高 油 门 ：    3900          */
/* 		起飞零点油门：   2460	/ 39999   */

/*****************************************************************************/
/*																		油门分配                                    */
/*		                                                                        */
/*		电机1=零点油门 + PID_roll平衡油门  -  PID_pit平衡油门  +  PID_yaw平衡油门   */
/*		电机2=零点油门 - PID_roll平衡油门  +  PID_pit平衡油门  +  PID_yaw平衡油门   */
/*		电机3=零点油门 + PID_roll平衡油门  +  PID_pit平衡油门  -  PID_yaw平衡油门   */
/*		电机4=零点油门 - PID_roll平衡油门  -  PID_pit平衡油门  -  PID_yaw平衡油门   */
/*                                                                           */
/*****************************************************************************/

/*********PITCH************/
uint8_t numG=0;
float last_value_Y[]={0,0,0,0,0,0,0,0};

float averag_gyro_Y(float value)
{
		numG++;
		if(numG==8) numG=0;	
		last_value_Y[numG]=value;
		value=0;
		value=(last_value_Y[0]+last_value_Y[1]+last_value_Y[2]+last_value_Y[3]+last_value_Y[4]+last_value_Y[5]+last_value_Y[6]+last_value_Y[7])/8.0;
	
		return value;
}


uint8_t numOUTG=0;
float last_value_OUTY[]={0,0,0,0,0,0,0,0,0};

float averag_gyro_OUT_Y(float value)
{
		uint8_t i;
		numOUTG++;
		if(numOUTG==5) numOUTG=0;	
		last_value_OUTY[numOUTG]=value;
		
//		value=(float)(last_value_OUTY[0]+last_value_OUTY[1]+last_value_OUTY[2]+last_value_OUTY[3]+last_value_OUTY[4])/5;
	  value=0;
	  for(i=0;i<5;i++)
	  {
				value+=last_value_OUTY[i];
		}
		value/=5.0;
	
		return -value;
}
/*********ROLL************/
uint8_t numG_roll=0;
float last_value_X[]={0,0,0,0,0,0,0,0};

float averag_gyro_X(float value)
{
		numG_roll++;
		if(numG_roll==8) numG_roll=0;	
		last_value_X[numG_roll]=value;
		value=0;
		value=(last_value_X[0]+last_value_X[1]+last_value_X[2]+last_value_X[3]+last_value_X[4]+last_value_X[5]+last_value_X[6]+last_value_X[7])/8.0;
	
		return value;
}


uint8_t numOUTG_roll=0;
float last_value_OUTX[]={0,0,0,0,0,0,0,0,0};

float averag_gyro_OUT_X(float value)
{
		uint8_t i;
		numOUTG_roll++;
		if(numOUTG_roll==5) numOUTG_roll=0;	
		last_value_OUTX[numOUTG_roll]=value;
		
//		value=(float)(last_value_OUTX[0]+last_value_OUTX[1]+last_value_OUTX[2]+last_value_OUTX[3]+last_value_OUTX[4])/5;
	  value=0;
	  for(i=0;i<5;i++)
	  {
				value+=last_value_OUTX[i];
		}
		value/=5.0;
	
		return -value;
}

/*****************YAW***************/
uint8_t numG_YAW=0;
float last_value_Z[]={0,0,0,0,0,0,0,0};

float averag_gyro_Z(float value)
{
		numG_YAW++;
		if(numG_YAW==8) numG_YAW=0;	
		last_value_Z[numG_YAW]=value;
		value=0;
		value=(last_value_Z[0]+last_value_Z[1]+last_value_Z[2]+last_value_Z[3]+last_value_Z[4]+last_value_Z[5]+last_value_Z[6]+last_value_Z[7])/8.0;
	
		return value;
}

uint8_t numYAWOUTG=0;
float last_value_OUTZ[]={0,0,0,0,0,0,0,0,0};

float averag_gyro_OUT_Z(float value)
{
		uint8_t i;
		numYAWOUTG++;
		if(numYAWOUTG==5) numYAWOUTG=0;	
		last_value_OUTZ[numYAWOUTG]=value;
		
//		value=(float)(last_value_OUTY[0]+last_value_OUTY[1]+last_value_OUTY[2]+last_value_OUTY[3]+last_value_OUTY[4])/5;
	  value=0;
	  for(i=0;i<5;i++)
	  {
				value+=last_value_OUTZ[i];
		}
		value/=5.0;
	
		return -value;
}

//void PID_Control(float P,float I,float D,int left_set)

//{
//   static int left_spend;
//   static int spend_err[3] = {0,0,0};
//   static int  errP = 0,errD = 0,errI = 0;
//   spend_err[2] = spend_err[1]+spend_err[2]; 	//历史所有的偏差
//   spend_err[1] = spend_err[0];             	//保存上一次偏差
////   spend_err[0] =  - ;					//偏差值=期望值-现实值

//   errP=spend_err[0];                            		//此时的偏差
//   errI=spend_err[0]+spend_err[1]+spend_err[2];    	//历史以来所有的偏差
//   errD=spend_err[0]-spend_err[1];                  //此时的偏差与上一次偏差之差   两个偏差之比
//   left_spend = (int)(errP*P+errI*I+errD*D);        //PID计算 输出控制量
//   
//	 /*控制量限幅*/
//}
