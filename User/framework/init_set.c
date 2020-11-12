#include "init_set.h"
#include "pwm_config.h"
#include "PID_control.h"


//����****************
extern uint8_t  frame_flag;
extern uint8_t  isr_cnt;
extern uint8_t  frame_cnt;

extern uint8_t clean_value;//���������־λ

uint8_t key_val; //�洢���յ�����
uint8_t screen;//��ʾ��־λ

extern int8_t ERR_STOP;     //��б����

/************************/
/*********���˵�**********/
/************************/
uint8_t POWER_set=0;//��Դ���ñ�־λ

void set_data(void)
{    	
		
		uint8_t mode_set;//ģʽѡ��
	
		
	
		if( frame_flag == 1 )      // һ֡�������ݽ������ 
		{
			  key_val = IrDa_Process();
				if(key_val==RED_POWER)
				{
					POWER_set=1;//��Դ���ô�
					mode_set=1;
					screen=0;//��ʾ������
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , DISABLE); /* TIM6 ���¿�ʱ�ӣ���ʼ��ʱ */
					TIM3_Mode_PWM1_Config(1950);
					TIM3_Mode_PWM2_Config(1950);
					TIM3_Mode_PWM3_Config(1950);
					TIM3_Mode_PWM4_Config(1950);
					
				}					
				while(POWER_set)//POWER 
				{
					
					if(!screen)//��ʾ
					{
						  screen=1; //��ʾ
						
							OLED_Clear();//����
						
							OLED_ShowCHinese(40,0,21);//�� 
							OLED_ShowCHinese(72,0,22);//��
							 
							OLED_ShowCHinese(32,2,23);//�� 
							OLED_ShowCHinese(48,2,24);//��
							OLED_ShowCHinese(64,2,21);//�� 
							OLED_ShowCHinese(80,2,22);//��
							 
							OLED_ShowCHinese(32,4,25);//�� 
							OLED_ShowCHinese(48,4,26);//��
							OLED_ShowCHinese(64,4,21);//�� 
							OLED_ShowCHinese(80,4,22);//��
							
							OLED_ShowCHinese(32,6,27);//�� 
							OLED_ShowCHinese(48,6,28);//��
							OLED_ShowCHinese(64,6,21);//�� 
							OLED_ShowCHinese(80,6,22);//��
							
							//��ʾ���ڵ�ѡ��
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
					
					if( frame_flag == 1 )      // һ֡�������ݽ������
					{	
						key_val = IrDa_Process();//��ȡ����
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
												
						else if(key_val==RED_OK)// ">"  ����
						{
							  if(mode_set==1)    //����ģʽ1����
								{
										MENU_1();//������һ���˵�1
								}
								else if(mode_set==2)   //����ģʽ2����
								{
										MENU_2();//������һ���˵�2									
								}
								else if(mode_set==3)   //����ģʽ3����
								{
										
									  MENU_3();//������һ���˵�3	
								}
						}					
					
						if(key_val==RED_RETURN)//RETURN����
						{
							  OLED_Clear();//����
								OLED_ShowCHinese(40,3,21);//�� 
								OLED_ShowCHinese(72,3,22);//��
								if(ERR_STOP)
									OLED_ShowString(32,6,"LOCK"); //����
								screen=0;//��ʾ���
								POWER_set=0;
							  key_val=255;
								clean_value=1;//���������־λ
							
								RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE); /* TIM6 ���¿�ʱ�ӣ���ʼ��ʱ */
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
/*******��һ���˵� 1 ********/
/***************************/

void MENU_1(void)
{
		uint8_t mode_set_1; //��һ���˵� 1 ��־λ
									
		mode_set_1=1;
		screen=0;//��ʾ���
								
		 while(mode_set_1)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowString(32,0,"PID_SET");//��ʾPID
						OLED_ShowString(16,2,"INT_PID");//��ʾPID
						OLED_ShowString(16,4,"OUT_PID");//��ʾPID
						OLED_ShowString(16,6,"YAW_PID");//��ʾPID
					
						//��ʾ���ڵ�ѡ��
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
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
					
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
					  else if(key_val==RED_OK)// ">"  ����
						{
							  if(mode_set_1==1)    //����ģʽ1����
								{
										MENU_1_1();
								}
								else if(mode_set_1==2)   //����ģʽ2����
								{
										MENU_1_2();									
								}
								else if(mode_set_1==3)   //����ģʽ2����
								{
										MENU_1_3();									
								}/**/								
						}
					
					
						if(key_val==RED_RETURN)//RETURN����
						{						
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_1=0;
							  key_val=255;							
								while(frame_flag)//�˳���һ���˵� ��������
								{
									
									frame_flag = 0;//��־λ0
									Delay_us(800000);
								}
								key_val=255;
								
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
/*******��һ���˵� 2 ********/
/***************************/


void MENU_2(void)
{
		uint8_t mode_set_2; //��һ���˵� 2 ��־λ
									
		mode_set_2=1;
		screen=0;//��ʾ���
								
		 while(mode_set_2)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowCHinese(32,0,25);//�� 
						OLED_ShowCHinese(48,0,26);//��
						OLED_ShowCHinese(64,0,21);//�� 
						OLED_ShowCHinese(80,0,22);//��	
				}
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
					key_val = IrDa_Process();//��ȡ����				
				
						if(key_val==RED_RETURN)//RETURN����
						{						
								screen=0;//��ʾ���
								mode_set_2=0; //ģʽ����	
							
								while(frame_flag)//�˳���һ���˵� ��������
								{
									
									frame_flag = 0;//��־λ0
									
									Delay_us(800000);
								}
								key_val=255;
								ERR_STOP=0;//��̬���� ͣ������ʱ����
						}
				}
		 }
}

/***************************/
/*******��һ���˵� 3 ********/
/***************************/


/*

*/

void MENU_3(void)
{
	  uint8_t mode_set_3; //������һ���˵� 311 ��־λ
									
		mode_set_3=1; //Ĭ�ϵ�һ��ģʽ���شų�У׼��
		screen=0;//��ʾ���
								
		 while(mode_set_3)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
					
						OLED_ShowCHinese(48,0,30);//У
						OLED_ShowCHinese(64,0,31);//׼

						OLED_ShowCHinese(24,3,27);//�� 
						OLED_ShowCHinese(40,3,28);//��
						OLED_ShowCHinese(56,3,29);//��
						OLED_ShowCHinese(72,3,30);//У 
						OLED_ShowCHinese(88,3,31);//׼

						OLED_ShowCHinese(24,5,32);//�� 
						OLED_ShowCHinese(40,5,33);//��
						OLED_ShowCHinese(56,5,34);//��
						OLED_ShowCHinese(72,5,30);//У 
						OLED_ShowCHinese(88,5,31);//׼

						//��ʾ���ڵ�ѡ��
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
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
				
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
												
						else if(key_val==RED_OK)// ">"  ����
						{   
								RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE); /* TIM6 ���¿�ʱ�ӣ���ʼ��ʱ */
									
							  if(mode_set_3==1)    //����ģʽ1����
								{
									
									MENU_3_1();
								}
								else if(mode_set_3==2)   //����ģʽ2����
								{
										MENU_3_2();									
								}								
						}					
					
						if(key_val==RED_RETURN)//RETURN����
						{
							  OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_3=0;
							  key_val=255;
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
		 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , DISABLE); /* TIM6 ���¿�ʱ�ӣ���ʼ��ʱ */
}

/******************************/
/*******�Ӷ����˵� 3-1 ********/
/******************************/

#include "data_exchang.h"
#include "AHRS.h"
#include "FLASH_RW.h"

void MENU_3_1(void)
{
		uint8_t mode_set_3_1; //����һ���˵� 31 ��־λ
		extern uint8_t AGM_START;//�ǶȲɼ���־λ
    extern short Mag[3];
		extern short Xscope,Yscope,Zscope;  //��Χ
	  extern T_float_xyz Mag_corr;//У׼��
	
		extern int FLASH_MAG_X_DATA;
		extern int FLASH_MAG_Y_DATA;          //�ش�ԭ��У׼
		extern int FLASH_MAG_Z_DATA;
	
	  extern short Xoffset,Yoffset,Zoffset;  //����
	
		uint8_t scope;
	
		mode_set_3_1=1; //û��ѡ��ֻ��У׼��ɽ��뱣��ѡ�񣬻���ֱ���˳�
		screen=0;//��ʾ���
		
			Xscope=0;
			Yscope=0;
			Zscope=0;
	
			Mag_corr.X=0;
			Mag_corr.Y=0;          //��ŵ���������
			Mag_corr.Z=0;
	
		 while(mode_set_3_1)
		 {			 							 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowCHinese(24,0,27);//�� 
						OLED_ShowCHinese(40,0,28);//��
						OLED_ShowCHinese(56,0,29);//��
						OLED_ShowCHinese(72,0,30);//У 
						OLED_ShowCHinese(88,0,31);//׼

				}
				
			   /*������ɼ��������������*/
				
				if(AGM_START) //50ms�ɼ�һ��
				{
						AGM_START=0;//�����־λ
						MPU9250ReadMag(Mag);//�ɼ�����
						calibration_Mag(Mag[0],Mag[1],Mag[2]); //���������У׼
											  
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
				
				 
				
				if((Xscope>=120)&&(Yscope>=120)&&(Zscope>=120))//RETURN����  
				{
						complete_data();//���뱣�����
						
						OLED_Clear();//����
						screen=0;//��ʾ���
						mode_set_3_1=0;	
						key_val=255;
				}  
								
				
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
						
				/************************************/
            if(key_val==RED_MENU)//menuģ��ɼ����
						{
								complete_data();//���뱣�����
							
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_3_1=0;	
							  key_val=255;
            }
				/************************************/
						
						if(key_val==RED_RETURN)//RETURN����
						{
							  OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_3_1=0;

								read_flash_data();   //��ȡ����									
								Xoffset=FLASH_MAG_X_DATA;
								Yoffset=FLASH_MAG_Y_DATA;          //��ŵ���������
								Zoffset=FLASH_MAG_Z_DATA;	
							
							  key_val=255;
						}
				}
		 }
}

/******************************/
/*******�Ӷ����˵� 3-2 ********/
/******************************/

#include "data_exchang.h"
#include "AHRS.h"
#include "FLASH_RW.h"

void MENU_3_2(void)
{
		uint8_t mode_set_3_2; //����һ���˵� 31 ��־λ
		extern uint8_t AGM_START;//�ǶȲɼ���־λ
    extern short Acel[3];    // ���ٶ�
    extern short Gyro[3];    // ���ٶ�

	  extern short offset_GX,offset_GY,offset_GZ;  //����
		extern short offset_AX,offset_AY,offset_AZ;  //����
	
		extern int FLASH_Gyro_X_DATA;
		extern int FLASH_Gyro_Y_DATA;         //�Ƕ�ԭ��У׼
		extern int FLASH_Gyro_Z_DATA;

		extern int FLASH_Acel_X_DATA;
		extern int FLASH_Acel_Y_DATA;         //���ٶ�ԭ��У׼
		extern int FLASH_Acel_Z_DATA;
	
		extern uint8_t calibration_GA_time; //У׼�ɼ�����
	
		mode_set_3_2=1; //û��ѡ��ֻ��У׼��ɽ��뱣��ѡ�񣬻���ֱ���˳�
		screen=0;//��ʾ���
		calibration_GA_time=0;//����������
		
		offset_GX=0;
		offset_GY=0;
		offset_GZ=0;
		offset_AX=0;
		offset_AY=0;
		offset_AZ=0;
			
		 while(mode_set_3_2)
		 {			 							 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowCHinese(24,0,32);//�� 
						OLED_ShowCHinese(40,0,33);//��
						OLED_ShowCHinese(56,0,34);//��
						OLED_ShowCHinese(72,0,30);//У 
						OLED_ShowCHinese(88,0,31);//׼

				}
				
			   /*������ɼ��������������*/
				
				if(AGM_START) //20ms�ɼ�һ��
				{
						AGM_START=0;//�����־λ
					
						MPU6050ReadAcc(Acel);	//�ɼ�����
						MPU6050ReadGyro(Gyro);//�ɼ�����
						
						calibration_GA(Gyro[0],Gyro[1],Gyro[2],Acel[0],Acel[1],Acel[2]);		
					
						OLED_ShowNum(64,6,calibration_GA_time,3,16);    //��ɶ�
				}
				
				if(calibration_GA_time==255) //У׼����
				{
						/*��ʾ����*/
					
						OLED_Clear();//����
						screen=0;//��ʾ���
						mode_set_3_2=0;
					
						read_flash_data();   //��ȡ����									
						offset_GX=FLASH_Gyro_X_DATA;
						offset_GY=FLASH_Gyro_Y_DATA;         //�Ƕ�ԭ��У׼
						offset_GZ=FLASH_Gyro_Z_DATA;

						offset_AX=FLASH_Acel_X_DATA;
						offset_AY=FLASH_Acel_Y_DATA;         //���ٶ�ԭ��У׼
						offset_AZ=FLASH_Acel_Z_DATA;
				}
				
				if(calibration_GA_time==100)//RETURN����  
				{
						complete_GA_data();//���뱣�����
						
						OLED_Clear();//����
						screen=0;//��ʾ���
						mode_set_3_2=0;	
						key_val=255;
				}  
								
				
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
						
				/************************************/
            if(key_val==RED_MENU)//menuģ��ɼ����
						{
								complete_GA_data();//���뱣�����
							
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_3_2=0;	
							  key_val=255;
            }
				/************************************/
						
						if(key_val==RED_RETURN)//RETURN����
						{
							  OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_3_2=0;

								read_flash_data();   //��ȡ����									
								offset_GX=FLASH_Gyro_X_DATA;
								offset_GY=FLASH_Gyro_Y_DATA;         //�Ƕ�ԭ��У׼
								offset_GZ=FLASH_Gyro_Z_DATA;

								offset_AX=FLASH_Acel_X_DATA;
								offset_AY=FLASH_Acel_Y_DATA;         //���ٶ�ԭ��У׼
								offset_AZ=FLASH_Acel_Z_DATA;	
							
							  key_val=255;
						}
				}
		 }
}





/*********************************/
/**У׼�ɼ�����Զ�����ȷ��ѡ����***/
/******* �������˵� 3-1-1  ********/
/*********************************/




void complete_data(void)//������� �Ƿ񱣴�����
{
		uint8_t mode_set_3_1_1; //������һ���˵� 311 ��־λ
	
		extern int FLASH_MAG_X_DATA;
		extern int FLASH_MAG_Y_DATA;          //�ش�ԭ��У׼
		extern int FLASH_MAG_Z_DATA;
	
//		extern T_float_xyz Mag_corr;//У׼��
		extern short Xoffset,Yoffset,Zoffset;  //����
									
		mode_set_3_1_1=2; //Ĭ�ϵڶ���ģʽ��ȡ����
		screen=0;//��ʾ���
								
		 while(mode_set_3_1_1)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowCHinese(24,0,27);//�� 
						OLED_ShowCHinese(40,0,28);//��
						OLED_ShowCHinese(56,0,29);//��
						OLED_ShowCHinese(72,0,30);//У 
						OLED_ShowCHinese(88,0,31);//׼

						//��ʾ���ڵ�ѡ��
						switch( mode_set_3_1_1 )
						{
							case 1:
									OLED_ShowCHinese_inverse(24,3,35);//����ɫ 
									OLED_ShowCHinese_inverse(40,3,36);//�淴ɫ
									OLED_ShowCHinese(56,3,37);//ȡ
									OLED_ShowCHinese(72,3,38);//�� 
							break;
							
							case 2:
									OLED_ShowCHinese(24,3,35);//�� 
									OLED_ShowCHinese(40,3,36);//��
									OLED_ShowCHinese_inverse(56,3,37);//ȡ
									OLED_ShowCHinese_inverse(72,3,38);//��
							break;
							
							default:       
							break;
						}
				}
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
				
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
												
						else if(key_val==RED_OK)// ">"  ����
						{
							  if(mode_set_3_1_1==1)    //����ģʽ1����
								{
										/*ִ��ѡ��1*/
									
//									  OLED_ShowString(32,2,"reading");
									  printf("������%d,\t%d,\t%d\n\n",Xoffset,Yoffset,Zoffset);
									  read_flash_data();   //��ȡ����									
									  FLASH_MAG_X_DATA=Xoffset;  //����;
										FLASH_MAG_Y_DATA=Yoffset;          //��ŵ���������
										FLASH_MAG_Z_DATA=Zoffset;										
										write_flash_data();  //д������
									
										mode_set_3_1_1=0;
								}
								else if(mode_set_3_1_1==2)   //����ģʽ2����
								{
										/*ִ��ѡ��2*/
										mode_set_3_1_1=0;
								}								
						}					
					
						if(key_val==RED_RETURN)//RETURN����
						{
							  OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_3_1_1=0;
								key_val=255;
						}
						else
						{
								//��ʾ���ڵ�ѡ��
								switch( mode_set_3_1_1 )
								{
									case 1:
											OLED_ShowCHinese_inverse(24,3,35);//����ɫ 
											OLED_ShowCHinese_inverse(40,3,36);//�淴ɫ
											OLED_ShowCHinese(56,3,37);//ȡ
											OLED_ShowCHinese(72,3,38);//�� 
									break;
									
									case 2:
											OLED_ShowCHinese(24,3,35);//�� 
											OLED_ShowCHinese(40,3,36);//��
											OLED_ShowCHinese_inverse(56,3,37);//ȡ
									    OLED_ShowCHinese_inverse(72,3,38);//��
									break;
									
									default:       
									break;
								}
						}
				}
		 }
}


/*********************************/
/**У׼�ɼ�����Զ�����ȷ��ѡ����***/
/******* �������˵� 3-2-2  ********/
/*********************************/




void complete_GA_data(void)//������� �Ƿ񱣴�����
{
		uint8_t mode_set_3_2_2; //������һ���˵� 311 ��־λ
	
		extern int FLASH_Gyro_X_DATA;
		extern int FLASH_Gyro_Y_DATA;         //�Ƕ�ԭ��У׼
		extern int FLASH_Gyro_Z_DATA;

		extern int FLASH_Acel_X_DATA;
		extern int FLASH_Acel_Y_DATA;         //���ٶ�ԭ��У׼
		extern int FLASH_Acel_Z_DATA;
	
	  extern short offset_GX,offset_GY,offset_GZ;  //����
		extern short offset_AX,offset_AY,offset_AZ;  //����
	
									
		mode_set_3_2_2=2; //Ĭ�ϵڶ���ģʽ��ȡ����
		screen=0;//��ʾ���
								
		 while(mode_set_3_2_2)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowCHinese(24,0,32);//�� 
						OLED_ShowCHinese(40,0,33);//��
						OLED_ShowCHinese(56,0,34);//��
						OLED_ShowCHinese(72,0,30);//У 
						OLED_ShowCHinese(88,0,31);//׼

						//��ʾ���ڵ�ѡ��
						switch( mode_set_3_2_2 )
						{
							case 1:
									OLED_ShowCHinese_inverse(24,3,35);//����ɫ 
									OLED_ShowCHinese_inverse(40,3,36);//�淴ɫ
									OLED_ShowCHinese(56,3,37);//ȡ
									OLED_ShowCHinese(72,3,38);//�� 
							break;
							
							case 2:
									OLED_ShowCHinese(24,3,35);//�� 
									OLED_ShowCHinese(40,3,36);//��
									OLED_ShowCHinese_inverse(56,3,37);//ȡ
									OLED_ShowCHinese_inverse(72,3,38);//��
							break;
							
							default:       
							break;
						}
				}
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
				
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
												
						else if(key_val==RED_OK)// ">"  ����
						{
							  if(mode_set_3_2_2==1)    //����ģʽ1����
								{
										/*ִ��ѡ��1*/
									
//									  OLED_ShowString(32,2,"reading");
//									  printf("�� �Ȳ�����%d,\t%d,\t%d\n\n",offset_GX,offset_GY,offset_GZ);
//										printf("���ٶȲ�����%d,\t%d,\t%d\n\n",offset_AX,offset_AY,offset_AZ);
									  read_flash_data();   //��ȡ����		
									
										FLASH_Gyro_X_DATA=offset_GX;
										FLASH_Gyro_Y_DATA=offset_GY;         //�Ƕ�ԭ��У׼
										FLASH_Gyro_Z_DATA=offset_GZ;

										FLASH_Acel_X_DATA=offset_AX;
										FLASH_Acel_Y_DATA=offset_AY;         //���ٶ�ԭ��У׼
										FLASH_Acel_Z_DATA=offset_AZ;								
										write_flash_data();  //д������
									
										mode_set_3_2_2=0;
								}
								else if(mode_set_3_2_2==2)   //����ģʽ2����
								{
										/*ִ��ѡ��2*/
										mode_set_3_2_2=0;
								}								
						}					
					
						if(key_val==RED_RETURN)//RETURN����
						{
							  OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_3_2_2=0;
								key_val=255;
						}
						else
						{
								//��ʾ���ڵ�ѡ��
								switch( mode_set_3_2_2 )
								{
									case 1:
											OLED_ShowCHinese_inverse(24,3,35);//����ɫ 
											OLED_ShowCHinese_inverse(40,3,36);//�淴ɫ
											OLED_ShowCHinese(56,3,37);//ȡ
											OLED_ShowCHinese(72,3,38);//�� 
									break;
									
									case 2:
											OLED_ShowCHinese(24,3,35);//�� 
											OLED_ShowCHinese(40,3,36);//��
											OLED_ShowCHinese_inverse(56,3,37);//ȡ
									    OLED_ShowCHinese_inverse(72,3,38);//��
									break;
									
									default:       
									break;
								}
						}
				}
		 }
}
/***************************/
/*******��һ���˵� 1-1 ******/
/***************************/

void MENU_1_1(void)
{
		uint8_t mode_set_1_1; //��һ���˵� 1 ��־λ
		
		extern int interior_KP; 			//�ڲ�
		extern int interior_KI;
		extern int interior_KD;
	
		extern int FLASH_INPID_P_DATA;
		extern int FLASH_INPID_I_DATA;       //�ڻ�PID
		extern int FLASH_INPID_D_DATA;

		extern float interior_KP_f; 			//�ڲ�
		extern float interior_KI_f;
		extern float interior_KD_f;
	
		mode_set_1_1=1;
		screen=0;//��ʾ���
		
		Flash_data_init(); //��ȡ�ڴ�����
	
		 while(mode_set_1_1)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowString(32,0,"PID INT");//��ʾ
					
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
						OLED_ShowNum(64,2,interior_KP,5,16);    //KP��ֵ
						OLED_ShowNum(64,4,interior_KI,5,16);    //KI��ֵ
						OLED_ShowNum(64,6,interior_KD,5,16);    //KD��ֵ
					
						/*��ʾ���ڵ�ѡ��*/		
				}
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
					
//					
					/*1��λ*/	
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
						/*ʮ��λ*/
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
						/*�ٽ�λ*/
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
						else if(key_val==RED_Right)//"��"
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
//					  else if(key_val==RED_OK)// ">"  ����
//						{
//							  if(mode_set_1_1==1)    //����ģʽ1����
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_1==2)   //����ģʽ2����
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_1==3)   //����ģʽ2����
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN����
						{						
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_1_1=0;
							  key_val=255;							
								while(frame_flag)//�˳���һ���˵� ��������
								{
									
									frame_flag = 0;//��־λ0
									Delay_us(800000);
								}
								key_val=255;
								
										FLASH_INPID_P_DATA=interior_KP;
										FLASH_INPID_I_DATA=interior_KI;      //�ڻ�PID
										FLASH_INPID_D_DATA=interior_KD;
								
										interior_KP_f=(float)interior_KP/1000; 			//�ڻ� ����
										interior_KI_f=(float)interior_KI/1000;
										interior_KD_f=(float)interior_KD/1000;
								
								write_flash_data();  //д������
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
								OLED_ShowNum(64,2,interior_KP,5,16);    //KP��ֵ99.999
								OLED_ShowNum(64,4,interior_KI,5,16);    //KI��ֵ
								OLED_ShowNum(64,6,interior_KD,5,16);    //KD��ֵ
						}
				}
		 }
}

/***************************/
/*******��һ���˵� 1-2 ******/
/***************************/

void MENU_1_2(void)
{
		uint8_t mode_set_1_2; //��һ���˵� 1 ��־λ
		
	  extern int external_KP; 			//�⻷
		extern int external_KI;
		extern int external_KD;

		extern int FLASH_OUTPID_P_DATA;
		extern int FLASH_OUTPID_I_DATA;     //�⻷PID
		extern int FLASH_OUTPID_D_DATA;
	
	  extern float external_KP_f; 			//�⻷
		extern float external_KI_f;
		extern float external_KD_f;
	
		mode_set_1_2=1;
		screen=0;//��ʾ���
		
		Flash_data_init(); //��ȡ�ڴ�����
	
		 while(mode_set_1_2)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowString(32,0,"PID OUT");//��ʾ
					
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
						OLED_ShowNum(64,2,external_KP,6,16);    //KP��ֵ
						OLED_ShowNum(64,4,external_KI,6,16);    //KI��ֵ
						OLED_ShowNum(48,6,external_KD,8,16);    //KD��ֵ
					
						/*��ʾ���ڵ�ѡ��*/		
				}
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
					
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
						/*�ٽ�λ*/
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
						/*ǧ��λ*/
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
						/*���λ*/
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
						}/*ʮ���λ*/
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
						
						else if(key_val==RED_Right)//"��"
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
//					  else if(key_val==RED_OK)// ">"  ����
//						{
//							  if(mode_set_1_2==1)    //����ģʽ1����
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_2==2)   //����ģʽ2����
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_2==3)   //����ģʽ2����
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN����
						{						
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_1_2=0;
							  key_val=255;							
								while(frame_flag)//�˳���һ���˵� ��������
								{
									
									frame_flag = 0;//��־λ0
									Delay_us(800000);
								}
								key_val=255;
								
								
										FLASH_OUTPID_P_DATA=external_KP;
										FLASH_OUTPID_I_DATA=external_KI;     //�⻷PID
										FLASH_OUTPID_D_DATA=external_KD;

								    external_KP_f=(float)external_KP/1000.0; 			//�⻷
										external_KI_f=(float)external_KI/1000.0;
										external_KD_f=(float)external_KD/100.0;

								
								write_flash_data();  //д������
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
								OLED_ShowNum(64,2,external_KP,6,16);    //KP��ֵ99.999
								OLED_ShowNum(64,4,external_KI,6,16);    //KI��ֵ
								OLED_ShowNum(48,6,external_KD,8,16);    //KD��ֵ
						}
				}
		 }
}

/***************************/
/*******��һ���˵� 1 ********/
/***************************/

void MENU_1_3(void)
{
		uint8_t mode_set_1_3; //��һ���˵� 1 ��־λ
									
		mode_set_1_3=1;
		screen=0;//��ʾ���
								
		 while(mode_set_1_3)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowString(32,0,"YAW_SET");//��ʾPID
						OLED_ShowString(16,2,"INT_PID");//��ʾPID
						OLED_ShowString(16,4,"OUT_PID");//��ʾPID
//						OLED_ShowString(16,6,"YAW_PID");//��ʾPID
					
						//��ʾ���ڵ�ѡ��
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
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
					
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
					  else if(key_val==RED_OK)// ">"  ����
						{
							  if(mode_set_1_3==1)    //����ģʽ1����
								{
										MENU_1_3_1();
								}
								else if(mode_set_1_3==2)   //����ģʽ2����
								{
										MENU_1_3_2();									
								}
//								else if(mode_set_1==3)   //����ģʽ2����
//								{
//										MENU_1_3();									
//								}/**/								
						}
					
					
						if(key_val==RED_RETURN)//RETURN����
						{						
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_1_3=0;
							  key_val=255;							
								while(frame_flag)//�˳���һ���˵� ��������
								{
									
									frame_flag = 0;//��־λ0
									Delay_us(800000);
								}
								key_val=255;
								
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
/*******��һ���˵� 1-3-1 ******/
/***************************/

void MENU_1_3_1(void)
{
		uint8_t mode_set_1_3_1; //��һ���˵� 1 ��־λ
		
		extern int YAWinterior_KP; 			//�ڲ�
		extern int YAWinterior_KI;
		extern int YAWinterior_KD;
	
		extern int FLASH_YAWINPID_P_DATA;
		extern int FLASH_YAWINPID_I_DATA;       //�ڻ�PID
		extern int FLASH_YAWINPID_D_DATA;

		extern float YAWinterior_KP_f; 			//�ڲ�
		extern float YAWinterior_KI_f;
		extern float YAWinterior_KD_f;
	
		mode_set_1_3_1=1;
		screen=0;//��ʾ���
		
		Flash_data_init(); //��ȡ�ڴ�����
	
		 while(mode_set_1_3_1)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowString(32,0,"YAW INT");//��ʾ
					
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
						OLED_ShowNum(64,2,YAWinterior_KP,5,16);    //KP��ֵ
						OLED_ShowNum(64,4,YAWinterior_KI,5,16);    //KI��ֵ
						OLED_ShowNum(64,6,YAWinterior_KD,5,16);    //KD��ֵ
					
						/*��ʾ���ڵ�ѡ��*/		
				}
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
					
//					
					/*1��λ*/	
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
						/*ʮ��λ*/
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
						/*�ٽ�λ*/
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
						else if(key_val==RED_Right)//"��"
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
//					  else if(key_val==RED_OK)// ">"  ����
//						{
//							  if(mode_set_1_1==1)    //����ģʽ1����
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_1==2)   //����ģʽ2����
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_1==3)   //����ģʽ2����
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN����
						{						
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_1_3_1=0;
							  key_val=255;							
								while(frame_flag)//�˳���һ���˵� ��������
								{
									
									frame_flag = 0;//��־λ0
									Delay_us(800000);
								}
								key_val=255;
								
										FLASH_YAWINPID_P_DATA=YAWinterior_KP;
										FLASH_YAWINPID_I_DATA=YAWinterior_KI;      //�ڻ�PID
										FLASH_YAWINPID_D_DATA=YAWinterior_KD;
								
										YAWinterior_KP_f=(float)YAWinterior_KP/1000; 			//�ڻ� ����
										YAWinterior_KI_f=(float)YAWinterior_KI/1000;
										YAWinterior_KD_f=(float)YAWinterior_KD/1000;
								
								write_flash_data();  //д������
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
								OLED_ShowNum(64,2,YAWinterior_KP,5,16);    //KP��ֵ99.999
								OLED_ShowNum(64,4,YAWinterior_KI,5,16);    //KI��ֵ
								OLED_ShowNum(64,6,YAWinterior_KD,5,16);    //KD��ֵ
						}
				}
		 }
}

/***************************/
/*******��һ���˵� 1-3-2 ******/
/***************************/

void MENU_1_3_2(void)
{
		uint8_t mode_set_1_3_2; //��һ���˵� 1 ��־λ
		
	  extern int YAWexternal_KP; 			//�⻷
		extern int YAWexternal_KI;
		extern int YAWexternal_KD;

		extern int FLASH_YAWOUTPID_P_DATA;
		extern int FLASH_YAWOUTPID_I_DATA;     //�⻷PID
		extern int FLASH_YAWOUTPID_D_DATA;
	
	  extern float YAWexternal_KP_f; 			//�⻷
		extern float YAWexternal_KI_f;
		extern float YAWexternal_KD_f;
	
		mode_set_1_3_2=1;
		screen=0;//��ʾ���
		
		Flash_data_init(); //��ȡ�ڴ�����
	
		 while(mode_set_1_3_2)
		 {			 
				if(!screen)//��ʾ
				{
						screen=1; //��ʾ
					
						OLED_Clear();//����
						OLED_ShowString(32,0,"PID OUT");//��ʾ
					
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
						OLED_ShowNum(64,2,YAWexternal_KP,6,16);    //KP��ֵ
						OLED_ShowNum(64,4,YAWexternal_KI,6,16);    //KI��ֵ
						OLED_ShowNum(48,6,YAWexternal_KD,8,16);    //KD��ֵ
					
						/*��ʾ���ڵ�ѡ��*/		
				}
			 
				if( frame_flag == 1 )      // һ֡�������ݽ������
				{	
						key_val = IrDa_Process();//��ȡ����				
					
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
						/*�ٽ�λ*/
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
						/*ǧ��λ*/
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
						/*���λ*/
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
						}/*ʮ���λ*/
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
						
						else if(key_val==RED_Right)//"��"
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
//					  else if(key_val==RED_OK)// ">"  ����
//						{
//							  if(mode_set_1_3_2==1)    //����ģʽ1����
//								{
//										MENU_1_1();
//								}
//								else if(mode_set_1_3_2==2)   //����ģʽ2����
//								{
//										MENU_1_2();									
//								}
//								else if(mode_set_1_3_2==3)   //����ģʽ2����
//								{
//										MENU_1_3();									
//								}								
//						}
					
					
						if(key_val==RED_RETURN)//RETURN����
						{						
								OLED_Clear();//����
								screen=0;//��ʾ���
								mode_set_1_3_2=0;
							  key_val=255;							
								while(frame_flag)//�˳���һ���˵� ��������
								{
									
									frame_flag = 0;//��־λ0
									Delay_us(800000);
								}
								key_val=255;
								
								
										FLASH_YAWOUTPID_P_DATA=YAWexternal_KP;
										FLASH_YAWOUTPID_I_DATA=YAWexternal_KI;     //�⻷PID
										FLASH_YAWOUTPID_D_DATA=YAWexternal_KD;

								    YAWexternal_KP_f=(float)YAWexternal_KP/1000.0; 			//�⻷
										YAWexternal_KI_f=(float)YAWexternal_KI/1000.0;
										YAWexternal_KD_f=(float)YAWexternal_KD/100.0;

								
								write_flash_data();  //д������
						}
						else
						{
								//��ʾ���ڵ�ѡ��
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
								OLED_ShowNum(64,2,YAWexternal_KP,6,16);    //KP��ֵ99.999
								OLED_ShowNum(64,4,YAWexternal_KI,6,16);    //KI��ֵ
								OLED_ShowNum(48,6,YAWexternal_KD,8,16);    //KD��ֵ
						}
				}
		 }
}
