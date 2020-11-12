#include "data_exchang.h"
#include "swap_data.h"//��λ��
#include "math.h"
#include "LED_config.h"
#include "AHRS.h"
#include "Kalman_filter.h"
#include "PID_control.h"
#include "oled.h"



//���ݴ���
T_float_xyz radian_gyro;   //������Ƕ�ֵ
T_float_xyz averag_gyro; //�������˲���ĽǶ�����
T_float_xyz origin_Acel;//������ļ��ٶ�ֵ
T_float_xyz averag_Acel; //�������˲���ļ��ٶ�����
T_float_xyz value_gyro; //У׼�������

//��Ųɼ�������
short Acel[3];    //������[2][3] [4][5] [6][7]      ���ٶ�
short Gyro[3];    //������[8][9] [10][11] [12][13]   �Ƕ�
short Mag[3];     //������[14][15] [16][17] [18][19]   �ش�
short TEMP[1];
double TEMP_280;  //������[20][21]   �¶�
s32 Press;        //������[22][23] ��ѹ
extern unsigned short int Distance;      //������ľ��� 

extern uint8_t AGM_START;//�ǶȲɼ���־λ
extern uint8_t DATA_START;//��ͨ���ݲɼ���־λ
extern uint8_t PRESS_START;//��ѹ�ɼ���־λ
extern uint8_t Ultrasonic_START;//�������ɼ���־λ


double MAG;//��ż���õĵشŽǶ�

extern double ADC_ConvertedValueLocal;//������[24][25] ��ѹ
extern __IO uint16_t ADC_ConvertedValue;

T_float_angle angle;



void data_init(void)
{		
		extern short Xoffset,Yoffset,Zoffset;  //����
		extern T_float_angle angle;
	
		if(AGM_START) //5ms�ɼ�һ��
		{    
//				PWM_control();//PWM�������
				AGM_START=0;//�����־λ
//				send_line( angle.roll*100, angle.roll*100, angle.yaw*100, radian_gyro.X*100, radian_gyro.Y*100, radian_gyro.Z*100);//ɽ��

			  data_handling();//�ɼ���һ�����ݲ��ҽ������ݴ���
					
				PID_control();//PID����
//							
			 
				
			
//				IMUupdate(averag_gyro.X, averag_gyro.Y, averag_gyro.Z,averag_Acel.X, averag_Acel.Y, averag_Acel.Z);//��Ԫ���㷨
//				IMUupdate(radian_gyro.X, radian_gyro.Y, radian_gyro.Z,averag_Acel.X, averag_Acel.Y, averag_Acel.Z,1,1,1);			
//			printf("\nroll: %f\npitch:%f\n\n",angle.roll*57.296,angle.pitch*57.296);
			
//				MPU6050ReadAcc(Acel);					 
//				MPU6050ReadGyro(Gyro);			
//				MPU9250ReadMag(Mag);
//			calibration_Mag(Mag[0],Mag[1],Mag[2]);
				
	
//				printf("\n���ٶȣ�%8d%8d%8d\t",Acel[0],Acel[1],Acel[2]);
//				printf("������:%8d%8d%8d\t",Gyro[0],Gyro[1],Gyro[2]);
//				printf("\n�شų�:%8d%8d%8d\t",Mag[0]-Xoffset,Mag[1]-Yoffset,Mag[2]-Zoffset);
////						printf("\n�����¶�%d\t",Temp);
//						printf("�¶�Ϊ %lf",TEMP_280);
//							printf("\t��ѹΪ %d��\n\n",Press);
//				      printf("\t��ѹΪ= %lf V \t",ADC_ConvertedValueLocal) ;
//							printf("1�ž���Ϊ:%d cm\t",Distance);
				
					
		}

		if(PRESS_START)//100ms�İ�ʱ��ɼ�
		{
				PRESS_START=0;
			
				BMP280_GetPressure(&Press);//��ʱ10ms
		}
		if(DATA_START) //100ms�ɼ�һ��
		{		
				DATA_START=0;			
				/*��ѹ�ɼ�*/
				ADC_ConvertedValueLocal =(float) ADC_ConvertedValue/4096*16.6; // ��ȡת����ADֵ
			  OLED_ShowNum(64,6,ADC_ConvertedValueLocal*100,5,16);    //KD��ֵ
			
				/*�������ɼ�*/
				UltrasonicWave_StartMeasure();//�������

 
				/*��ѹ���¶Ȳɼ�*/
				BMP280_GetTemperature(&TEMP_280);//��ʱ4ms					

			
			
			
			
				
		}
//			UltrasonicWave_StartMeasure();//�������
		UltrasonicWave_OK();//�������
		/*�����ж���*/
//	  LED0_OFF;LED0_ON;
//		MPU6050ReadAcc(Acel);					 
//		MPU6050ReadGyro(Gyro);			
//		MPU9250ReadMag(Mag);LED0_ON;				
//		MPU6050_ReturnTemp(&Temp);	
//		BMP280_GetTemperature(&TEMP_280);				
//		BMP280_GetPressure(&Press);		
    

////		printf("\t��ѹΪ= %lf V \t",ADC_ConvertedValueLocal) ;
//		printf("\n�����ǣ�%8d%8d%8d\t",Acel[0],Acel[1],Acel[2]);
//		printf("���ٶ�%8d%8d%8d\t",Gyro[0],Gyro[1],Gyro[2]);
//		printf("\n�شų�%8d%8d%8d\t",Mag[0],Mag[1],Mag[2]);
////		printf("\n�����¶�%d\t",Temp);
////		printf("�¶�Ϊ %lf",TEMP_280);
//		printf("\t��ѹΪ %d��\n\n",Press);
}
short Xlast,Ylast,Zlast;            //

void MAG_data(short magX,short magY,short magZ)
{
		 if(magX>200||magX<-200)
		{
			Mag[0]=Xlast;
		}
		else
		{
			Xlast=Mag[0];
		}
		
	  if(magY>200||magY<-200)
		{
			Mag[1]=Ylast;
		}
		else
		{
			Ylast=Mag[1];
		}
		
		if(magZ>200||magZ<-120)
		{
			Mag[2]=Zlast;
		}
		else
		{
			Zlast=Mag[2];
		}
	
//		double magx,magy;
//	
//		if(Mag[0]<100&&Mag[1]<100)
//		{
//				magx=Mag[0]+(double)(Gyro[1]*0.0035);
//				magy=Mag[1]+(double)(Gyro[0]*0.0035);
//										
//				if(magx>0&&magy<0)
//				{
//						MAG=(double)(-(double)(atan(magy/magx))*57.2958279);//0-90
//				}
//				else if(magx<0)
//				{
//						MAG=(double)(180-(double)(atan(magy/magx))*57.2958279);//90-180,180-270
//				
//				}
//				else if(magx>0&&magy>0)
//				{
//						MAG=(double)(360-(double)(atan(magy/magx))*57.2958279);//270-360
//				}
//				else if(magx==0&&magy<0)
//				{
//						MAG=90;
//				}
//				else if(magx<0&&magy==0)
//				{
//						MAG=180;
//				}
//				else if(magx==0&&magy>0)
//				{
//						MAG=270;
//				}
//	   }
//		
////		 Mag[2]=(short)MAG;//+Mag[2];
		 
		
}

//*********************************************//
//*                ��������У׼                *//
//*********************************************//

short Xmax,Ymax,Zmax,Xmin,Ymin,Zmin; //���ֵ ��Сֵ
short Xlast,Ylast,Zlast;            //
short Xoffset,Yoffset,Zoffset;  //����
short Xscope,Yscope,Zscope;  //��Χ
T_float_xyz Mag_corr;//У׼��
double Orien;//����

void calibration_Mag(short magX,short magY,short magZ)
{

		
		if(magX>120||magX<-120)
		{
			magX=Xlast;
		}
		else if(Xscope<120)	
		{
			 Xlast=magX;
				if(Xmax<magX)
				{
					 Xmax=magX;
				}
				if(Xmin>magX)
				{
					 Xmin=magX;
				}
				Xoffset=(Xmax+Xmin)/2;
		}
			
		if(magY>120||magY<-120)
		{
			magY=Ylast;
		}
		else if(Yscope<120)	
		{
			 Ylast=magY;
				if(Ymax<magY)
				{
					 Ymax=magY;
				}
				if(Ymin>magY)
				{
					 Ymin=magY;
				}
				Yoffset=(Ymax+Ymin)/2;
		}
		
		if(magZ>120||magZ<-120)
		{
			magZ=Zlast;
		}
		else if(Zscope<120)
		{
			 Zlast=magZ;
				if(Zmax<magZ)
				{
					 Zmax=magZ;
				}
				if(Zmin>magZ)
				{
					 Zmin=magZ;
				}
				Zoffset=(Zmax+Zmin)/2;
		}
				
		    Xscope=Xmax-Xmin;
				Yscope=Ymax-Ymin; //�õ�����
				Zscope=Zmax-Zmin;
		

		
				printf("��С��%d,\t%d,\t%d\n",Xmin,Ymin,Zmin);
				printf("���%d,\t%d,\t%d\n",Xmax,Ymax,Zmax);
				printf("���䣺%d,\t%d,\t%d\n\n",Xmax-Xmin,Ymax-Ymin,Zmax-Zmin);
				printf("������%d,\t%d,\t%d\n\n",Xoffset,Yoffset,Zoffset);
				
				Mag_corr.X=magX-Xoffset; //����ƫ��ֵ
				Mag_corr.Y=magY-Yoffset;
				Mag_corr.Z=magZ-Zoffset;
		
				                                           
			
				printf("ԭʼ��%d,\t%d,\t%d\n\n",(int)Mag_corr.X,(int)Mag_corr.Y,(int)Mag_corr.Z);
				printf("У׼��%d,\t%d,\t%d\n\n",magX,magY,magZ);
				 Orien=(double)((double)(atan(Mag_corr.Y/Mag_corr.X))*57.2958279);      //����
				printf("�Ƕȣ�%f\n",Orien);
}	



//*********************************************//
//*                 ������У׼                 *//
//*********************************************//

uint8_t calibration_GA_time; //У׼�ɼ�����
int average_GX,average_GY,average_GZ,average_AX,average_AY,average_AZ;//ƽ��
short offset_GX,offset_GY,offset_GZ;  //����
short offset_AX,offset_AY,offset_AZ;  //����

//uint8_t calibration_GA_OK;   //У׼��ɱ�־λ

void calibration_GA(short GyroX,short GyroY,short GyroZ,short AcelX,short AcelY,short AcelZ)
{
	   
	  calibration_GA_time++;
		
		if(GyroX>500||GyroY>500||GyroZ>500||GyroX<-500||GyroY<-500||GyroZ<-500)
		{
			  calibration_GA_time=255;
		}
	
		if(calibration_GA_time==100)
		{
				offset_GX=(short)((float)average_GX/100.0+0.5);
				offset_GY=(short)((float)average_GY/100.0+0.5);
				offset_GZ=(short)((float)average_GZ/100.0+0.5);
				offset_AX=(short)((float)average_AX/100.0+0.5);
				offset_AY=(short)((float)average_AY/100.0+0.5);
				offset_AZ=(short)((float)average_AZ/100.0+0.5);
			
//				calibration_GA_OK=1;   //У׼�����1
		}
		else
		{
			 average_GX+=GyroX;
			 average_GY+=GyroY;
			 average_GZ+=GyroZ;
			 average_AX+=AcelX;
			 average_AY+=AcelY;
			 average_AZ+=AcelZ;
			
//			printf("�� �ȣ�%d,\t%d,\t%d\n\n",GyroX,GyroY,GyroZ);
//			printf("���ٶȣ�%d,\t%d,\t%d\n\n",AcelX,AcelY,AcelZ);
//			printf("�� �Ȳ�����%d,\t%d,\t%d\n\n",average_GX,average_GY,average_GZ);
//			printf("���ٶȲ�����%d,\t%d,\t%d\n\n",average_AX,average_AY,average_AZ);
		}
//		printf("�� �Ȳ�����%d,\t%d,\t%d\n\n",average_GX,average_GY,average_GZ);
//			printf("���ٶȲ�����%d,\t%d,\t%d\n\n",average_AX,average_AY,average_AZ);
}



/*****************************************************/
/*                                                   */
/*                     ���ݴ���                       */
/*                                                   */
/*****************************************************/

//#define Gyro_G 	0.03051756	//  1/32768/1000      ����������Ϊ +��1000			
//#define Gyro_Gr	0.0005327   //  1/32768/1000/57.3 

#include "Kalman_filter.h"




void data_handling(void)
{
			extern short offset_GX,offset_GY,offset_GZ;  //����
			extern short offset_AX,offset_AY,offset_AZ;  //����
	
//			MPU6050ReadAcc(Acel);					 
//			MPU6050ReadGyro(Gyro);
			MPU6050Read_all(Acel,Gyro,TEMP);
	
			MPU9250ReadMag(Mag);
	
	
//	    printf("\nroll: %f\npitch:%f\n\n",radian_gyro.X*57.296,radian_gyro.Y*57.296);
	    //����õ�����ֵ
//			radian_gyro.X = Gyro[0] * Gyro_Gr - offset_GX * Gyro_Gr;
//			radian_gyro.Y = Gyro[1] * Gyro_Gr - offset_GY * Gyro_Gr;   //�õ��ǶȵĻ���
//			radian_gyro.Z = Gyro[2] * Gyro_Gr - offset_GZ * Gyro_Gr;
	
			
		  value_gyro.X = (float)(Gyro[0]  - offset_GX)/100;
			value_gyro.Y = (float)(Gyro[1]  - offset_GY)/100;   //���ٶȶ�У׼
			value_gyro.Z = (float)(Gyro[2]  - offset_GZ)/100; 
			
			radian_gyro.X = value_gyro.X/3724.5 ;
			radian_gyro.Y = value_gyro.Y/3724.5 ;   //���ٶȶ�У׼
			radian_gyro.Z = value_gyro.Z/3724.5 ;
						
			origin_Acel.X=(float)Acel[0] - offset_AX;
	    origin_Acel.Y=(float)Acel[1] - offset_AY;   //�������ٶ�
			origin_Acel.Z=(float)Acel[2];
	
			MAG_data(Mag[0], Mag[1], Mag[2]);//�ش����ƴ���
//	
//			AHRSupdate(radian_gyro.X,radian_gyro.Y, radian_gyro.Z, origin_Acel.X, origin_Acel.Y, origin_Acel.Z, Mag[0], Mag[1], Mag[2]); 
			
			DATA_Kalman_Filter();//����������
			AHRSupdate(radian_gyro.X,radian_gyro.Y, radian_gyro.Z, averag_Acel.X, averag_Acel.Y, averag_Acel.Z, Mag[0], Mag[1], Mag[2]); 

			
	    /*���ͼ��ٶ��˲�ǰ������*/
//			send_line( averag_Acel.X, averag_Acel.Y, averag_Acel.Z, origin_Acel.X, origin_Acel.Y, origin_Acel.Z);//ɽ��		
			/*���ͽǶ��˲�ǰ������*/
//        send_line((float)averag_gyro.X*100, (float)averag_gyro.Y*100, (float)averag_gyro.Z*100,(float)radian_gyro.X*100, (float)radian_gyro.Y*100, (float)radian_gyro.Z*100);//ɽ��
//			  send_line(origin_Acel.X, origin_Acel.Y, averag_Acel.X,averag_Acel.Y, -angle.roll*100, -angle.pitch*100);
				
//			angle.pitch = averag_gyro.X;
//			angle.roll = averag_gyro.Y;
}

/*YAW���ݳ�ʼ��*/
void YAW_data_init(void)
{
	  extern T_float_angle angle;   //�ɻ��Ƕ�����
		extern float START_YAW; //��ʼƫ��	
		uint8_t i=10;
	
		while(i)
		if(AGM_START) //5ms�ɼ�һ��		
		{    
				AGM_START=0;//�����־λ
			  init_quaternion();
			  i--;
		}
		 i=100;
		while(i)
		if(AGM_START) //5ms�ɼ�һ��		
		{    
				AGM_START=0;//�����־λ
			  data_handling();//�ɼ���һ�����ݲ��ҽ������ݴ���
			  i--;
		}
		START_YAW=angle.yaw;//����yaw
}




//*********************************************//
//*                �����˲�                   *//
//*********************************************//
 
void change(void)
{

  float ax,ay,az;
  float gx,gy,gz;
  float aerrx=0,aerry=0;
	T_float_xyz Acel1;
	T_float_xyz Gyro1;
	
//	int GX,GY,GZ;
      int AX_DATA,AY_DATA,AZ_DATA;
      int GX_DATA,GY_DATA,GZ_DATA;

			AX_DATA=(int)averag_Acel.X;
			AY_DATA=(int)averag_Acel.Y;
			AZ_DATA=(int)averag_Acel.Z;
      GX_DATA=(int)averag_gyro.X;
			GY_DATA=(int)averag_gyro.Y;
			GZ_DATA=(int)averag_gyro.Z;
//	
//	GX= GX_DATA;
//  GY= GY_DATA;
//  GZ= GZ_DATA;
//	gx=GX/16.4;
//  gy=GY/16.4;
//  gz=GZ/16.4;
	
  /*������λg�Ͷ�/s*/
//  ax=AX_DATA;
//  ay=AY_DATA;
//  az=AZ_DATA;
//  gx=GX_DATA/16.4;
//  gy=GY_DATA/16.4;
//  gz=GZ_DATA/16.4;
  gx=AX_DATA;
  gy=AY_DATA;
  gz=AZ_DATA;
  ax=GX_DATA/16.4;
  ay=GY_DATA/16.4;
  az=GZ_DATA/16.4;
	
	
  /*�õ���������Ȼ����ϵ�ļн�*/
  Acel1.X=atan2(ax,az)*57.2958;
  Acel1.Y=atan2(ay,az)*57.2958;
  Acel1.Z=atan(az/sqrt(ax*ax+ay*ay))*57.2958;
	
  Gyro1.X =Gyro1.X+gx*0.0000017;
  Gyro1.Y =Gyro1.Y+gy*0.0000017; 
  Gyro1.Z =Gyro1.Z+gz*0.000017;  
  /******************************************
  �����˲���ʵ�ֲ��θ���,GY����AX,GX����AY
	Z���޷���������,��ͨ���شŴ��������������
	��������
  ******************************************/
  aerrx=Acel1.X - Gyro1.Y;
  aerry=Acel1.Y - Gyro1.X;
  Gyro1.Y=Gyro1.Y + aerrx*0.3;
  Gyro1.X=Gyro1.X + aerry*0.3;
	angle.pitch=Gyro1.X+0;		
	angle.roll=Gyro1.Y+0; //����  
	
}
 /*
#define Kp 20.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.002f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period???????

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(T_int16_xyz *gyr, T_int16_xyz *acc, T_float_angle *angle)
{
	float ax = acc->X,ay = acc->Y,az = acc->Z;
	float gx = gyr->X,gy = gyr->Y,gz = gyr->Z;
  float norm;
//  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  // ???????????
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
		
	gx *= Gyro_Gr;
	gy *= Gyro_Gr;
	gz *= Gyro_Gr;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc?????
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              ?????????/??
  vx = 2*(q1q3 - q0q2);												//????xyz???
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //???????????????
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //???????
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//???PI???????,???????
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//???gz????????????????,??????????????

  // integrate quaternion rate and normalise						   //????????
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

	angle->yaw += gyr->Z*Gyro_G*0.002f;
	
	angle->pit = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 - AngleOffset_Pit; // pitch
	angle->rol = -atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 - AngleOffset_Rol; // roll
}*/


