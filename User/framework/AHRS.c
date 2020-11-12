#include "AHRS.h"
#include "math.h"
#include "swap_data.h"//��λ��

#define RtA 		57.324841f				
#define AtR    		0.0174533f				
#define Acc_G 		0.0011963f				
//#define Gyro_G 		0.0610351f				
//#define Gyro_Gr		0.0010653f
//#define FILTER_NUM 	20

extern T_float_angle angle;

//float 	AngleOffset_Rol=0,AngleOffset_Pit=0;

//void Prepare_Data(T_int16_xyz *acc_in,T_int16_xyz *acc_out)
//{
//	static uint8_t 	filter_cnt=0;
//	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
//	int32_t temp1=0,temp2=0,temp3=0;
//	uint8_t i;

//	ACC_X_BUF[filter_cnt] = acc_in->X;
//	ACC_Y_BUF[filter_cnt] = acc_in->Y;
//	ACC_Z_BUF[filter_cnt] = acc_in->Z;
//	for(i=0;i<FILTER_NUM;i++)
//	{
//		temp1 += ACC_X_BUF[i];
//		temp2 += ACC_Y_BUF[i];
//		temp3 += ACC_Z_BUF[i];
//	}
//	acc_out->X = temp1 / FILTER_NUM;
//	acc_out->Y = temp2 / FILTER_NUM;
//	acc_out->Z = temp3 / FILTER_NUM;
//	filter_cnt++;
//	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
//}



//   ������ƽ��������
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration ����һ��ţ�ٵ�����
	return y;
} 


//  ��float�����ݾ���ֵ
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}


/*   �������Ǻ�����̩��չ��ʽ �����ֵ*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}


/***********************************************
  * @brief  �ɱ���������Ӧ����
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.28 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
/*************************************/

//******************************************************
#include "data_exchang.h"


#define Kp 16.0     //proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0006f   //integral gain governs rate of convergence of gyroscope biases
//#define Kp 6.50     //proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.05f   //integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0025f  //half the sample period,halfT 0.5f��Ҫ���ݾ�����̬����������������T����̬�������ڣ�T*���ٶ�=΢�ֽǶ�

#define Accel_4_Scale_Factor    8192.0f
//float Yaw;
//#define q30  1073741824.0f
//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float q0, q1, q2, q3;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;

extern short Acel[3];    //������[2][3] [4][5] [6][7]      ���ٶ�
extern short Gyro[3];    //������[8][9] [10][11] [12][13]   �Ƕ�
extern short Mag[3];     //������[14][15] [16][17] [18][19]   �ش�


/*******************************************************************************
* Function Name  : init_quaternion
* Description    : �����ʼ����Ԫ��q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void)
{ 
//  unsigned long timestamp;
  signed short int accel[3];//, mag[3];
  float init_Yaw, init_Pitch, init_Roll;
	
	extern short offset_AX,offset_AY,offset_AZ;  //����
	extern short Xoffset,Yoffset,Zoffset;  //����
  int i;

//  if(!i2cread(MPU9150_Addr, Accel_Xout_H, 6, data_write))  //MPU6050ReadAcc(Acel);
//    {
				MPU6050ReadAcc(Acel);
				accel[0]=Acel[0] -offset_AX;
				accel[1]=Acel[1] -offset_AY;
				accel[2]=Acel[2];
							
				init_ax=(float)(accel[0] / Accel_4_Scale_Factor);	   //��λת�����������ٶȵĵ�λ��m/s2
				init_ay=(float)(accel[1] / Accel_4_Scale_Factor);    //#define Accel_4_Scale_Factor    8192.0f
				init_az=(float)(accel[2] / Accel_4_Scale_Factor);
			
				printf("ax=%f,   ay=%f,   az=%f", init_ax, init_ay, init_az);
			
				for(i=0;i<5;i++)   //��һ�ζ�ȡ��compsaa�����Ǵ�ģ����Ҫ������α�֤�Ժ�������ȷ��оƬbug
				{
//						mpu_set_bypass(1);                     //����bypass��������������
//						mpu_get_compass_reg(mag, &timestamp);  //��ȡcompass����
						//����x y���У׼��δ��z�����У׼���ο�MEMSense��У׼����
            MPU9250ReadMag(Mag);
						
						init_mx =(float)Mag[1]-8;						
						init_my =(float)1.046632*Mag[0]-1.569948;
						init_mz =(float)-Mag[2];
					
//						init_mx =(float)Mag[1]-Yoffset-8;						
//						init_my =(float)(Mag[0]-Xoffset)*1.046632-1.569948;
//						init_mz =(float)-(Mag[2]);
//						mpu_set_bypass(0);						//�ر�bypass��������������
						printf("    mx=%f,   my=%f,   mz=%f \n\r", init_mx, init_my, init_mz);
				}
				//������y��Ϊǰ������    
				init_Roll = -atan2(init_ax, init_az);    //����ĵ�λ�ǻ��ȣ�����Ҫ�۲���Ӧ����57.3ת��Ϊ�Ƕ�
				init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
				init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
													 init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//������atan2(my, mx)�����е�init_Roll��init_Pitch�ǻ���
				if(init_Yaw < 0)
				{
						init_Yaw = init_Yaw + 2*3.141593;
				}
				if(init_Yaw > 360)
				{
						init_Yaw = init_Yaw - 2*3.141593;
				}
				            
				//����ʼ��ŷ����ת���ɳ�ʼ����Ԫ����ע��sin(a)��λ�õĲ�ͬ������ȷ����xyz��ת����Pitch����Roll����Yaw������ZXY˳����ת,Qzyx=Qz*Qy*Qx�����е�init_YawRollPtich�ǽǶ�        
				q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
				q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��pitch
				q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��roll
				q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw

				//������x��Ϊǰ������
				//  init_Roll  = atan2(init_ay, init_az);
				//  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
				//  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
				//                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
				//  q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
				//  q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   ��x����ת��roll
				//  q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   ��y����ת��pitch
				//  q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   ��z����ת��Yaw
							
				printf("��ʼ����Ԫ����Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw*57.295780, init_Pitch*57.295780, init_Roll*57.295780);
//  }
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag���ں��㷨��Դ��S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3��Ҫ��ʼ�����ܴ��뵽����ĳ����У�����ֱ��ʹ��1 0 0 0��������ļ��㣬��������Ϊ��
// 1.����У׼accle gyro mag��
// 2.����init_quaternion������1��accle��xyz�����ݣ������ù�ʽ�������ʼ��ŷ���ǣ�
//   ����ACCEL_1G=9.81����λ����m/s2����init_Yaw�����ô����Ƽ��������
// 3.�����Լ��Ĳ������ڣ�������halfT��halfT=��������/2����������Ϊִ��1��AHRSupdate���õ�ʱ�䣻
// 4.��2�м������ŷ����ת��Ϊ��ʼ������Ԫ��q0 q1 q2 q3���ںϼ��ٶȼƣ������ǣ�������º��ŷ����pitch��roll��Ȼ��ʹ��pitch roll�ʹ����Ƶ����ݽ��л����˲��ںϵõ�Yaw������ʹ�ã�����ŷ��������㣻
// 5.��ֱ��ʹ����Ԫ����
// 6.�ظ�4�����ɸ�����̬;

//�ܵ���˵�������������ǣ����ٶȼ�������������Pitch��Roll��������������������Yaw;
//���³����У�gx, gy, gz��λΪ����/s��ax, ay, azΪ���ٶȼ������ԭʼ16��������, mx, my, mzΪ�����������ԭʼ16�������ݣ�
//ǰ������mpu9150�ļ��ٶȼƺ������ǵ�x��Ϊǰ������;
//���³�����õĲο�����Ϊ��mpu9150�ļ��ٶȼƺ���������ָ��xyz����Ϊ������

//������Ϊ����500��/s��ǰ���£������ǵ���������65.5LSB/��/s�����԰������������ʮ���������ݳ���65.5���ǽ��ٶȣ���λ�ǡ�/s��
//Ȼ���ٳ���57.3�ͱ�ɻ�����;(1����=180/pi=57.3��)

//ŷ���ǵ�λΪ����radian������57.3�Ժ�ת��Ϊ�Ƕ�,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;//, halfT;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;
   float Pitch, Roll, Yaw;

/*����֮��ĳ���ʹ�ã����ټ���ʱ��*/
        //auxiliary variables to reduce number of repeated operations��
   float q0q0 = q0*q0;
   float q0q1 = q0*q1;
   float q0q2 = q0*q2;
   float q0q3 = q0*q3;
   float q1q1 = q1*q1;
   float q1q2 = q1*q2;
   float q1q3 = q1*q3;
   float q2q2 = q2*q2;   
   float q2q3 = q2*q3;
   float q3q3 = q3*q3;
          
/*��һ������ֵ�����ٶȼƺʹ����Ƶĵ�λ��ʲô������ν����Ϊ�����ڴ˱����˹�һ������*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ������������Ǵӷ���������ϵ����������ϵ��ת����ʽ*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*�����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣����Ҷ���byָ������������by=ĳֵ��bx=0
������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
�����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
�ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
��Ϊbx=0�����Ծͼ򻯳�(by*by)  = ((hx*hx) + (hy*hy))�������by�������޸�by��bxָ����Զ����ĸ���ָ������*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)����������Ǵ���������ϵ������������ϵ��ת����ʽ(ת�þ���)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
��Ϊbx=0�����������漰��bx�Ĳ��ֶ���ʡ���ˡ�ͬ��by=0�����������漰��by�Ĳ���Ҳ���Ա�ʡ�ԣ�������Լ������Ǹ���ָ���йء�
������������vxyz�����㣬��Ϊ����g��az=1��ax=ay=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
����Կ���������ʽ��wxyz�Ĺ�ʽ����by����ay��0������bz����az��1�����ͱ����vxyz�Ĺ�ʽ�ˣ�����q0q0+q1q1+q2q2+q3q3=1����*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
//   halfT=GET_NOWTIME();		//�õ�ÿ����̬���µ����ڵ�һ��
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //�ܹؼ���һ�仰��ԭ�㷨û��
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //���Բ������ڵ�һ��
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise����Ԫ�������㷨
   q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
   q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
   q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
   q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
        
   // normalise quaternion
   norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 = q0 * norm;       //w
   q1 = q1 * norm;       //x
   q2 = q2 * norm;       //y
   q3 = q3 * norm;       //z
        
///*����Ԫ�������Pitch  Roll  Yaw
//����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
		Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //ƫ���ǣ���z��ת��	
    if(Yaw < 0 ){Yaw = Yaw + 360;}
		if(Yaw > 360 ){Yaw = Yaw - 360;}
//		Roll 		= asin	(2	*q2*q3 + 2*q0*q1) * 57.3; //�����ǣ���x��ת��	 
//		Pitch   = -atan2(-2	*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3; //�����ǣ���y��ת��
   	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //�����ǣ���y��ת��	 
    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //�����ǣ���x��ת��
//	if(Roll<0)Yaw+=Roll/2;
//	else if(Yaw<180) Yaw-=Roll/2;
//	send_line( Yaw, Pitch*100, Roll*100, Yaw, Pitch*10, Roll*10);//ɽ��
		angle.yaw = Yaw; 
		angle.roll =Roll;
		angle.pitch =Pitch;
	
/*���������Ԫ�������Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
����57.3��Ϊ�˽�����ת��Ϊ�Ƕ�*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //�����ǣ���y��ת��	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //�����ǣ���x��ת��
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.3;  //ƫ���ǣ���z��ת��

//	printf("q0=%f, q1=%f, q2=%f, q3=%f, Yaw=%f, Pitch=%f, Roll=%f, halfT=%f \n\r", q0, q1, q2, q3, Yaw, Pitch, Roll, halfT);
//    printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
}


/*******************************************************************************
���ټ��� 1/Sqrt(x)��Դ������3��һ�δ��룬�����0x5f3759df���������Ĵ����4�� 	
*******************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//************************************************************
///********************************************************************/
//#define Kp    1.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki    0.0008f                     // integral gain governs rate of convergence of gyroscope biases
////#define halfT 0.00125f                 // �������ڵ�һ��  ������ 2.5MS �ɼ�һ��  ���� halfT��1.25MS
//#define halfT 0.01f                 // �������ڵ�һ��  ������ 2.5MS �ɼ�һ��  ���� halfT��1.25MS
//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
//float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
// 
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
//{
//		float norm;
//		int16_t Xr,Yr;
//		float vx, vy, vz;// wx, wy, wz;
//		float ex, ey, ez;

//		// �Ȱ���Щ�õõ���ֵ���
//		float q0q0 = q0*q0;
//		float q0q1 = q0*q1;
//		float q0q2 = q0*q2;
//		//  float q0q3 = q0*q3;//
//		float q1q1 = q1*q1;
//		//  float q1q2 = q1*q2;//
//		float q1q3 = q1*q3;
//		float q2q2 = q2*q2;
//		float q2q3 = q2*q3;
//		float q3q3 = q3*q3;
//	//���Ƕ���6050���ݲ�������ų�	
//		if(ax*ay*az==0)
//			return;
//			
//		norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc���ݹ�һ��
//		ax = ax *norm;
//		ay = ay * norm;
//		az = az * norm;

//		// estimated direction of gravity and flux (v and w)              �����������������/��Ǩ
//		vx = 2*(q1q3 - q0q2);												//��Ԫ����xyz�ı�ʾ
//		vy = 2*(q0q1 + q2q3);
//		vz = q0q0 - q1q1 - q2q2 + q3q3 ;

//		// error is sum of cross product between reference direction of fields and direction measured by sensors
//		ex = (ay*vz - az*vy) ;                           					 //�������������õ���־������
//		ey = (az*vx - ax*vz) ;
//		ez = (ax*vy - ay*vx) ;

//		exInt = exInt + VariableParameter(ex) * ex * Ki;								  //�������л���
//		eyInt = eyInt + VariableParameter(ey) * ey * Ki;
//		ezInt = ezInt + VariableParameter(ez) * ez * Ki;
//	// adjusted gyroscope measurements

//		gx = gx + Kp *  VariableParameter(ex) * ex + exInt;	
//		gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
//		gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
//										
//		// integrate quaternion rate and normalise						   //��Ԫ�ص�΢�ַ���
//		q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//		q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//		q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//		q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

//		// normalise quaternion
//		norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
//		q0 = q0 * norm;
//		q1 = q1 * norm;
//		q2 = q2 * norm;
//		q3 = q3 * norm;


//		
//		angle.roll = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1); // roll
//		angle.pitch = asin(-2*q1q3 + 2*q0q2); // pitch
//		
//		
//		
//		//          ���ڵش���ν�����ǲ���                       //    
//		//�ο�  http://baike.baidu.com/view/1239157.htm?fr=aladdin    //
//		/*
//		Xr = X_HMC * COS(angle.pitch/AtR) + Y_HMC * SIN(-angle.pitch/AtR) * SIN(-angle.roll/AtR) - Z_HMC * COS(angle.roll/AtR) * SIN(-angle.pitch/AtR);
//		Yr = Y_HMC * COS(angle.roll/AtR) + Z_HMC * SIN(-angle.roll/AtR);
//    
//		angle.yaw = atan2((double)Yr,(double)Xr) * RtA; // yaw*/ 
//		angle.roll *= RtA;
//		angle.pitch *= RtA;
//	  
//}





/*
////////////////////////////////////////////////////////////////////////////////
#define Kp 20.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
																				//����������������ٶȼ��ٶȼƺʹ�ǿ��
#define Ki 0.002f                          // integral gain governs rate of convergence of gyroscope biases
																				//������������������������ǵ�ƫ��
#define halfT 0.001f                   // half the sample period�������ڵ�һ��

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation��Ԫ��Ԫ�ش������ȡ��
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error �����������
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az ,float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // �Ȱ���Щ�õõ���ֵ���
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  //�ѼӼƵ���ά����ת�ɵ�λ������

  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  //�ӻ�������ϵ�ĵ������̲⵽��ʸ��ת�ɵ�������ϵ�µĴų�ʸ��hxyz������ֵ��
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
  
//  �����������ϵ�µĴų�ʸ��bxyz���ο�ֵ����
//  ��Ϊ����ش�ˮƽ�нǣ�������֪��0�ȣ���ȥ��ƫ�ǵ����أ��̶��򱱣�������by=0��bx=ĳֵ
//  ������ο��ش�ʸ���ڴ�ֱ����Ҳ�з���bz��������ÿ���ط����ǲ�һ���ġ�
//  �����޷���֪��Ҳ���޷������ںϣ��и��ʺ�����ֱ���������ںϵļ��ٶȼƣ�������ֱ�ӴӲ���ֵhz�ϸ��ƹ�����bz=hz��
//  �ų�ˮƽ�������ο�ֵ�Ͳ���ֵ�Ĵ�СӦ����һ�µ�(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))��
//  ��Ϊby=0�����Ծͼ򻯳�(bx*bx)  = ((hx*hx) + (hy*hy))�������bx��
           
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
 
//  ���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
//  �������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
//  ���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������
  
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
 
 // ���ǰѵ�������ϵ�ϵĴų�ʸ��bxyz��ת����������wxyz��
 // ��Ϊby=0�����������漰��by�Ĳ��ֶ���ʡ���ˡ�
 // ������������vxyz�����㣬��Ϊ����g��gz=1��gx=gy=0�����������漰��gxgy�Ĳ���Ҳ��ʡ����
  
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  //���ڰѼ��ٶȵĲ���ʸ���Ͳο�ʸ����������Ѵų��Ĳ���ʸ���Ͳο�ʸ��Ҳ����������������������ݡ�
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  
 // axyz�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
//  axyz�ǲ����õ�������������vxyz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
// ������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
//  ������������������������Ҳ�������������ˣ�����ʾ��exyz�����������������Ĳ����
//  �����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ���������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����
  
if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  ezInt = ezInt + ez * Ki * halfT;

  // �ò���������PI����������ƫ
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  }

  // ��Ԫ��΢�ַ���
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // ��Ԫ���淶��
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* RtA; // yaw
  angle.pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* RtA; // pitch
  angle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll
}*/

