#include "AHRS.h"
#include "math.h"
#include "swap_data.h"//上位机

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



//   快速求平方根倒数
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
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 


//  求float型数据绝对值
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}


/*   采用三角函数的泰勒展开式 求近似值*/
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
  * @brief  可变增益自适应参数
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
#define halfT 0.0025f  //half the sample period,halfT 0.5f需要根据具体姿态更新周期来调整，T是姿态更新周期，T*角速度=微分角度

#define Accel_4_Scale_Factor    8192.0f
//float Yaw;
//#define q30  1073741824.0f
//float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
float q0, q1, q2, q3;
float exInt = 0, eyInt = 0, ezInt = 0;        // scaled integral error
float init_ax, init_ay, init_az, init_gx, init_gy, init_gz, init_mx, init_my, init_mz;

extern short Acel[3];    //存入于[2][3] [4][5] [6][7]      加速度
extern short Gyro[3];    //存入于[8][9] [10][11] [12][13]   角度
extern short Mag[3];     //存入于[14][15] [16][17] [18][19]   地磁


/*******************************************************************************
* Function Name  : init_quaternion
* Description    : 算出初始化四元数q0 q1 q2 q3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void init_quaternion(void)
{ 
//  unsigned long timestamp;
  signed short int accel[3];//, mag[3];
  float init_Yaw, init_Pitch, init_Roll;
	
	extern short offset_AX,offset_AY,offset_AZ;  //补偿
	extern short Xoffset,Yoffset,Zoffset;  //补偿
  int i;

//  if(!i2cread(MPU9150_Addr, Accel_Xout_H, 6, data_write))  //MPU6050ReadAcc(Acel);
//    {
				MPU6050ReadAcc(Acel);
				accel[0]=Acel[0] -offset_AX;
				accel[1]=Acel[1] -offset_AY;
				accel[2]=Acel[2];
							
				init_ax=(float)(accel[0] / Accel_4_Scale_Factor);	   //单位转化成重力加速度的单位：m/s2
				init_ay=(float)(accel[1] / Accel_4_Scale_Factor);    //#define Accel_4_Scale_Factor    8192.0f
				init_az=(float)(accel[2] / Accel_4_Scale_Factor);
			
				printf("ax=%f,   ay=%f,   az=%f", init_ax, init_ay, init_az);
			
				for(i=0;i<5;i++)   //第一次读取的compsaa数据是错的，因此要多读几次保证以后数据正确，芯片bug
				{
//						mpu_set_bypass(1);                     //开启bypass，必须有这句代码
//						mpu_get_compass_reg(mag, &timestamp);  //读取compass数据
						//进行x y轴的校准，未对z轴进行校准，参考MEMSense的校准方法
            MPU9250ReadMag(Mag);
						
						init_mx =(float)Mag[1]-8;						
						init_my =(float)1.046632*Mag[0]-1.569948;
						init_mz =(float)-Mag[2];
					
//						init_mx =(float)Mag[1]-Yoffset-8;						
//						init_my =(float)(Mag[0]-Xoffset)*1.046632-1.569948;
//						init_mz =(float)-(Mag[2]);
//						mpu_set_bypass(0);						//关闭bypass，必须有这句代码
						printf("    mx=%f,   my=%f,   mz=%f \n\r", init_mx, init_my, init_mz);
				}
				//陀螺仪y轴为前进方向    
				init_Roll = -atan2(init_ax, init_az);    //算出的单位是弧度，如需要观察则应乘以57.3转化为角度
				init_Pitch=  asin(init_ay);              //init_Pitch = asin(ay / 1);      
				init_Yaw  =  atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
													 init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));//类似于atan2(my, mx)，其中的init_Roll和init_Pitch是弧度
				if(init_Yaw < 0)
				{
						init_Yaw = init_Yaw + 2*3.141593;
				}
				if(init_Yaw > 360)
				{
						init_Yaw = init_Yaw - 2*3.141593;
				}
				            
				//将初始化欧拉角转换成初始化四元数，注意sin(a)的位置的不同，可以确定绕xyz轴转动是Pitch还是Roll还是Yaw，按照ZXY顺序旋转,Qzyx=Qz*Qy*Qx，其中的init_YawRollPtich是角度        
				q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
				q1 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) - sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是pitch
				q2 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是roll
				q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw

				//陀螺仪x轴为前进方向
				//  init_Roll  = atan2(init_ay, init_az);
				//  init_Pitch = -asin(init_ax);              //init_Pitch = asin(ax / 1);      
				//  init_Yaw   = -atan2(init_mx*cos(init_Roll) + init_my*sin(init_Roll)*sin(init_Pitch) + init_mz*sin(init_Roll)*cos(init_Pitch),
				//                      init_my*cos(init_Pitch) - init_mz*sin(init_Pitch));                       //atan2(mx, my);
				//  q0 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //w
				//  q1 = sin(0.5*init_Roll)*cos(0.5*init_Pitch)*cos(0.5*init_Yaw) - cos(0.5*init_Roll)*sin(0.5*init_Pitch)*sin(0.5*init_Yaw);  //x   绕x轴旋转是roll
				//  q2 = cos(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw) + sin(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw);  //y   绕y轴旋转是pitch
				//  q3 = cos(0.5*init_Roll)*cos(0.5*init_Pitch)*sin(0.5*init_Yaw) - sin(0.5*init_Roll)*sin(0.5*init_Pitch)*cos(0.5*init_Yaw);  //z   绕z轴旋转是Yaw
							
				printf("初始化四元数：Yaw=%f, Pitch=%f, Roll=%f \n\r", init_Yaw*57.295780, init_Pitch*57.295780, init_Roll*57.295780);
//  }
}

/***************************************************************************************************************************************
* Function Name  : AHRSupdate
* Description    : accel gyro mag的融合算法，源自S.O.H. Madgwick
* Input          : None
* Output         : None
* Return         : None
// q0 q1 q2 q3需要初始化才能带入到下面的程序中，不能直接使用1 0 0 0进行下面的计算，整个步骤为：
// 1.首先校准accle gyro mag；
// 2.调用init_quaternion，根据1中accle的xyz轴数据，并利用公式计算出初始化欧拉角，
//   其中ACCEL_1G=9.81，单位都是m/s2，而init_Yaw可以用磁力计计算出来；
// 3.根据自己的采样周期，来调整halfT，halfT=采样周期/2，采样周期为执行1次AHRSupdate所用的时间；
// 4.将2中计算出的欧拉角转化为初始化的四元数q0 q1 q2 q3，融合加速度计，陀螺仪，算出更新后的欧拉角pitch和roll，然后使用pitch roll和磁力计的数据进行互补滤波融合得到Yaw，即可使用，但是欧拉角有奇点；
// 5.或直接使用四元数；
// 6.重复4，即可更新姿态;

//总的来说，核心是陀螺仪，加速度计用来修正补偿Pitch和Roll，磁力计用来修正补偿Yaw;
//以下程序中，gx, gy, gz单位为弧度/s，ax, ay, az为加速度计输出的原始16进制数据, mx, my, mz为磁力计输出的原始16进制数据；
//前进方向：mpu9150的加速度计和陀螺仪的x轴为前进方向;
//以下程序采用的参考方向为：mpu9150的加速度计和陀螺仪所指的xyz方向为正方向；

//在量程为正负500度/s的前提下，陀螺仪的灵敏度是65.5LSB/度/s，所以把陀螺仪输出的十六进制数据除以65.5就是角速度，单位是°/s，
//然后再除以57.3就变成弧度制;(1弧度=180/pi=57.3度)

//欧拉角单位为弧度radian，乘以57.3以后转换为角度,0<yaw<360, -90<pitch<+90, -180<roll<180
***************************************************************************************************************************************/
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
   float norm;//, halfT;
   float hx, hy, hz, bz, by;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;
   float Pitch, Roll, Yaw;

/*方便之后的程序使用，减少计算时间*/
        //auxiliary variables to reduce number of repeated operations，
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
          
/*归一化测量值，加速度计和磁力计的单位是什么都无所谓，因为它们在此被作了归一化处理*/        
   //normalise the measurements
   norm = invSqrt(ax*ax + ay*ay + az*az);       
   ax = ax * norm;
   ay = ay * norm;
   az = az * norm;
   norm = invSqrt(mx*mx + my*my + mz*mz);          
   mx = mx * norm;
   my = my * norm;
   mz = mz * norm;         
        
/*从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值），下面这个是从飞行器坐标系到世界坐标系的转换公式*/
   //compute reference direction of flux
   hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
   hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
   hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);

/*计算地理坐标系下的磁场矢量bxyz（参考值）。
因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），我定义by指向正北，所以by=某值，bx=0
但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。这里修改by和bx指向可以定义哪个轴指向正北*/
//   bx = sqrtf((hx*hx) + (hy*hy));
   by = sqrtf((hx*hx) + (hy*hy));
   bz = hz;        
    
   // estimated direction of gravity and flux (v and w)，下面这个是从世界坐标系到飞行器坐标系的转换公式(转置矩阵)
   vx = 2*(q1q3 - q0q2);
   vy = 2*(q0q1 + q2q3);
   vz = q0q0 - q1q1 - q2q2 + q3q3;

/*我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
因为bx=0，所以所有涉及到bx的部分都被省略了。同理by=0，所以所有涉及到by的部分也可以被省略，这根据自己定义那个轴指北有关。
类似上面重力vxyz的推算，因为重力g的az=1，ax=ay=0，所以上面涉及到gxgy的部分也被省略了
你可以看看两个公式：wxyz的公式，把by换成ay（0），把bz换成az（1），就变成了vxyz的公式了（其中q0q0+q1q1+q2q2+q3q3=1）。*/
//   wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//   wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//   wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
   wx = 2*by*(q1q2 + q0q3) + 2*bz*(q1q3 - q0q2);
   wy = 2*by*(0.5 - q1q1 - q3q3) + 2*bz*(q0q1 + q2q3);
   wz = 2*by*(q2q3 - q0q1) + 2*bz*(0.5 - q1q1 - q2q2);
           
//现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
   // error is sum of cross product between reference direction of fields and direction measured by sensors
   ex = (ay*vz - az*vy) + (my*wz - mz*wy);
   ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
   ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
   
//   halfT=GET_NOWTIME();		//得到每次姿态更新的周期的一半
   
   if(ex != 0.0f && ey != 0.0f && ez != 0.0f)      //很关键的一句话，原算法没有
   {
      // integral error scaled integral gain
      exInt = exInt + ex*Ki * halfT;			   //乘以采样周期的一半
      eyInt = eyInt + ey*Ki * halfT;
      ezInt = ezInt + ez*Ki * halfT;
      // adjusted gyroscope measurements
      gx = gx + Kp*ex + exInt;
      gy = gy + Kp*ey + eyInt;
      gz = gz + Kp*ez + ezInt;
   }         

   // integrate quaternion rate and normalise，四元数更新算法
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
        
///*由四元数计算出Pitch  Roll  Yaw
//乘以57.3是为了将弧度转化为角度*/
		Yaw   = -atan2(2*q1*q2 - 2*q0*q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) * 57.3;  //偏航角，绕z轴转动	
    if(Yaw < 0 ){Yaw = Yaw + 360;}
		if(Yaw > 360 ){Yaw = Yaw - 360;}
//		Roll 		= asin	(2	*q2*q3 + 2*q0*q1) * 57.3; //俯仰角，绕x轴转动	 
//		Pitch   = -atan2(-2	*q0*q2 + 2*q1*q3, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3; //滚动角，绕y轴转动
   	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //俯仰角，绕y轴转动	 
    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //滚动角，绕x轴转动
//	if(Roll<0)Yaw+=Roll/2;
//	else if(Yaw<180) Yaw-=Roll/2;
//	send_line( Yaw, Pitch*100, Roll*100, Yaw, Pitch*10, Roll*10);//山外
		angle.yaw = Yaw; 
		angle.roll =Roll;
		angle.pitch =Pitch;
	
/*最初的由四元数计算出Pitch  Roll  Yaw
Roll=arctan2(2wx+2yz, 1-2xx-2yy);
Pitch=arcsin(2wy-2zx);
Yaw=arctan2(2wz+2xy, 1-2yy-2zz);
1=q0*q0+q1*q1+q2*q2+q3*q3;
乘以57.3是为了将弧度转化为角度*/
	
//	Pitch = asin(-2*q1*q3 + 2*q0*q2) * 57.3; //俯仰角，绕y轴转动	 
//    Roll  = atan2(2*q2*q3 + 2*q0*q1,-2*q1*q1 - 2*q2*q2 + 1) * 57.3; //滚动角，绕x轴转动
//	Yaw   = atan2(2*q1*q2 + 2*q0*q3,-2*q2*q2 - 2*q3*q3 + 1) * 57.3;  //偏航角，绕z轴转动

//	printf("q0=%f, q1=%f, q2=%f, q3=%f, Yaw=%f, Pitch=%f, Roll=%f, halfT=%f \n\r", q0, q1, q2, q3, Yaw, Pitch, Roll, halfT);
//    printf("Yaw=%f, Pitch=%f, Roll=%f \n\r", Yaw, Pitch, Roll);
}


/*******************************************************************************
快速计算 1/Sqrt(x)，源自雷神3的一段代码，神奇的0x5f3759df！比正常的代码快4倍 	
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
////#define halfT 0.00125f                 // 采样周期的一半  本程序 2.5MS 采集一次  所以 halfT是1.25MS
//#define halfT 0.01f                 // 采样周期的一半  本程序 2.5MS 采集一次  所以 halfT是1.25MS
//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
//float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
// 
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
//{
//		float norm;
//		int16_t Xr,Yr;
//		float vx, vy, vz;// wx, wy, wz;
//		float ex, ey, ez;

//		// 先把这些用得到的值算好
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
//	//这是对于6050数据不合理的排除	
//		if(ax*ay*az==0)
//			return;
//			
//		norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
//		ax = ax *norm;
//		ay = ay * norm;
//		az = az * norm;

//		// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
//		vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
//		vy = 2*(q0q1 + q2q3);
//		vz = q0q0 - q1q1 - q2q2 + q3q3 ;

//		// error is sum of cross product between reference direction of fields and direction measured by sensors
//		ex = (ay*vz - az*vy) ;                           					 //向量外积在相减得到差分就是误差
//		ey = (az*vx - ax*vz) ;
//		ez = (ax*vy - ay*vx) ;

//		exInt = exInt + VariableParameter(ex) * ex * Ki;								  //对误差进行积分
//		eyInt = eyInt + VariableParameter(ey) * ey * Ki;
//		ezInt = ezInt + VariableParameter(ez) * ez * Ki;
//	// adjusted gyroscope measurements

//		gx = gx + Kp *  VariableParameter(ex) * ex + exInt;	
//		gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
//		gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
//										
//		// integrate quaternion rate and normalise						   //四元素的微分方程
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
//		//          关于地磁如何进行倾角补偿                       //    
//		//参考  http://baike.baidu.com/view/1239157.htm?fr=aladdin    //
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
																				//比例增益控制收敛速度加速度计和磁强计
#define Ki 0.002f                          // integral gain governs rate of convergence of gyroscope biases
																				//积分增益控制收敛速率陀螺仪的偏见
#define halfT 0.001f                   // half the sample period采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation四元数元素代表估计取向
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error 比例积分误差
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az ,float mx, float my, float mz)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // 先把这些用得到的值算好
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
  //把加计的三维向量转成单位向量。

  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;

  //从机体坐标系的电子罗盘测到的矢量转成地理坐标系下的磁场矢量hxyz（测量值）
  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
  
//  计算地理坐标系下的磁场矢量bxyz（参考值）。
//  因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），所以by=0，bx=某值
//  但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
//  我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
//  磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
//  因为by=0，所以就简化成(bx*bx)  = ((hx*hx) + (hy*hy))。可算出bx。
           
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
 
//  这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
//  根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
//  所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
  
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
 
 // 我们把地理坐标系上的磁场矢量bxyz，转到机体上来wxyz。
 // 因为by=0，所以所有涉及到by的部分都被省略了。
 // 类似上面重力vxyz的推算，因为重力g的gz=1，gx=gy=0，所以上面涉及到gxgy的部分也被省略了
  
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  //现在把加速度的测量矢量和参考矢量做叉积，把磁场的测量矢量和参考矢量也做叉积。都拿来来修正陀螺。
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  
 // axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
//  axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
// 那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
//  向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
//  这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
  
if(ex != 0.0f && ey != 0.0f && ez != 0.0f){
  exInt = exInt + ex * Ki * halfT;
  eyInt = eyInt + ey * Ki * halfT;	
  ezInt = ezInt + ez * Ki * halfT;

  // 用叉积误差来做PI修正陀螺零偏
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

  }

  // 四元数微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // 四元数规范化
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  angle.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* RtA; // yaw
  angle.pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* RtA; // pitch
  angle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll
}*/

