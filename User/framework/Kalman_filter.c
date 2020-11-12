#include "Kalman_filter.h"
#include "AHRS.h"

extern  T_float_xyz origin_Acel;//修正后的值加速度
extern  T_float_xyz averag_Acel; //卡尔曼滤波后的加速度数据

extern  T_float_xyz radian_gyro;   //角度的弧度值角度
extern  T_float_xyz averag_gyro; //卡尔曼滤波后的角度数据
extern  T_float_xyz value_gyro; //校准后的数据

void DATA_Kalman_Filter(void)
{
			averag_Acel.X = KalmanFilter_x(origin_Acel.X,KALMAN_Q,KALMAN_R);  // ACC X轴卡尔曼滤波
			averag_Acel.Y = KalmanFilter_y(origin_Acel.Y,KALMAN_Q,KALMAN_R);  // ACC Y轴卡尔曼滤波   加速度
			averag_Acel.Z = KalmanFilter_z(origin_Acel.Z,KALMAN_Q,KALMAN_R);  // ACC Z轴卡尔曼滤波
//	    averag_Acel.X =origin_Acel.X;
//			averag_Acel.Y =origin_Acel.Y;
//			averag_Acel.Z = origin_Acel.Z;
//			averag_gyro.X = KalmanFilter_Gx(value_gyro.X,KALMAN_Q_G,KALMAN_R_G);  // ACC X轴卡尔曼滤波
////			averag_gyro.X = KalmanFilter_Gx2(averag_gyro.X,KALMAN_Q_G,KALMAN_R_G);  // ACC X轴卡尔曼滤波
//			averag_gyro.Y = KalmanFilter_Gy(value_gyro.Y,KALMAN_Q_G,KALMAN_R_G);  // ACC Y轴卡尔曼滤波   加速度
//			averag_gyro.Z = KalmanFilter_Gz(value_gyro.Z,KALMAN_Q_G,KALMAN_R_G);  // ACC Z轴卡尔曼滤波
}


/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/

/*           卡尔曼对三个轴加速度进行滤波处理           */
static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last;           //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R);   //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
	
   return x_now;                
}

static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
	
   return x_now;                
}

static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
	
   return x_now;                
}

/****************角度******************/

static double KalmanFilter_Gx(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last;           //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R);   //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
	
   return x_now;                
}

static double KalmanFilter_Gy(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
	
   return x_now;                
}

static double KalmanFilter_Gz(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
	
   return x_now;                
}

//static double KalmanFilter_Gx2(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
//{
//   double R = MeasureNoise_R;
//   double Q = ProcessNiose_Q;
//   static double x_last;
//   double x_mid = x_last;
//   double x_now;
//   static double p_last;
//   double p_mid ;
//   double p_now;
//   double kg;        

//   x_mid=x_last;           //x_last=x(k-1|k-1),x_mid=x(k|k-1)
//   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
//   kg=p_mid/(p_mid+R);   //kg为kalman filter，R为噪声
//   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
//                
//   p_now=(1-kg)*p_mid;//最优值对应的covariance       
//   p_last = p_now; //更新covariance值
//   x_last = x_now; //更新系统状态值
//	
//   return x_now;                
//}
