#include "Kalman_filter.h"
#include "AHRS.h"

extern  T_float_xyz origin_Acel;//�������ֵ���ٶ�
extern  T_float_xyz averag_Acel; //�������˲���ļ��ٶ�����

extern  T_float_xyz radian_gyro;   //�ǶȵĻ���ֵ�Ƕ�
extern  T_float_xyz averag_gyro; //�������˲���ĽǶ�����
extern  T_float_xyz value_gyro; //У׼�������

void DATA_Kalman_Filter(void)
{
			averag_Acel.X = KalmanFilter_x(origin_Acel.X,KALMAN_Q,KALMAN_R);  // ACC X�Ῠ�����˲�
			averag_Acel.Y = KalmanFilter_y(origin_Acel.Y,KALMAN_Q,KALMAN_R);  // ACC Y�Ῠ�����˲�   ���ٶ�
			averag_Acel.Z = KalmanFilter_z(origin_Acel.Z,KALMAN_Q,KALMAN_R);  // ACC Z�Ῠ�����˲�
//	    averag_Acel.X =origin_Acel.X;
//			averag_Acel.Y =origin_Acel.Y;
//			averag_Acel.Z = origin_Acel.Z;
//			averag_gyro.X = KalmanFilter_Gx(value_gyro.X,KALMAN_Q_G,KALMAN_R_G);  // ACC X�Ῠ�����˲�
////			averag_gyro.X = KalmanFilter_Gx2(averag_gyro.X,KALMAN_Q_G,KALMAN_R_G);  // ACC X�Ῠ�����˲�
//			averag_gyro.Y = KalmanFilter_Gy(value_gyro.Y,KALMAN_Q_G,KALMAN_R_G);  // ACC Y�Ῠ�����˲�   ���ٶ�
//			averag_gyro.Z = KalmanFilter_Gz(value_gyro.Z,KALMAN_Q_G,KALMAN_R_G);  // ACC Z�Ῠ�����˲�
}


/*	
	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
*/

/*           ����������������ٶȽ����˲�����           */
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
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R);   //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
	
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
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
	
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
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
	
   return x_now;                
}

/****************�Ƕ�******************/

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
   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R);   //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
	
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
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
	
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
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
   p_last = p_now; //����covarianceֵ
   x_last = x_now; //����ϵͳ״ֵ̬
	
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
//   p_mid=p_last+Q;        //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����
//   kg=p_mid/(p_mid+R);   //kgΪkalman filter��RΪ����
//   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ
//                
//   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance       
//   p_last = p_now; //����covarianceֵ
//   x_last = x_now; //����ϵͳ״ֵ̬
//	
//   return x_now;                
//}
