#include "stm32f10x.h"

#define RtA 	57.324841f		//  180/3.1415  角度制 转化为弧度制		
#define AtR    	0.0174533f		//  1/RtA             RtA倒数		
#define Acc_G 	0.0011963f		//  1/32768/4/9.8     加速度量程为4G		
#define Gyro_G 	0.03051756f	//  1/32768/1000      陀螺仪量程为 +—1000			
#define Gyro_Gr	0.00020f   //  1/32768/1000/57.3 

/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/
#define KALMAN_Q        0.00002f
#define KALMAN_R        0.0015f
//#define KALMAN_Q        0.02
//#define KALMAN_R        6.0000

//#define KALMAN_Q_G      0.00002f
//#define KALMAN_R_G      0.01f
#define KALMAN_Q_G      0.1f
#define KALMAN_R_G      0.1f

void DATA_Kalman_Filter(void);

static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);

static double KalmanFilter_Gx(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_Gy(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_Gz(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);

static double KalmanFilter_Gx2(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);

//float Q_rsqrt(float number);
//float FL_ABS(float x);
//float COS(float x);
//float SIN(float y);

