#ifndef _AHRS_H_
#define _AHRS_H_
#include "stm32f10x.h"

typedef struct{
				float pitch;
				float roll;
        float yaw;}T_float_angle;
typedef struct{
				float X;
				float Y;
				float Z;}T_float_xyz;
typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}T_int16_xyz;
typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t AUX1;
				int16_t AUX2;
				int16_t AUX3;
				int16_t AUX4;
				int16_t AUX5;
				int16_t AUX6;}T_RC_Data;
typedef struct{
				u8	ARMED;}T_RC_Control;

//typedef struct {
//        float pitch;
//				float roll;
//        float yaw;}angle;

extern float 	AngleOffset_Rol,AngleOffset_Pit; 

void Prepare_Data(T_int16_xyz *acc_in,T_int16_xyz *acc_out);

//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az ,float mx, float my, float mz);
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
				
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);				
float invSqrt(float x);
void init_quaternion(void);
				

#endif
