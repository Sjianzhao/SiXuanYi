#include "stm32f10x.h"

void PID_roll_Control( float roll_set);
void PID_pitch_Control( float pitch_set);
void PID_yaw_Control( float yaw_set);

void PID_control(void);
void PWM_control(void);

float averag_gyro_Y(float value);
float averag_gyro_OUT_Y(float value);

float averag_gyro_X(float value);
float averag_gyro_OUT_X(float value);

float averag_gyro_Z(float value);
float averag_gyro_OUT_Z(float value);

