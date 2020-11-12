#include "stm32f10x.h"
#include "nR24L01_Reg.h"
#include "i2c.h"
#include "mpu6050.h"
#include "BMP_280.h"
#include "ADC_DMA.h"
#include "UltrasonicWave.h"
#include "GPIO_Config.h"

#define uchar unsigned char
/*	
typedef struct{
				float rol;
				float pit;
				float yaw;}T_float_angle;
typedef struct{
				float X;
				float Y;
				float Z;}T_float_xyz;
typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}T_int16_xyz;
    */
void data_chang(void);
void data_init(void);
void MAG_data(short magX,short magY,short magZ);
void change(void);
void calibration_Mag(short magX,short magY,short magZ);
void calibration_GA(short GyroX,short GyroY,short GyroZ,short AcelX,short AcelY,short AcelZ);
void data_handling(void);//数据处理
void YAW_data_init(void);//数据初始化

