#ifndef __BMP280_H__
#define __BMP280_H__

#include "stm32f10x.h"
#include "bsp_SysTick.h"
//#include "bsp_i2c.h"
//#include "system.h"

//从机地址   I2C1总线上 BMP280的地址
#define BMP280Slave_Address  0xec


#define  BMP280_REGISTER_DIG_T1          0x88
#define  BMP280_REGISTER_DIG_T2          0x8A
#define  BMP280_REGISTER_DIG_T3          0x8C

#define  BMP280_REGISTER_DIG_P1          0x8E
#define  BMP280_REGISTER_DIG_P2          0x90
#define  BMP280_REGISTER_DIG_P3          0x92
#define  BMP280_REGISTER_DIG_P4          0x94
#define  BMP280_REGISTER_DIG_P5          0x96
#define  BMP280_REGISTER_DIG_P6          0x98
#define  BMP280_REGISTER_DIG_P7          0x9A
#define  BMP280_REGISTER_DIG_P8          0x9C
#define  BMP280_REGISTER_DIG_P9          0x9E
        
#define  BMP280_REGISTER_CHIPID          0xD0
#define  BMP280_REGISTER_VERSION         0xD1
#define  BMP280_REGISTER_SOFTRESET       0xE0
        
#define  BMP280_REGISTER_CAL26           0xE1  // R calibration stored in 0xE1-0xF0

#define  BMP280_REGISTER_CONTROL         0xF4
#define  BMP280_REGISTER_CONFIG          0xF5
#define  BMP280_REGISTER_PRESSUREDATA    0xF7
#define  BMP280_REGISTER_TEMPDATA        0xFA

#define Standard_Pressure 101325   //单位pa



//函数声明
void I2C1_Configuration(void);
void BMP280_Init(void);//初始化
void BMP280_I2C1WriteByte(u8 addr,u8 dat);  //写入一个字节
//void BMP280_I2C2ReadData(u8 addr ,u8* pBuffer,u16 no);
void BMP280_I2C2ReadData(u8 ReadAddr,u8* pBuffer, u16 NumByteToRead);//读取一组数据

void BMP280_GetTemperature(double* TEMP_280);
void BMP280_GetPressure(s32* Pressure);

#endif
