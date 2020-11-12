#include "mpu6050.h"
#include "GPIO_Config.h"
#include "bsp_i2c.h"
//#include "bsp_led.h"
#include "bsp_SysTick.h"
#include "i2c.h"
#include "LED_config.h"

unsigned char TX_DATA[4];  	 //显示据缓存区
unsigned char BUF[10];       //接收数据缓存区
char  test=0; 				 //IIC用到
short T_X,T_Y,T_Z,T_T;		 //X,Y,Z轴，温度

void PMU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	I2C_EE_ByteWrite(reg_dat,reg_add);
}

void PMU6050_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
	I2C_EE_BufferRead(Read,reg_add,num);
}

/*     -----     PMU9250     -----     */
void PMU9250_WriteReg(u8 reg_add,u8 reg_dat)
{
	I2C_AK_ByteWrite(reg_dat,reg_add);
}

void PMU9250_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
	I2C_AK_BufferRead(Read,reg_add,num);
}
/*     -----     PMU9250     -----     */

void MPU6050_Init(void)
{
  int i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	Delay_us(5000);
//	IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_PWR_MGMT_1,0x00);
//	IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_SMPLRT_DIV,0x07);
//	IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_CONFIG,0x06);
//	IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_ACCEL_CONFIG,0x01);
//	IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_GYRO_CONFIG,0x18);
	
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_PWR_MGMT_1,0x00); //解除休眠状态
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_SMPLRT_DIV,0x07); //采样率
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_CONFIG,0x03); //低通滤波		
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_GYRO_CONFIG,0x08);   //陀螺仪量程+-500度
	  IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_ACCEL_CONFIG,0x08);  //加速度量程+-4g
	
			IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_FF_THR,0x06); //加速度低通滤波		
	
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,0x37,0x02);//旁路模式
//		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x01); //工作在模式1	14位输出，单次测量
    IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x12); //工作在模式2	16位输出，连续次测量
	
    IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x10,0xFF);
		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x11,0xFF);  //灵敏度调节
		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x12,0xFF);

	
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //解除休眠状态
//	PMU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //陀螺仪采样率
//	PMU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	
//	PMU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //配置加速度传感器工作在16G模式
//	PMU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
	Delay_us(100);
	
//	PMU9250_WriteReg(MPU9250_RA_MAG_CONFIG, 0x18);
}
void MPU6050ReadID(void)
{
	unsigned char Re = 0;
    PMU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
     printf("%d\r\n",Re);
}
void MPU6050ReadAcc(short *accData)
{
    u8 buf[6];
//    PMU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
		IIC_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_ACC_OUT,buf,6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}
void MPU6050ReadGyro(short *gyroData)
{
    u8 buf[6];
		IIC_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_GYRO_OUT,buf,6);
//    PMU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050Read_all(short *accData,short *gyroData,short *tempData)
{
    u8 buf[14];
		IIC_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_ACC_OUT,buf,14);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
	
	  tempData[0] = (buf[6] << 8) | buf[7];
	
	  gyroData[0] = (buf[8] << 8) | buf[9];
    gyroData[1] = (buf[10] << 8) | buf[11];
    gyroData[2] = (buf[12] << 8) | buf[13];
	
}

/*    ********    MPU9250ReadMag    ********    */
void MPU9250ReadMag(short *magData)
{
    u8 buf[6];
//		Delay_us(1000); //原始1000
//		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,0x37,0x02);//原始未屏蔽
//		PMU6050_WriteReg(0X37, 0X02);
//	  Delay_us(1000);//原始4000
	
/*		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x01); //工作在模式1  */
//		PMU9250_WriteReg(0x0A, 0x01);
//		Delay_us(7200);//原始7000
//    PMU9250_ReadData(MPU9250_MAG_OUT,buf,6);
		IIC_ReadData(MPU9250_SLAVE_ADDRESS,MPU9250_MAG_OUT,buf,6);
    magData[0] = (buf[1] << 8) | buf[0];
    magData[1] = (buf[3] << 8) | buf[2];
    magData[2] = (buf[5] << 8) | buf[4];
		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x01); //工作在模式1
}

void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReturnTemp(short *Temperature)
{
	short temp3;
	u8 buf[2];
	
//	PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
	IIC_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_RA_TEMP_OUT_H,buf,2);
  temp3= (buf[0] << 8) | buf[1];
//	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
	*Temperature=((double)temp3)/340;  //36.53+
}


/*      ***********      MPU9250      ***********      */
