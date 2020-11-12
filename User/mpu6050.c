#include "mpu6050.h"
#include "GPIO_Config.h"
#include "bsp_i2c.h"
//#include "bsp_led.h"
#include "bsp_SysTick.h"
#include "i2c.h"
#include "LED_config.h"

unsigned char TX_DATA[4];  	 //��ʾ�ݻ�����
unsigned char BUF[10];       //�������ݻ�����
char  test=0; 				 //IIC�õ�
short T_X,T_Y,T_Z,T_T;		 //X,Y,Z�ᣬ�¶�

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
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
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
	
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_PWR_MGMT_1,0x00); //�������״̬
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_SMPLRT_DIV,0x07); //������
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_CONFIG,0x03); //��ͨ�˲�		
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_GYRO_CONFIG,0x08);   //����������+-500��
	  IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_ACCEL_CONFIG,0x08);  //���ٶ�����+-4g
	
			IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,MPU6050_RA_FF_THR,0x06); //���ٶȵ�ͨ�˲�		
	
		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,0x37,0x02);//��·ģʽ
//		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x01); //������ģʽ1	14λ��������β���
    IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x12); //������ģʽ2	16λ����������β���
	
    IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x10,0xFF);
		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x11,0xFF);  //�����ȵ���
		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x12,0xFF);

	
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //�������״̬
//	PMU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //�����ǲ�����
//	PMU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	
//	PMU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //���ü��ٶȴ�����������16Gģʽ
//	PMU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
	Delay_us(100);
	
//	PMU9250_WriteReg(MPU9250_RA_MAG_CONFIG, 0x18);
}
void MPU6050ReadID(void)
{
	unsigned char Re = 0;
    PMU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
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
//		Delay_us(1000); //ԭʼ1000
//		IIC_add_WriteI2C(MPU6050_SLAVE_ADDRESS,0x37,0x02);//ԭʼδ����
//		PMU6050_WriteReg(0X37, 0X02);
//	  Delay_us(1000);//ԭʼ4000
	
/*		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x01); //������ģʽ1  */
//		PMU9250_WriteReg(0x0A, 0x01);
//		Delay_us(7200);//ԭʼ7000
//    PMU9250_ReadData(MPU9250_MAG_OUT,buf,6);
		IIC_ReadData(MPU9250_SLAVE_ADDRESS,MPU9250_MAG_OUT,buf,6);
    magData[0] = (buf[1] << 8) | buf[0];
    magData[1] = (buf[3] << 8) | buf[2];
    magData[2] = (buf[5] << 8) | buf[4];
		IIC_add_WriteI2C(MPU9250_SLAVE_ADDRESS,0x0a,0x01); //������ģʽ1
}

void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReturnTemp(short *Temperature)
{
	short temp3;
	u8 buf[2];
	
//	PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
	IIC_ReadData(MPU6050_SLAVE_ADDRESS,MPU6050_RA_TEMP_OUT_H,buf,2);
  temp3= (buf[0] << 8) | buf[1];
//	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
	*Temperature=((double)temp3)/340;  //36.53+
}


/*      ***********      MPU9250      ***********      */
