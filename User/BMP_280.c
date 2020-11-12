#include "BMP_280.h"
#include "math.h"
#include "GPIO_Config.h"
#include "i2c.h"

//��ѹ������BMP280

signed t_fine;
s32 temp_sb,press_sb; //��������
unsigned short dig_T1,dig_P1;//�¶���ѹ����
short dig_T2,dig_T3;//�¶Ȳ���
short dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9; //��ѹ����
//s32 TEMP_280; //280�¶�
s32 Pa;       //��ѹ

//I2C1���ߵ����� 
//PB6->SCL  PB7->SDA

//BMP280��ʼ������
void BMP280_Init(void)
{
//     BMP280_I2C1WriteByte(BMP280_REGISTER_CONTROL,0xff);
	   IIC_add_WriteI2C(BMP280Slave_Address,BMP280_REGISTER_CONTROL,0xff);
//		Delay_us(2000);
}



//BMP280��ѹ������  I2C1�����ϵ�д����

/**************************************
��IIC�豸дһ���ֽ���������
&��addr:��ַ   dat:����
**************************************/
void BMP280_I2C1WriteByte(u8 addr,u8 dat)
{
	I2C_GenerateSTART(I2C1, ENABLE); //�����źŴ���  ��ʼ�ź�
	
  	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT));//�ȴ���־λ��ȷ��
	   
  	I2C_Send7bitAddress(I2C1,BMP280Slave_Address,I2C_Direction_Transmitter);//���ʹӻ���ַ
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    	
  	I2C_SendData(I2C1,addr);//���͵�ַ
	
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
  	I2C_SendData(I2C1, dat); //��������
  	while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	
  	I2C_GenerateSTOP(I2C1,ENABLE);//ֹͣ�ź�
 
}


/**************************************
��IIC�豸��һ����������
**************************************/
void BMP280_I2C2ReadData(u8 ReadAddr,u8* pBuffer, u16 NumByteToRead)
{  
  //*((u8 *)0x4001080c) |=0x80; 
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)); // Added by Najoua 27/08/2008    
	
		I2C_GenerateSTART(I2C1, ENABLE);//��ʼ�ź�  /* Send START condition */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));//*EV5,��ģʽ*//* Test on EV5 and clear it */
    
		I2C_Send7bitAddress(I2C1, BMP280Slave_Address, I2C_Direction_Transmitter); //�㲥7λ�ӻ���ַ д����/* Send AK address for write */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  //* Test on EV6 and clear it */
	   
		I2C_Cmd(I2C1, ENABLE); /* Clear EV6 by setting again the PE bit */
		
		I2C_SendData(I2C1, ReadAddr);/* Send the AK's internal address to write to */  	
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));	/* Test on EV8 and clear it */ 
   
		I2C_GenerateSTART(I2C1, ENABLE);  //��ʼ�ź�       /* Send STRAT condition a second time */  
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); /* Test on EV5 and clear it */  
		
		I2C_Send7bitAddress(I2C1, BMP280Slave_Address, I2C_Direction_Receiver);//�㲥7λ�ӻ���ַ /* Send AK address for read */  
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));/* Test on EV6 and clear it */
   
		while(NumByteToRead) /* While there is data to be read */ 
		{
			if(NumByteToRead == 1)
			{				
				I2C_AcknowledgeConfig(I2C1, DISABLE); //���һλ��Ҫ�ر�Ӧ���				
				I2C_GenerateSTOP(I2C1, ENABLE); //����ֹͣλ
			}			
			if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))/* Test on EV7 and clear it */  
			{      				
				*pBuffer = I2C_ReceiveData(I2C1);/* Read a byte from the AK */				
				pBuffer++; /* Point to the next location where the byte read will be saved */								
				NumByteToRead--; /* Decrement the read bytes counter */       
			}   
		}
		I2C_AcknowledgeConfig(I2C1, ENABLE); //�ٴ�����Ӧ��ģʽ
}

/*
signed t_fine;
s32 temp_sb,press_sb; //��������
unsigned short dig_T1,dig_P1;//�¶���ѹ����
short dig_T2,dig_T3;//�¶Ȳ���
short dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9; //��ѹ����
s32 TEMP_280; //280�¶�
s32 Pa;       //��ѹ
*/

//��ȡ�¶�ֵ
void BMP280_GetTemperature(double* TEMP_280)
{
		u8 buf[6];
		s32 var1,var2;
//		double TEMP_280;
//	float TEMP_280;
//	double var1,var2;
		IIC_ReadData(BMP280Slave_Address,BMP280_REGISTER_TEMPDATA,buf,3);
//    BMP280_I2C2ReadData(BMP280_REGISTER_TEMPDATA,buf,3);//��ȡ�¶Ȼ�������
    temp_sb= (buf[2] >>4) |(buf[1] << 4) | buf[0]<<12;

//	  BMP280_I2C2ReadData(BMP280_REGISTER_DIG_T1,buf,6);//��ȡ�¶Ȳ���
	
		IIC_ReadData(BMP280Slave_Address,BMP280_REGISTER_DIG_T1,buf,6);
    dig_T1=(buf[1] << 8) | buf[0];
		dig_T2=(buf[3] << 8) | buf[2];
		dig_T3=(buf[5] << 8) | buf[4];
	
	  var1=(((double) temp_sb)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
		var2=((((double)temp_sb)/131072.0-((double)dig_T1)/8192.0)*(((double)temp_sb)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
	
		t_fine=var1+var2;	
	  *TEMP_280=(var1+var2)/5120.0-10.0;
	
		
}
//��ȡ��ѹֵ
void BMP280_GetPressure(s32* Pressure)
{
	  u8 buf[18];
		s32 var1,var2;
//	 s32 high;
//	double var1,var2;	
//		double TEMP_280;
//	float TEMP_280;

	  IIC_ReadData(BMP280Slave_Address,BMP280_REGISTER_PRESSUREDATA,buf,3);
//		BMP280_I2C2ReadData(BMP280_REGISTER_PRESSUREDATA,buf,3);//��ȡ��ѹ��������
    press_sb= (buf[2] >>4) |(buf[1] << 4) | buf[0]<<12;
	
	  IIC_ReadData(BMP280Slave_Address,BMP280_REGISTER_DIG_P1,buf,18);
//	  BMP280_I2C2ReadData(BMP280_REGISTER_DIG_P1,buf,18);//��ȡ��ѹ����
    dig_P1=(buf[1]  << 8) | buf[0];
		dig_P2=(buf[3]  << 8) | buf[2];
		dig_P3=(buf[5]  << 8) | buf[4];
		dig_P4=(buf[7]  << 8) | buf[6];
		dig_P5=(buf[9]  << 8) | buf[8];
		dig_P6=(buf[11] << 8) | buf[10];
		dig_P7=(buf[13] << 8) | buf[12];
		dig_P8=(buf[15] << 8) | buf[14];
		dig_P9=(buf[17] << 8) | buf[16];
	
		var1=((double)t_fine/2.0)-64000.0;
		var2=var1*var1*((double)dig_P6)/32768.0;
		var2=var2+var1*((double)dig_P5)*2.0;
		var2=(var2/4.0)+(((double)dig_P4)*65536.0);
		var1=(((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
		var1=(1.0+var1/32768.0)*((double)dig_P1);
		
		Pa=1048576.0-(double)press_sb;
		Pa=(Pa-(var2/4096.0))*6250.0/var1;
		var1=((double)dig_P9)*Pa*Pa/2147483648.0;
		var2=Pa*((double)dig_P8)/32768.0;
		*Pressure=Pa+(var1+var2+((double)dig_P7))/16.0;
//		high=Pa-100530.0;
//		printf("\t��ѹΪ %d��\n\n\n",Pa);
}

