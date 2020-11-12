/*
����ж���Ҫ��IO�����ã����ʹ��BSRR��BRR����������Ҫ��ODR .

ODR ���ɶ���д��ֻ������(16λ)����ʽ����


BSSR��ֻ��д�벢ֻ������(16λ)����ʽ���������ͬʱ�����˸�16bit�͵�16bit�����16bit�����á�

BRR��ֻ��д�벢ֻ������(16λ)����ʽ����


GPIOE->BSRR = 0x80; // ��'1'
GPIOE->BRR = 0x80; // ��'0'

*/
#include "i2c.h"
#include "bsp_SysTick.h"
#include "mpu6050.h"


void gpio_i2c(void)//IIC GPIO����
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C_SDA_OUT(void)//SDA����Ϊ���
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//�������
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void I2C_SDA_IN(void)//SDA����Ϊ����
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void Delay_600ns(void)
{
		 __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//���۵�����ָ�� 1/72M=13.89ns ���41.98ns
}

void I2C_Start()//��ʼ�ź�
{
    SETSDA; 	//����SDA     //����SDA
//	  GPIOB->BSRR = 0x80; // ��'1' ���ԼĴ�������
    SETSCL;  //����SCL    //����SCL 
    
   __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); //170ns
	
//		Delay_us(1); //��ʱ5us                
    CLRSDA;      //����SDA             
//    Delay_us(1); //��ʱ5us
		Delay_600ns();
    CLRSCL;  //����SCL     //����SCL              
}

void I2C_Stop()//ֹͣ�ź�
{
    CLRSDA;  //����SDA                 
    SETSCL;  //����SCL  //����SCL                  
//    Delay_us(1); //��ʱ5us 
		Delay_600ns();	
    SETSDA; 	//����SDA  //����SDA                   
    Delay_us(1);//��ʱ5us                  
}

void I2C_SendACK(u8 ack)//iic ����Ӧ��
{
    if(ack)  
		{
      SETSDA; 	//����SDA 
		}
    else
		{			
			CLRSDA;   //����SDA
		}
    SETSCL;  //����SCL                    
    Delay_600ns();//Delay_us(1);                 
    CLRSCL ; //����SCL 
			
//    Delay_us(1);                
}

u8 I2C_RecvACK()//����Ӧ��
{
    u8 cy;
    SETSCL;  //����SCL                   
//    I2C_SDA_IN();   //SDA��Ϊ����             
    Delay_us(1); 
	  
    if(RED_SDA) cy = 1;            
    else 				cy=0;
		
    CLRSCL;  //����SCL                   
    Delay_us(1);                 
//   I2C_SDA_OUT();//SD��Ϊ���
    return cy;
}

void I2C_SendByte(u8 dat)//����һ���ֽڵ�IIC����
{
    u8 i;
    for (i=0; i<8; i++)         
    {
        if((dat<<i)&0x80)       
				{
					SETSDA; 	//����SDA 
				}					
        else 
				{
					CLRSDA;
				}
        SETSCL;  //����SCL                
//        Delay_us(1);
				Delay_600ns();
        CLRSCL;  //����SCL               
        Delay_us(1);            
    }
//    I2C_RecvACK();
	
				SETSCL;  //����SCL    	//ACK����                            
//				Delay_us(1); 
		    Delay_600ns();
				CLRSCL;  //����SCL                   
//				Delay_us(1);
}

u8 I2C_RecvByte()//��IIC���߶�һ���ֽ�
{
    u8 i;
    u8 dat = 0,cy;
    SETSDA; 	//����SDA  	                   
//    I2C_SDA_IN();				//SDA��Ϊ����
    for (i=0; i<8; i++)         
    {
        dat <<= 1;
        SETSCL;  //����SCL                
//        Delay_us(1);
         
				if(RED_SDA)         //SDA����Ϊ1   
          cy=1;
        else cy=0;
        dat|=cy;
        CLRSCL;  //����SCL	
        Delay_us(1);             
    }
//    I2C_SDA_OUT();			//SDA��Ϊ���
    return dat;
}
/*****************
	��IIC�豸дһ���ֽ�����
******************/
void Single_WriteI2C(u8 REG_Address,u8 REG_data)
{
    I2C_Start();       //д��ʼ�ź�           
    I2C_SendByte(BMP280Slave_Address1); //д��ַ��д�ź�  
    I2C_SendByte(REG_Address);  //д�Ĵ�����ַ
    I2C_SendByte(REG_data);     //д�Ĵ�������  
    I2C_Stop();                 //дֹͣ�ź�  
}
/*****************
	��IIC�ӻ��豸дһ���ֽ�����
******************/
void IIC_add_WriteI2C(u8 add,u8 REG_Address,u8 REG_data)
{
    I2C_Start();       //д��ʼ�ź�           
    I2C_SendByte(add); //д��ַ��д�ź�  
    I2C_SendByte(REG_Address);  //д�Ĵ�����ַ
    I2C_SendByte(REG_data);     //д�Ĵ�������  
    I2C_Stop();                 //дֹͣ�ź�  
}
/**************************************
��IIC�豸��һ���ֽ�����
**************************************/
u8 Single_ReadI2C(u8 REG_Address)
{
	u8 REG_data;
	I2C_Start();                   //д��ʼ�ź�  
	I2C_SendByte(BMP280Slave_Address1);    //д��ַ��д�ź� 
	I2C_SendByte(REG_Address);     //д�Ĵ�����ַ
	I2C_Start();                    //д��ʼ�ź� 
	I2C_SendByte(BMP280Slave_Address1+1);   //д��ַ�Ͷ��ź�
	REG_data=I2C_RecvByte();        //������
	I2C_SendACK(1);                //����Ӧ��
	I2C_Stop();                    //ֹͣ�ź�
	return REG_data;								
}
/**************************************
��IIC�ӻ���ַ�豸��һ���ֽ�����
@ add :ic��ַ
@ REG_Address
**************************************/
u8 Single_add_ReadI2C(u8 add,u8 REG_Address)
{
	u8 REG_data;
	I2C_Start();                   //д��ʼ�ź�  
	I2C_SendByte(add);    //д��ַ��д�ź� 
	I2C_SendByte(REG_Address);     //д�Ĵ�����ַ
	I2C_Start();                    //д��ʼ�ź� 
	I2C_SendByte(add+1);   //д��ַ�Ͷ��ź�
	REG_data=I2C_RecvByte();        //������
	I2C_SendACK(1);                //����Ӧ��
	I2C_Stop();                    //ֹͣ�ź�
	return REG_data;								
}

/**************************************
��IIC�豸��һ������
icadd:		�ӻ���ַ   ReadAddr���Ĵ�����ַ   
pBuffer����ŵ����� Num������
**************************************/
//u8 Single_add_ReadI2C(u8 add,u8 REG_Address��u16 num)
//void IIC_ReadData(u8 icadd,u8 ReadAddr,u8* pBuffer, u16 Num)
//{
//	while(Num--)
//	{	
//		*pBuffer=Single_add_ReadI2C(icadd,ReadAddr++);        //������
//		pBuffer++;
//	}									
//}
void IIC_ReadData(u8 icadd,u8 ReadAddr,u8* pBuffer, u16 Num)
{
		I2C_Start();                   //д��ʼ�ź�  
		I2C_SendByte(icadd);    //д��ַ��д�ź� 	
		I2C_SendByte(ReadAddr);     //д�Ĵ�����ַ
	
	  I2C_Start();                //д��ʼ�ź�
	  I2C_SendByte(icadd+1);   //д��ַ�Ͷ��ź�
	
		while(Num--)
		{
			*pBuffer=I2C_RecvByte();        //������
			pBuffer++;
			if(!Num) 	I2C_SendACK(1);                //����Ӧ��			
			else 			I2C_SendACK(0);                //����Ӧ��		
	  }
		I2C_Stop();                    //ֹͣ�ź�
		
}


