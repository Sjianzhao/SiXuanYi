/*
如果中断中要对IO口设置，最好使用BSRR和BRR操作，而不要用ODR .

ODR ：可读可写并只能以字(16位)的形式操作


BSSR：只能写入并只能以字(16位)的形式操作，如果同时设置了高16bit和低16bit，则低16bit起作用。

BRR：只能写入并只能以字(16位)的形式操作


GPIOE->BSRR = 0x80; // 置'1'
GPIOE->BRR = 0x80; // 置'0'

*/
#include "i2c.h"
#include "bsp_SysTick.h"
#include "mpu6050.h"


void gpio_i2c(void)//IIC GPIO设置
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//配置引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C_SDA_OUT(void)//SDA设置为输出
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//推挽输出
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void I2C_SDA_IN(void)//SDA设置为输入
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void Delay_600ns(void)
{
		 __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();//理论单周期指令 1/72M=13.89ns 测得41.98ns
}

void I2C_Start()//起始信号
{
    SETSDA; 	//拉高SDA     //拉高SDA
//	  GPIOB->BSRR = 0x80; // 置'1' 测试寄存器操作
    SETSCL;  //拉高SCL    //拉低SCL 
    
   __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP(); //170ns
	
//		Delay_us(1); //延时5us                
    CLRSDA;      //拉低SDA             
//    Delay_us(1); //延时5us
		Delay_600ns();
    CLRSCL;  //拉低SCL     //拉低SCL              
}

void I2C_Stop()//停止信号
{
    CLRSDA;  //拉低SDA                 
    SETSCL;  //拉高SCL  //拉高SCL                  
//    Delay_us(1); //延时5us 
		Delay_600ns();	
    SETSDA; 	//拉高SDA  //拉高SDA                   
    Delay_us(1);//延时5us                  
}

void I2C_SendACK(u8 ack)//iic 发送应答
{
    if(ack)  
		{
      SETSDA; 	//拉高SDA 
		}
    else
		{			
			CLRSDA;   //拉低SDA
		}
    SETSCL;  //拉高SCL                    
    Delay_600ns();//Delay_us(1);                 
    CLRSCL ; //拉低SCL 
			
//    Delay_us(1);                
}

u8 I2C_RecvACK()//接收应答
{
    u8 cy;
    SETSCL;  //拉高SCL                   
//    I2C_SDA_IN();   //SDA设为输入             
    Delay_us(1); 
	  
    if(RED_SDA) cy = 1;            
    else 				cy=0;
		
    CLRSCL;  //拉低SCL                   
    Delay_us(1);                 
//   I2C_SDA_OUT();//SD设为输出
    return cy;
}

void I2C_SendByte(u8 dat)//发送一个字节到IIC总线
{
    u8 i;
    for (i=0; i<8; i++)         
    {
        if((dat<<i)&0x80)       
				{
					SETSDA; 	//拉高SDA 
				}					
        else 
				{
					CLRSDA;
				}
        SETSCL;  //拉高SCL                
//        Delay_us(1);
				Delay_600ns();
        CLRSCL;  //拉低SCL               
        Delay_us(1);            
    }
//    I2C_RecvACK();
	
				SETSCL;  //拉高SCL    	//ACK部分                            
//				Delay_us(1); 
		    Delay_600ns();
				CLRSCL;  //拉低SCL                   
//				Delay_us(1);
}

u8 I2C_RecvByte()//从IIC总线读一个字节
{
    u8 i;
    u8 dat = 0,cy;
    SETSDA; 	//拉高SDA  	                   
//    I2C_SDA_IN();				//SDA设为输入
    for (i=0; i<8; i++)         
    {
        dat <<= 1;
        SETSCL;  //拉高SCL                
//        Delay_us(1);
         
				if(RED_SDA)         //SDA输入为1   
          cy=1;
        else cy=0;
        dat|=cy;
        CLRSCL;  //拉低SCL	
        Delay_us(1);             
    }
//    I2C_SDA_OUT();			//SDA设为输出
    return dat;
}
/*****************
	向IIC设备写一个字节数据
******************/
void Single_WriteI2C(u8 REG_Address,u8 REG_data)
{
    I2C_Start();       //写起始信号           
    I2C_SendByte(BMP280Slave_Address1); //写地址和写信号  
    I2C_SendByte(REG_Address);  //写寄存器地址
    I2C_SendByte(REG_data);     //写寄存器数据  
    I2C_Stop();                 //写停止信号  
}
/*****************
	向IIC从机设备写一个字节数据
******************/
void IIC_add_WriteI2C(u8 add,u8 REG_Address,u8 REG_data)
{
    I2C_Start();       //写起始信号           
    I2C_SendByte(add); //写地址和写信号  
    I2C_SendByte(REG_Address);  //写寄存器地址
    I2C_SendByte(REG_data);     //写寄存器数据  
    I2C_Stop();                 //写停止信号  
}
/**************************************
从IIC设备读一个字节数据
**************************************/
u8 Single_ReadI2C(u8 REG_Address)
{
	u8 REG_data;
	I2C_Start();                   //写起始信号  
	I2C_SendByte(BMP280Slave_Address1);    //写地址和写信号 
	I2C_SendByte(REG_Address);     //写寄存器地址
	I2C_Start();                    //写起始信号 
	I2C_SendByte(BMP280Slave_Address1+1);   //写地址和读信号
	REG_data=I2C_RecvByte();        //读数据
	I2C_SendACK(1);                //发送应答
	I2C_Stop();                    //停止信号
	return REG_data;								
}
/**************************************
从IIC从机地址设备读一个字节数据
@ add :ic地址
@ REG_Address
**************************************/
u8 Single_add_ReadI2C(u8 add,u8 REG_Address)
{
	u8 REG_data;
	I2C_Start();                   //写起始信号  
	I2C_SendByte(add);    //写地址和写信号 
	I2C_SendByte(REG_Address);     //写寄存器地址
	I2C_Start();                    //写起始信号 
	I2C_SendByte(add+1);   //写地址和读信号
	REG_data=I2C_RecvByte();        //读数据
	I2C_SendACK(1);                //发送应答
	I2C_Stop();                    //停止信号
	return REG_data;								
}

/**************************************
从IIC设备读一组数据
icadd:		从机地址   ReadAddr：寄存器地址   
pBuffer：存放的数组 Num：个数
**************************************/
//u8 Single_add_ReadI2C(u8 add,u8 REG_Address，u16 num)
//void IIC_ReadData(u8 icadd,u8 ReadAddr,u8* pBuffer, u16 Num)
//{
//	while(Num--)
//	{	
//		*pBuffer=Single_add_ReadI2C(icadd,ReadAddr++);        //读数据
//		pBuffer++;
//	}									
//}
void IIC_ReadData(u8 icadd,u8 ReadAddr,u8* pBuffer, u16 Num)
{
		I2C_Start();                   //写起始信号  
		I2C_SendByte(icadd);    //写地址和写信号 	
		I2C_SendByte(ReadAddr);     //写寄存器地址
	
	  I2C_Start();                //写起始信号
	  I2C_SendByte(icadd+1);   //写地址和读信号
	
		while(Num--)
		{
			*pBuffer=I2C_RecvByte();        //读数据
			pBuffer++;
			if(!Num) 	I2C_SendACK(1);                //发送应答			
			else 			I2C_SendACK(0);                //发送应答		
	  }
		I2C_Stop();                    //停止信号
		
}


