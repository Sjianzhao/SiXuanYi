

#include "stm32f10x.h"
/*库函数操作*/
//#define SETSCL GPIO_SetBits(GPIOB,GPIO_Pin_6)  		//拉高SCL	
//#define	CLRSCL	GPIO_ResetBits(GPIOB,GPIO_Pin_6)	//拉低SCL
//#define SETSDA GPIO_SetBits(GPIOB,GPIO_Pin_7); //拉高SDA
//#define	CLRSDA GPIO_ResetBits(GPIOB,GPIO_Pin_7)   //拉低SDA
//#define RED_SDA GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)//读SDA定义
/*寄存器操作*/
#define SETSCL 		GPIOB->BSRR	=0x40  		//拉高SCL	
#define	CLRSCL		GPIOB->BRR	=0x40			//拉低SCL
#define SETSDA 		GPIOB->BSRR	=0x80 		//拉高SDA
#define	CLRSDA 		GPIOB->BRR	=0x80   	//拉低SDA
#define RED_SDA 	(GPIOB->IDR & 0X0080) == 0X0080 //读SDA定义
void Single_WriteI2C(u8 REG_Address,u8 REG_data);
u8 Single_ReadI2C(u8 REG_Address);
void gpio_i2c(void);
void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);

void Delay_600ns(void);

u8 Single_add_ReadI2C(u8 add,u8 REG_Address);
void IIC_ReadData(u8 icadd,u8 ReadAddr,u8* pBuffer, u16 Num);
void IIC_add_WriteI2C(u8 add,u8 REG_Address,u8 REG_data);

#define BMP280Slave_Address1  0x77






































































