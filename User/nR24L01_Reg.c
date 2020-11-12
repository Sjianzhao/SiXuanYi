#include "nR24L01_Reg.h"
#include "GPIO_Config.h"
#include "bsp_SysTick.h"

//#include "NRF24L01.H"

#define uint unsigned int

uchar  nrf_sta;
uchar  RxBuf[32] = 	"0";	//接收缓存 存入idata区
uchar  TxBuf[32] = 	"0";	//发送缓存
	
uchar  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x32,0x24,0x01};	//本地地址
uchar  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x32,0x24,0x01};	//接收地址

//===== 粗略的延时 =====
//void delayus(uint us)
//{
//	while(us--);
//}
//================== NRF24L01初始化 ==================
void init_NRF24L01(void)
{
  Delay_us(100);
 	SPI_FLASH_CE_LOW();    // 片选使能
	SPI_FLASH_CS_HIGH();
// 	SPI_FLASH_CE_LOW();   // SPI时钟拉低
	SPI_FLASH_CLK_LOW();
	SPI_Write_Buf(WRITE_REGIST + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    		//写发送地址	
	SPI_Write_Buf(WRITE_REGIST + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); 	//写接收端地址
	SPI_Write_Reg(WRITE_REGIST + EN_AA, 0x01);                             			//通道0自动应答 	
	SPI_Write_Reg(WRITE_REGIST + EN_RXADDR, 0x01);                         	//允许接收地址频道0 
	SPI_Write_Reg(WRITE_REGIST + RF_CH, 0x35);                             			//设置信道工作频率，收发必须一致
	SPI_Write_Reg(WRITE_REGIST + RX_PW_P0, RX_PLOAD_WIDTH);                //设置接收数据长度
	SPI_Write_Reg(WRITE_REGIST + RF_SETUP, 0x08);   /*0x0F 2M 0x08 1M*/		   //设置发射速率为2MHZ，发射功率为最大值0dB	
	SPI_Write_Reg(WRITE_REGIST + NRF_CONFIG, 0x7c); 						//IRQ引脚不显示中断 掉电模式  1~16CRC校验
}
//==================
//读取状态标志
//==================
void get_nrf_sta(void)
{
	nrf_sta = SPI_Read_Reg(STATUS);
	SPI_Write_Reg(WRITE_REGIST+STATUS,0XFF);    
}
//==================
//设置为接收模式
//==================
void nrf_RxMod(void)
{
	SPI_FLASH_CE_LOW();
	SPI_Write_Reg(WRITE_REGIST+STATUS,0xff);	//清除中断标志
	SPI_Write_Reg(FLUSH_RX,0x00); 			//清除RX_FIFO寄存器
	SPI_Write_Reg(WRITE_REGIST + NRF_CONFIG, 0x7f);//IRQ引脚不显示中断 上电 接收模式   1~16CRC校验  
//Delay_us(100);	
	SPI_FLASH_CE_HIGH();
//	Delay_us(100);
}
//==================
//把接收到的数据存入数组
//==================
void nrf_read(uchar *rx_buf)
{
		if(RX_DR == 1)    		//收到数据
		{
			SPI_FLASH_CE_LOW();  
			SPI_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);//读取数据 存入数组
			SPI_Write_Reg(FLUSH_RX,0x00);//清除rx fifo寄存器
//			Delay_us(100);
			SPI_FLASH_CE_HIGH();
//      Delay_us(100);
		} 	 	
}
//==================
//设置为发送模式
//==================
void nrf_TxMod(void)
{
	SPI_FLASH_CE_LOW();
//Delay_us(100);
	
	SPI_Write_Reg(WRITE_REGIST+STATUS,0xff); 			//清除中断标志  	
	
//	Delay_us(100);
	SPI_Write_Reg(FLUSH_TX,0x00);								//清除TX_FIFO寄存器 
//	Delay_us(100);
	SPI_Write_Reg(WRITE_REGIST + NRF_CONFIG,0x7e);		//IRQ引脚不显示中断 上电 发射模式  1~16CRC校验
//	Delay_us(100);
	SPI_FLASH_CE_HIGH();            
//  Delay_us(100); 
}
//==================
//发送  不做任何判断只管发送
//==================
void nrf_trans(uchar *tx_buf)
{	
	SPI_FLASH_CE_LOW();			//StandBy I模式
//	Delay_us(100);
	SPI_Write_Reg(WRITE_REGIST+STATUS,0xFF);		//清除所有中断
//	Delay_us(100);
	SPI_Write_Reg(FLUSH_TX,0x00); 							//清除tx fifo寄存器	//===== 重要 =====
//	Delay_us(100);
	SPI_Write_Buf(WRITE_REGIST + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 		// 装载接收端地址
//	Delay_us(100);
	SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH); 			 						// 装载数据
//	Delay_us(100);
	SPI_FLASH_CE_HIGH();		 			//置高CE激发数据发送
//  Delay_us(100);printf("ABCD\n");		//此延时必须有 因为从待机模式到收发模式需要时间，最大需要130us
}
//=========================
//将float数编码装载 保留4位小数
//占用5个字节 数据范围+- 65535.9999
//=========================
void nrf_load_float(uchar a,float num)
{
	if(num > 0)
	{
		TxBuf[a] = '+';	
		TxBuf[a+1] = (uchar)num/256;  
		TxBuf[a+2] = (uchar)num%256;
		TxBuf[a+3] = (uchar)((num - (int)num)*10000)/256;
		TxBuf[a+4] = (uchar)((num - (int)num)*10000)%256;
	}
	else if(num < 0)
	{
		num = -num;
		TxBuf[a] = '-';	
		TxBuf[a+1] = (uchar)num/256;
		TxBuf[a+2] = (uchar)num%256;
		TxBuf[a+3] = (uchar)((num - (int)num)*10000)/256;
		TxBuf[a+4] = (uchar)((num - (int)num)*10000)%256;
	}
	else
	{
		TxBuf[a] = '0';
		TxBuf[a+1] = 0;
		TxBuf[a+2] = 0;
		TxBuf[a+3] = 0;
		TxBuf[a+4] = 0;
	}
}
//=======================
//将接收到的float数组解码
//占用5个字节 数据范围+- 65535.9999
//=======================
float nrf_unload_float(uchar a)
{
	float num;
	if(RxBuf[a] == '+'){
		num = RxBuf[a+1]*256 + RxBuf[a+2]+ (float)((int)RxBuf[a+3]*256 + RxBuf[a+4])/10000.0;
	}
	else if(RxBuf[a] == '-'){
		num = RxBuf[a+1]*256 + RxBuf[a+2]+ (float)((int)RxBuf[a+3]*256 + RxBuf[a+4])/10000.0;
		num = -num;
	}
	else if(RxBuf[a] == '0')
		num = 0;
	
	return (num);
}
//=======================
//将float数编码装载 保留2位小数
//占用3个字节 数据范围+- 255.99
//=======================
void nrf_load_sfloat(uchar a,float num)
{
	if(num > 0){
		TxBuf[a] = '+';	
		TxBuf[a+1] = (uchar)num;		//转换成uchar类型，自动将保留低8位，去除高位。
		TxBuf[a+2] = (uint)((num - (int)num)*100);
	}
	else if(num < 0){
		num = -num;
		TxBuf[a] = '-';	
		TxBuf[a+1] = (uchar)num;
		TxBuf[a+2] = (uint)((num - (int)num)*100);
	}
	else{
		TxBuf[a] = '0';
		TxBuf[a+1] = 0;
		TxBuf[a+2] = 0;
	}
}
//=======================
//将float数解码 保留2位小数
//占用3个字节 数据范围+- 255.99
//======================
float nrf_unload_sfloat(uchar a)	//a是数据包在数组内的起始位置
{
	float num;
	if(RxBuf[a] == '+'){
		num = RxBuf[a+1]+ (float)RxBuf[a+2]/100;
	}
	else if(RxBuf[a] == '-'){
		num = RxBuf[a+1]+ (float)RxBuf[a+2]/100;
		num = -num;
	}
	else if(RxBuf[a] == '0')
		num = 0;
	
	return (num);
}
/***********************SPI通讯*************************************/


////=============================================
////SPI读写函数，往MOSI引脚写入一个字节，同时从MISO引脚读取一个字节
////=============================================
//uchar SPI_RW(uchar num)
//{
//	uchar bit_ctr;
//   	for(bit_ctr=0;bit_ctr<8;bit_ctr++)  
//   	{
//		MOSI = (num & 0x80);            
//		num = (num << 1);               
//		SCK = 1;                        
//		num |= MISO;       		        
//		SCK = 0;            		    
//   	}
//    return(num);           		        
//}
//=============================================
//SPI读寄存器函数。只有一个功能：读取reg的值并返回
//先写入寄存器地址，使SPI器件定位到该地址，
//下一次读取取出该寄存器的值，并返回该值。
//=============================================
uchar SPI_Read_Reg(uchar reg)
{
	uchar reg_val;
	
	SPI_FLASH_CS_LOW();                
	SPI_FLASH_SendByte(reg);            
	reg_val = SPI_FLASH_SendByte(0);//此处的0没有什么意义，可以是1,2,3，……  
	SPI_FLASH_CS_HIGH();                
	
	return (reg_val);        
}
//=============================================
//SPI写寄存器函数。主要功能:往reg中写入value
//附加功能:返回寄存器的状态字
//=============================================
uchar SPI_Write_Reg(uchar reg, uchar value)
{
	uchar status;
	SPI_FLASH_CS_LOW();               
	status = SPI_FLASH_SendByte(reg);      
	SPI_FLASH_SendByte(value);            
	SPI_FLASH_CS_HIGH();                  
	return(status);            
}
//====================================
//将SPI设备缓冲区的数据读入单片机，并保存到数组中
//连续读取nBytes个字节
//====================================
uchar SPI_Read_Buf(uchar reg, uchar *pBuf, uchar nBytes)
{
	uchar status;
	uchar i;
	SPI_FLASH_CS_LOW(); 
	status = SPI_FLASH_SendByte(reg); 
	for(i = 0;i < nBytes;i++)
		pBuf[i] = SPI_FLASH_SendByte(0); 
	SPI_FLASH_CS_HIGH(); 
	return(status); 
}
//====================================
//将单片机有nBytes个成员的数组写入SPI设备的reg寄存器
//====================================
uchar SPI_Write_Buf(uchar reg, uchar *pBuf, uchar nBytes)
{
	uchar status;
	uchar i;
	
	SPI_FLASH_CS_LOW();             
	status = SPI_FLASH_SendByte(reg);   
	for(i = 0; i < nBytes;i++)
		SPI_FLASH_SendByte(pBuf[i]);
	SPI_FLASH_CS_HIGH();           
	return(status);     
}
