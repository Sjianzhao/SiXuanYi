#include "stm32f10x.h"

#ifndef __NRF24L01_H
#define __NRF24L01_H
//#include "MCU.H"
//#include "SPI.H"
#define uchar unsigned char 
//========== NRF24L01 IO端口定义 ==========
//SPI相关引脚在SPI.H文件内定义
//sbit CE = P1^0;
//sbit IRQ = P3^2;
//============ 状态标志 =============
extern uchar  nrf_sta;
#define RX_DR 		((nrf_sta >> 6) & 0x01)
#define TX_DS  		((nrf_sta >> 5) & 0x01)
#define MAX_RT 	((nrf_sta >> 4) & 0x01)
//================== NRF24L01的接收和发送地址 ===================
#define TX_ADR_WIDTH    5   // 5个字节的TX地址长度
#define RX_ADR_WIDTH    5   // 5个字节的RX地址长度
#define TX_PLOAD_WIDTH  32  // ?个字节的TX数据长度
#define RX_PLOAD_WIDTH  32  // ?个字节的RX数据长度
//================== NRF24L01寄存器指令 =======================
#define READ_REGIST       	 	0x00  	// 读寄存器
#define WRITE_REGIST       		0x20 	// 写寄存器
#define RD_RX_PLOAD     	0x61  	// 读取接收数据
#define WR_TX_PLOAD     0xA0  	// 写待发数据
#define FLUSH_TX        		0xE1 	// 冲洗发送 FIFO
#define FLUSH_RX        		0xE2  	// 冲洗接收 FIFO
#define REUSE_TX_PL     	0xE3  	// 定义重复装载数据
#define NOP             			0xFF  	// 保留
//================== SPI(nRF24L01)寄存器地址 =====================
#define NRF_CONFIG      0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           		0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        	0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           		0x05  // 工作频率设置
#define RF_SETUP        	0x06  // 发射速率、功耗功能设置
#define STATUS          	0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              			0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         	0x10  // 发送地址寄存器
#define RX_PW_P0        	0x11  // 接收频道0接收数据长度
#define RX_PW_P1        	0x12  // 接收频道0接收数据长度
#define RX_PW_P2       	0x13  // 接收频道0接收数据长度
#define RX_PW_P3        	0x14  // 接收频道0接收数据长度
#define RX_PW_P4        	0x15  // 接收频道0接收数据长度
#define RX_PW_P5        	0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//==============================
//收发缓存数组 
//==============================
extern uchar  RxBuf[32] ;	//接收缓存 存入idata区
extern uchar  TxBuf[32];	//发送缓存
//=====================
//函数声明
//=====================
void init_NRF24L01(void);
void get_nrf_sta(void);
void nrf_RxMod(void);
void nrf_read(uchar *rx_buf);
void nrf_TxMod(void);
void nrf_trans(uchar *tx_buf);
void nrf_load_float(uchar a,float num);
float nrf_unload_float(uchar a);
void nrf_load_sfloat(uchar a,float num);
float nrf_unload_sfloat(uchar a);
/*********SPI**************/
//uchar SPI_RW(uchar num);
uchar SPI_Read_Reg(uchar reg);
uchar SPI_Write_Reg(uchar reg, uchar value);
uchar SPI_Read_Buf(uchar reg, uchar *pBuf, uchar nBytes);
uchar SPI_Write_Buf(uchar reg, uchar *pBuf, uchar nBytes);


#endif

