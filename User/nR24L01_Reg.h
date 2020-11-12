#include "stm32f10x.h"

#ifndef __NRF24L01_H
#define __NRF24L01_H
//#include "MCU.H"
//#include "SPI.H"
#define uchar unsigned char 
//========== NRF24L01 IO�˿ڶ��� ==========
//SPI���������SPI.H�ļ��ڶ���
//sbit CE = P1^0;
//sbit IRQ = P3^2;
//============ ״̬��־ =============
extern uchar  nrf_sta;
#define RX_DR 		((nrf_sta >> 6) & 0x01)
#define TX_DS  		((nrf_sta >> 5) & 0x01)
#define MAX_RT 	((nrf_sta >> 4) & 0x01)
//================== NRF24L01�Ľ��պͷ��͵�ַ ===================
#define TX_ADR_WIDTH    5   // 5���ֽڵ�TX��ַ����
#define RX_ADR_WIDTH    5   // 5���ֽڵ�RX��ַ����
#define TX_PLOAD_WIDTH  32  // ?���ֽڵ�TX���ݳ���
#define RX_PLOAD_WIDTH  32  // ?���ֽڵ�RX���ݳ���
//================== NRF24L01�Ĵ���ָ�� =======================
#define READ_REGIST       	 	0x00  	// ���Ĵ���
#define WRITE_REGIST       		0x20 	// д�Ĵ���
#define RD_RX_PLOAD     	0x61  	// ��ȡ��������
#define WR_TX_PLOAD     0xA0  	// д��������
#define FLUSH_TX        		0xE1 	// ��ϴ���� FIFO
#define FLUSH_RX        		0xE2  	// ��ϴ���� FIFO
#define REUSE_TX_PL     	0xE3  	// �����ظ�װ������
#define NOP             			0xFF  	// ����
//================== SPI(nRF24L01)�Ĵ�����ַ =====================
#define NRF_CONFIG      0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           		0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        	0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           		0x05  // ����Ƶ������
#define RF_SETUP        	0x06  // �������ʡ����Ĺ�������
#define STATUS          	0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              			0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         	0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        	0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        	0x12  // ����Ƶ��0�������ݳ���
#define RX_PW_P2       	0x13  // ����Ƶ��0�������ݳ���
#define RX_PW_P3        	0x14  // ����Ƶ��0�������ݳ���
#define RX_PW_P4        	0x15  // ����Ƶ��0�������ݳ���
#define RX_PW_P5        	0x16  // ����Ƶ��0�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������
//==============================
//�շ��������� 
//==============================
extern uchar  RxBuf[32] ;	//���ջ��� ����idata��
extern uchar  TxBuf[32];	//���ͻ���
//=====================
//��������
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

