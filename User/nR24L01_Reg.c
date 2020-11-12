#include "nR24L01_Reg.h"
#include "GPIO_Config.h"
#include "bsp_SysTick.h"

//#include "NRF24L01.H"

#define uint unsigned int

uchar  nrf_sta;
uchar  RxBuf[32] = 	"0";	//���ջ��� ����idata��
uchar  TxBuf[32] = 	"0";	//���ͻ���
	
uchar  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x32,0x24,0x01};	//���ص�ַ
uchar  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x32,0x24,0x01};	//���յ�ַ

//===== ���Ե���ʱ =====
//void delayus(uint us)
//{
//	while(us--);
//}
//================== NRF24L01��ʼ�� ==================
void init_NRF24L01(void)
{
  Delay_us(100);
 	SPI_FLASH_CE_LOW();    // Ƭѡʹ��
	SPI_FLASH_CS_HIGH();
// 	SPI_FLASH_CE_LOW();   // SPIʱ������
	SPI_FLASH_CLK_LOW();
	SPI_Write_Buf(WRITE_REGIST + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    		//д���͵�ַ	
	SPI_Write_Buf(WRITE_REGIST + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); 	//д���ն˵�ַ
	SPI_Write_Reg(WRITE_REGIST + EN_AA, 0x01);                             			//ͨ��0�Զ�Ӧ�� 	
	SPI_Write_Reg(WRITE_REGIST + EN_RXADDR, 0x01);                         	//������յ�ַƵ��0 
	SPI_Write_Reg(WRITE_REGIST + RF_CH, 0x35);                             			//�����ŵ�����Ƶ�ʣ��շ�����һ��
	SPI_Write_Reg(WRITE_REGIST + RX_PW_P0, RX_PLOAD_WIDTH);                //���ý������ݳ���
	SPI_Write_Reg(WRITE_REGIST + RF_SETUP, 0x08);   /*0x0F 2M 0x08 1M*/		   //���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB	
	SPI_Write_Reg(WRITE_REGIST + NRF_CONFIG, 0x7c); 						//IRQ���Ų���ʾ�ж� ����ģʽ  1~16CRCУ��
}
//==================
//��ȡ״̬��־
//==================
void get_nrf_sta(void)
{
	nrf_sta = SPI_Read_Reg(STATUS);
	SPI_Write_Reg(WRITE_REGIST+STATUS,0XFF);    
}
//==================
//����Ϊ����ģʽ
//==================
void nrf_RxMod(void)
{
	SPI_FLASH_CE_LOW();
	SPI_Write_Reg(WRITE_REGIST+STATUS,0xff);	//����жϱ�־
	SPI_Write_Reg(FLUSH_RX,0x00); 			//���RX_FIFO�Ĵ���
	SPI_Write_Reg(WRITE_REGIST + NRF_CONFIG, 0x7f);//IRQ���Ų���ʾ�ж� �ϵ� ����ģʽ   1~16CRCУ��  
//Delay_us(100);	
	SPI_FLASH_CE_HIGH();
//	Delay_us(100);
}
//==================
//�ѽ��յ������ݴ�������
//==================
void nrf_read(uchar *rx_buf)
{
		if(RX_DR == 1)    		//�յ�����
		{
			SPI_FLASH_CE_LOW();  
			SPI_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);//��ȡ���� ��������
			SPI_Write_Reg(FLUSH_RX,0x00);//���rx fifo�Ĵ���
//			Delay_us(100);
			SPI_FLASH_CE_HIGH();
//      Delay_us(100);
		} 	 	
}
//==================
//����Ϊ����ģʽ
//==================
void nrf_TxMod(void)
{
	SPI_FLASH_CE_LOW();
//Delay_us(100);
	
	SPI_Write_Reg(WRITE_REGIST+STATUS,0xff); 			//����жϱ�־  	
	
//	Delay_us(100);
	SPI_Write_Reg(FLUSH_TX,0x00);								//���TX_FIFO�Ĵ��� 
//	Delay_us(100);
	SPI_Write_Reg(WRITE_REGIST + NRF_CONFIG,0x7e);		//IRQ���Ų���ʾ�ж� �ϵ� ����ģʽ  1~16CRCУ��
//	Delay_us(100);
	SPI_FLASH_CE_HIGH();            
//  Delay_us(100); 
}
//==================
//����  �����κ��ж�ֻ�ܷ���
//==================
void nrf_trans(uchar *tx_buf)
{	
	SPI_FLASH_CE_LOW();			//StandBy Iģʽ
//	Delay_us(100);
	SPI_Write_Reg(WRITE_REGIST+STATUS,0xFF);		//��������ж�
//	Delay_us(100);
	SPI_Write_Reg(FLUSH_TX,0x00); 							//���tx fifo�Ĵ���	//===== ��Ҫ =====
//	Delay_us(100);
	SPI_Write_Buf(WRITE_REGIST + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 		// װ�ؽ��ն˵�ַ
//	Delay_us(100);
	SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH); 			 						// װ������
//	Delay_us(100);
	SPI_FLASH_CE_HIGH();		 			//�ø�CE�������ݷ���
//  Delay_us(100);printf("ABCD\n");		//����ʱ������ ��Ϊ�Ӵ���ģʽ���շ�ģʽ��Ҫʱ�䣬�����Ҫ130us
}
//=========================
//��float������װ�� ����4λС��
//ռ��5���ֽ� ���ݷ�Χ+- 65535.9999
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
//�����յ���float�������
//ռ��5���ֽ� ���ݷ�Χ+- 65535.9999
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
//��float������װ�� ����2λС��
//ռ��3���ֽ� ���ݷ�Χ+- 255.99
//=======================
void nrf_load_sfloat(uchar a,float num)
{
	if(num > 0){
		TxBuf[a] = '+';	
		TxBuf[a+1] = (uchar)num;		//ת����uchar���ͣ��Զ���������8λ��ȥ����λ��
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
//��float������ ����2λС��
//ռ��3���ֽ� ���ݷ�Χ+- 255.99
//======================
float nrf_unload_sfloat(uchar a)	//a�����ݰ��������ڵ���ʼλ��
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
/***********************SPIͨѶ*************************************/


////=============================================
////SPI��д��������MOSI����д��һ���ֽڣ�ͬʱ��MISO���Ŷ�ȡһ���ֽ�
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
//SPI���Ĵ���������ֻ��һ�����ܣ���ȡreg��ֵ������
//��д��Ĵ�����ַ��ʹSPI������λ���õ�ַ��
//��һ�ζ�ȡȡ���üĴ�����ֵ�������ظ�ֵ��
//=============================================
uchar SPI_Read_Reg(uchar reg)
{
	uchar reg_val;
	
	SPI_FLASH_CS_LOW();                
	SPI_FLASH_SendByte(reg);            
	reg_val = SPI_FLASH_SendByte(0);//�˴���0û��ʲô���壬������1,2,3������  
	SPI_FLASH_CS_HIGH();                
	
	return (reg_val);        
}
//=============================================
//SPIд�Ĵ�����������Ҫ����:��reg��д��value
//���ӹ���:���ؼĴ�����״̬��
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
//��SPI�豸�����������ݶ��뵥Ƭ���������浽������
//������ȡnBytes���ֽ�
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
//����Ƭ����nBytes����Ա������д��SPI�豸��reg�Ĵ���
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
