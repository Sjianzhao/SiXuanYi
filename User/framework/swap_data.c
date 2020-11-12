#include "swap_data.h"
#include "data_exchang.h"
#include "AHRS.h"

//ATT�������������̬��
float ROL;//*100
float PIT;//*100
float YAW;//*100
vs32 Alt=10;
int32_t ALT_USE;
u8 ARMED; //: A0���� A1����

//�ɼ�������
int16_t ACC_X=1000;//���ٶ�
int16_t ACC_Y;
int16_t ACC_Z;
int16_t GYRO_X=1000;//�Ƕ�
int16_t GYRO_Y;
int16_t GYRO_Z;
int16_t MAG_X; //��������
int16_t MAG_Y;
int16_t MAG_Z;

u8 data_to_send[50];//���ͻ���

extern short Acel[3];    //������[2][3] [4][5] [6][7]   �Ƕ�
extern short Gyro[3];    //������[8][9] [10][11] [12][13]   ���ٶ�
extern short Mag[3];     //������[14][15] [16][17] [18][19]   �ش�

extern double MAG;//��ż���õĵشŽǶ�
T_float_angle Acel_filt;  //�����˲���ĽǶ�

void Data_Send_Status(void)
{
		u8 _cnt=0;
	  vs16 _temp;
	  vs32 _temp2 = Alt;
		u8 sum = 0,i;
	
		data_to_send[_cnt++]=0xAA;
		data_to_send[_cnt++]=0xAA;
		data_to_send[_cnt++]=0x01;
		data_to_send[_cnt++]=0;
		
	
//		ROL=(float)Gyro[0]*0.001;
//		_temp = (int)(ROL*100);//������
		
		_temp = (int)(Acel_filt.roll*100);//������
	
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		
//		_temp = (int)(PIT*100); //���
	_temp =(int)(Acel_filt.pitch*100);//������
		
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		
//		YAW=-(MAG-180);
		_temp = (int)(YAW*100); //��������
		
		//_temp = (int)(Mag_Heading*100);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		
		
		data_to_send[_cnt++]=BYTE3(_temp2);
		data_to_send[_cnt++]=BYTE2(_temp2);
		data_to_send[_cnt++]=BYTE1(_temp2);
		data_to_send[_cnt++]=BYTE0(_temp2);
			
//		if(Rc_C.ARMED==0)			data_to_send[_cnt++]=0xA0;	//����
//		else if(Rc_C.ARMED==1)		data_to_send[_cnt++]=0xA1;

		data_to_send[_cnt++]=0xA1;
		data_to_send[3] = _cnt-4;


		for(i=0;i<_cnt;i++)
			sum += data_to_send[i];
			
		data_to_send[_cnt++]=sum;
		send_USART1(data_to_send ,_cnt);//��������

}

void sent_data_com(void)
{
		u8 _cnt=0;
		u8 sum = 0,i;
	
		ACC_X=Acel[0];//���ٶ�
		ACC_Y=Acel[1];
		ACC_Z=Acel[2];
		GYRO_X=Gyro[0];//�Ƕ�
		GYRO_Y=Gyro[1];
		GYRO_Z=Gyro[2];
		MAG_X=Mag[0]; //��������
		MAG_Y=Mag[1];
		MAG_Z=Mag[2];
	
		data_to_send[_cnt++]=0xaa;
		data_to_send[_cnt++]=0xaa;	
		data_to_send[_cnt++]=0x02;
		data_to_send[_cnt++]=18;//����
		
		data_to_send[_cnt++]=ACC_X>>8; //�ش�
		data_to_send[_cnt++]=ACC_X&0xff;
		
		data_to_send[_cnt++]=ACC_Y>>8;
		data_to_send[_cnt++]=ACC_Y&0xff;
			
		data_to_send[_cnt++]=ACC_Z>>8;
		data_to_send[_cnt++]=ACC_Z&0xff;
	
		data_to_send[_cnt++]=GYRO_X>>8;//�Ƕ�
		data_to_send[_cnt++]=GYRO_X&0xff;
		
		data_to_send[_cnt++]=GYRO_Y>>8;
		data_to_send[_cnt++]=GYRO_Y&0xff;
			
		data_to_send[_cnt++]=GYRO_Z>>8;
		data_to_send[_cnt++]=GYRO_Z&0xff;
		
		data_to_send[_cnt++]=MAG_X>>8;//�Ƕ�
		data_to_send[_cnt++]=MAG_X&0xff;
		
		data_to_send[_cnt++]=MAG_Y>>8;
		data_to_send[_cnt++]=MAG_Y&0xff;
			
		data_to_send[_cnt++]=MAG_Z>>8;
		data_to_send[_cnt++]=MAG_Z&0xff;
		

		for(i=0;i<_cnt;i++)
			sum += data_to_send[i];
		data_to_send[_cnt++] = sum;
		
		send_USART1(data_to_send ,_cnt);//��������
}

void send_line(int16_t b1,int16_t b2,int16_t b3,int16_t b4,int16_t b5,int16_t b6)//ɽ����λ����
{
	send_byte_USART1(0x03);
	send_byte_USART1(0xfc);
	
	send_byte_USART1(b1&0xff);
  send_byte_USART1((b1>>8));
	
	send_byte_USART1(b2&0xff);
  send_byte_USART1((b2>>8));
	
	send_byte_USART1(b3&0xff);
  send_byte_USART1((b3>>8));
	
	send_byte_USART1(b4&0xff);
  send_byte_USART1((b4>>8));
	
  send_byte_USART1(b5&0xff);
  send_byte_USART1((b5>>8));
	
  send_byte_USART1(b6&0xff);
  send_byte_USART1((b6>>8));
	
  send_byte_USART1(0xfc);
	send_byte_USART1(0x03);
	
}

extern char rx_num;//������յĴ���;
extern uchar  RxBuf[32] ;	//���ջ���
extern uchar  TxBuf[32];	//���ͻ���

//��Ųɼ�������
extern short Acel[3];    //������[2][3] [4][5] [6][7]      ���ٶ�
extern short Gyro[3];    //������[8][9] [10][11] [12][13]   �Ƕ�
extern short Mag[3];     //������[14][15] [16][17] [18][19]   �ش�
extern double TEMP_280;  //������[20][21]   �¶�
extern s32 Press;        //������[22][23] ��ѹ
extern unsigned short int Distance;      //������ľ���

extern double ADC_ConvertedValueLocal;//������[24][25] ��ѹ
extern __IO uint16_t ADC_ConvertedValue;

extern T_float_xyz radian_gyro;   //������Ƕ�ֵ
extern T_float_xyz averag_gyro; //�������˲���ĽǶ�����
extern T_float_xyz origin_Acel;//������ļ��ٶ�ֵ
extern T_float_xyz averag_Acel; //�������˲���ļ��ٶ�����

extern int roll_PWM; 	//����
extern int pitch_PWM;	//���
extern int yaw_PWM;		//ƫ��

/************���յ�����**************/
short Acelx,Acely,Acelz,Gyrox,Gyroy,Gyroz,Magx,Magy,Magz;
double RX_TEMP_280; //����
uint16_t RX_ADC_ConvertedValue;
char RX_num,TX_num;
double RX_ADC_ConvertedValueLocal;
s32 RX_Press;
short accelerator_IN;  //����
short roll_IN,pitch_IN;//���������
/***********************************/

void data_chang(void)
{
	  
		
		TxBuf[1] = rx_num & 0xff;      //���մ���	
//		Acel[0]=Acel[0]*0.0035;
	
		TxBuf[2] = ((int16_t)averag_Acel.X >> 8) & 0xff;
		TxBuf[3] = (int16_t)averag_Acel.X & 0xff;      	//���ٶ� X
		
		TxBuf[4] = ((int16_t)averag_Acel.Y >> 8) & 0xff;
		TxBuf[5] = (int16_t)averag_Acel.Y & 0xff;     	 	//���ٶ� Y
	
		TxBuf[6] = ((int16_t)averag_Acel.Z >> 8) & 0xff;
		TxBuf[7] = (int16_t)averag_Acel.Z & 0xff;     	 	//���ٶ� Z
	
		TxBuf[8] = ((int16_t)averag_gyro.X >> 8) & 0xff;
		TxBuf[9] = (int16_t)averag_gyro.X & 0xff;     	 	//������ X
		
		TxBuf[10] = ((int16_t)averag_gyro.Y >> 8) & 0xff;
		TxBuf[11] = (int16_t)averag_gyro.Y & 0xff;    	  //������ Y
	
		TxBuf[12] = ((int16_t)averag_gyro.Z >> 8) & 0xff;
		TxBuf[13] = (int16_t)averag_gyro.Z & 0xff;    	  //������ Z
	
	  TxBuf[14] = ((int16_t)roll_PWM >> 8) & 0xff;
		TxBuf[15] = (int16_t)roll_PWM & 0xff;      				//����PWM
		
		TxBuf[16] = ((int16_t)pitch_PWM >> 8) & 0xff;
		TxBuf[17] = (int16_t)pitch_PWM & 0xff;      			//���PWM
		
		TxBuf[18] = ((int16_t)yaw_PWM >> 8) & 0xff;
		TxBuf[19] = (int16_t)yaw_PWM & 0xff;      				//ƫ��PWM
		
//		TxBuf[20] = (yaw_PWM >> 8) & 0xff;
//		TxBuf[21] = yaw_PWM & 0xff;      									//ƫ��PWM
		
//		TxBuf[14] = (Mag[0] >> 8) & 0xff;
//		TxBuf[15] = Mag[0] & 0xff;      //�ش� X
//			
//		Mag[1]=Mag[1]-21;
//		
//		TxBuf[16] = (Mag[1] >> 8) & 0xff;
//		TxBuf[17] = Mag[1] & 0xff;      //�ش� Y
//		
//		
//		MAG_data();
//	
//		TxBuf[18] = (Mag[2] >> 8) & 0xff;
//		TxBuf[19] = Mag[2] & 0xff;      //�ش� Z
//		
//		TxBuf[20] = ((short)(TEMP_280*100.0) >> 8) & 0xff;
//		TxBuf[21] = (short)(TEMP_280*100.0) & 0xff;      //�¶�
//				
//		
//		TxBuf[22] = (ADC_ConvertedValue >> 8) & 0xff;
//		TxBuf[23] = ADC_ConvertedValue & 0xff;      //��ѹ
//		
//		TxBuf[24] = (Press >> 24) & 0xff;
//		TxBuf[25] = (Press >> 16) & 0xff;      
//		TxBuf[26] = (Press >> 8) & 0xff;
//		TxBuf[27] = Press & 0xff;      		//��ѹ
		
		
	
	//��������
		RX_num=RxBuf[1];
		accelerator_IN=(RxBuf[2] << 8) | RxBuf[3];
		roll_IN=(RxBuf[4] << 8) | RxBuf[5];
		pitch_IN=(RxBuf[6] << 8) | RxBuf[7];
		Gyrox=(RxBuf[8] << 8) | RxBuf[9];
		Gyroy=(RxBuf[10] << 8) | RxBuf[11];
		Gyroz=(RxBuf[12] << 8) | RxBuf[13];
		Magx=(RxBuf[14] << 8) | RxBuf[15];
		Magy=(RxBuf[16] << 8) | RxBuf[17];
		Magz=(RxBuf[18] << 8) | RxBuf[19];
		RX_TEMP_280 = ((RxBuf[20] << 8) | RxBuf[21])/100.0;
		RX_ADC_ConvertedValue=(RxBuf[22] << 8) | RxBuf[23];		
		RX_Press=(RxBuf[24] << 24) |(RxBuf[25] << 16) |(RxBuf[26] << 8) | RxBuf[27];
		RX_ADC_ConvertedValueLocal=(float) RX_ADC_ConvertedValue/4096*15.6; // ��ȡת����ADֵ
		
		
//		printf("%d\n",Acelx);
//		printf("\n");
		
		TxBuf[20] = ((int16_t)accelerator_IN >> 8) & 0xff;
		TxBuf[21] = (int16_t)accelerator_IN & 0xff;      				//ƫ��PWM
//		Data_Send_Status();//��λ��̬
//		sent_data_com();//��λ����������
//		send_line(Acel_filt.pit,Acel_filt.rol,Acel_filt.yaw,10,100,1000);//ɽ��		
//		  send_line(Acel_filt.pit*100,Acel_filt.rol*100,Acel_filt.yaw*100,Gyro[0]*0.5,Gyro[1]*0.5,Gyro[2]*0.5);//ɽ��
//		printf("��ѹΪ= %lf V \t\n",RX_ADC_ConvertedValueLocal) ;
//		printf("���ٶ�:%8d%8d%8d\t\n",Acelx,Acely,Acelz);
//		printf("������:%8d%8d%8d\t\n",Gyrox,Gyroy,Gyroz);
//		printf("�شų�:%8d%8d%8d\t\n",Magx,Magy,Magz);
//		printf("�����¶�%d\t",Temp);
//		printf("�¶�Ϊ %lf\t\n",RX_TEMP_280);
//		printf("���մ���Ϊ %d\t",RX_num);
//		printf("���ʹ���Ϊ %d\t",rx_num);
//		printf("\t��ѹΪ %d��\n\n",RX_Press);
		
}
