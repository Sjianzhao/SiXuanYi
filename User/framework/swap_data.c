#include "swap_data.h"
#include "data_exchang.h"
#include "AHRS.h"

//ATT函数计算出的姿态角
float ROL;//*100
float PIT;//*100
float YAW;//*100
vs32 Alt=10;
int32_t ALT_USE;
u8 ARMED; //: A0加锁 A1解锁

//采集的数据
int16_t ACC_X=1000;//加速度
int16_t ACC_Y;
int16_t ACC_Z;
int16_t GYRO_X=1000;//角度
int16_t GYRO_Y;
int16_t GYRO_Z;
int16_t MAG_X; //电子罗盘
int16_t MAG_Y;
int16_t MAG_Z;

u8 data_to_send[50];//发送缓存

extern short Acel[3];    //存入于[2][3] [4][5] [6][7]   角度
extern short Gyro[3];    //存入于[8][9] [10][11] [12][13]   加速度
extern short Mag[3];     //存入于[14][15] [16][17] [18][19]   地磁

extern double MAG;//存放计算好的地磁角度
T_float_angle Acel_filt;  //互补滤波后的角度

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
//		_temp = (int)(ROL*100);//俯滚角
		
		_temp = (int)(Acel_filt.roll*100);//俯滚角
	
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		
//		_temp = (int)(PIT*100); //横滚
	_temp =(int)(Acel_filt.pitch*100);//俯滚角
		
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		
//		YAW=-(MAG-180);
		_temp = (int)(YAW*100); //电子罗盘
		
		//_temp = (int)(Mag_Heading*100);
		data_to_send[_cnt++]=BYTE1(_temp);
		data_to_send[_cnt++]=BYTE0(_temp);
		
		
		data_to_send[_cnt++]=BYTE3(_temp2);
		data_to_send[_cnt++]=BYTE2(_temp2);
		data_to_send[_cnt++]=BYTE1(_temp2);
		data_to_send[_cnt++]=BYTE0(_temp2);
			
//		if(Rc_C.ARMED==0)			data_to_send[_cnt++]=0xA0;	//锁定
//		else if(Rc_C.ARMED==1)		data_to_send[_cnt++]=0xA1;

		data_to_send[_cnt++]=0xA1;
		data_to_send[3] = _cnt-4;


		for(i=0;i<_cnt;i++)
			sum += data_to_send[i];
			
		data_to_send[_cnt++]=sum;
		send_USART1(data_to_send ,_cnt);//发送数据

}

void sent_data_com(void)
{
		u8 _cnt=0;
		u8 sum = 0,i;
	
		ACC_X=Acel[0];//加速度
		ACC_Y=Acel[1];
		ACC_Z=Acel[2];
		GYRO_X=Gyro[0];//角度
		GYRO_Y=Gyro[1];
		GYRO_Z=Gyro[2];
		MAG_X=Mag[0]; //电子罗盘
		MAG_Y=Mag[1];
		MAG_Z=Mag[2];
	
		data_to_send[_cnt++]=0xaa;
		data_to_send[_cnt++]=0xaa;	
		data_to_send[_cnt++]=0x02;
		data_to_send[_cnt++]=18;//长度
		
		data_to_send[_cnt++]=ACC_X>>8; //地磁
		data_to_send[_cnt++]=ACC_X&0xff;
		
		data_to_send[_cnt++]=ACC_Y>>8;
		data_to_send[_cnt++]=ACC_Y&0xff;
			
		data_to_send[_cnt++]=ACC_Z>>8;
		data_to_send[_cnt++]=ACC_Z&0xff;
	
		data_to_send[_cnt++]=GYRO_X>>8;//角度
		data_to_send[_cnt++]=GYRO_X&0xff;
		
		data_to_send[_cnt++]=GYRO_Y>>8;
		data_to_send[_cnt++]=GYRO_Y&0xff;
			
		data_to_send[_cnt++]=GYRO_Z>>8;
		data_to_send[_cnt++]=GYRO_Z&0xff;
		
		data_to_send[_cnt++]=MAG_X>>8;//角度
		data_to_send[_cnt++]=MAG_X&0xff;
		
		data_to_send[_cnt++]=MAG_Y>>8;
		data_to_send[_cnt++]=MAG_Y&0xff;
			
		data_to_send[_cnt++]=MAG_Z>>8;
		data_to_send[_cnt++]=MAG_Z&0xff;
		

		for(i=0;i<_cnt;i++)
			sum += data_to_send[i];
		data_to_send[_cnt++] = sum;
		
		send_USART1(data_to_send ,_cnt);//发送数据
}

void send_line(int16_t b1,int16_t b2,int16_t b3,int16_t b4,int16_t b5,int16_t b6)//山外上位机线
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

extern char rx_num;//保存接收的次数;
extern uchar  RxBuf[32] ;	//接收缓存
extern uchar  TxBuf[32];	//发送缓存

//存放采集的数据
extern short Acel[3];    //存入于[2][3] [4][5] [6][7]      加速度
extern short Gyro[3];    //存入于[8][9] [10][11] [12][13]   角度
extern short Mag[3];     //存入于[14][15] [16][17] [18][19]   地磁
extern double TEMP_280;  //存入于[20][21]   温度
extern s32 Press;        //存入于[22][23] 气压
extern unsigned short int Distance;      //计算出的距离

extern double ADC_ConvertedValueLocal;//存入于[24][25] 电压
extern __IO uint16_t ADC_ConvertedValue;

extern T_float_xyz radian_gyro;   //修正后角度值
extern T_float_xyz averag_gyro; //卡尔曼滤波后的角度数据
extern T_float_xyz origin_Acel;//修正后的加速度值
extern T_float_xyz averag_Acel; //卡尔曼滤波后的加速度数据

extern int roll_PWM; 	//俯仰
extern int pitch_PWM;	//横滚
extern int yaw_PWM;		//偏航

/************接收的数据**************/
short Acelx,Acely,Acelz,Gyrox,Gyroy,Gyroz,Magx,Magy,Magz;
double RX_TEMP_280; //接收
uint16_t RX_ADC_ConvertedValue;
char RX_num,TX_num;
double RX_ADC_ConvertedValueLocal;
s32 RX_Press;
short accelerator_IN;  //油门
short roll_IN,pitch_IN;//俯仰，横滚
/***********************************/

void data_chang(void)
{
	  
		
		TxBuf[1] = rx_num & 0xff;      //接收次数	
//		Acel[0]=Acel[0]*0.0035;
	
		TxBuf[2] = ((int16_t)averag_Acel.X >> 8) & 0xff;
		TxBuf[3] = (int16_t)averag_Acel.X & 0xff;      	//加速度 X
		
		TxBuf[4] = ((int16_t)averag_Acel.Y >> 8) & 0xff;
		TxBuf[5] = (int16_t)averag_Acel.Y & 0xff;     	 	//加速度 Y
	
		TxBuf[6] = ((int16_t)averag_Acel.Z >> 8) & 0xff;
		TxBuf[7] = (int16_t)averag_Acel.Z & 0xff;     	 	//加速度 Z
	
		TxBuf[8] = ((int16_t)averag_gyro.X >> 8) & 0xff;
		TxBuf[9] = (int16_t)averag_gyro.X & 0xff;     	 	//陀螺仪 X
		
		TxBuf[10] = ((int16_t)averag_gyro.Y >> 8) & 0xff;
		TxBuf[11] = (int16_t)averag_gyro.Y & 0xff;    	  //陀螺仪 Y
	
		TxBuf[12] = ((int16_t)averag_gyro.Z >> 8) & 0xff;
		TxBuf[13] = (int16_t)averag_gyro.Z & 0xff;    	  //陀螺仪 Z
	
	  TxBuf[14] = ((int16_t)roll_PWM >> 8) & 0xff;
		TxBuf[15] = (int16_t)roll_PWM & 0xff;      				//俯仰PWM
		
		TxBuf[16] = ((int16_t)pitch_PWM >> 8) & 0xff;
		TxBuf[17] = (int16_t)pitch_PWM & 0xff;      			//横滚PWM
		
		TxBuf[18] = ((int16_t)yaw_PWM >> 8) & 0xff;
		TxBuf[19] = (int16_t)yaw_PWM & 0xff;      				//偏航PWM
		
//		TxBuf[20] = (yaw_PWM >> 8) & 0xff;
//		TxBuf[21] = yaw_PWM & 0xff;      									//偏航PWM
		
//		TxBuf[14] = (Mag[0] >> 8) & 0xff;
//		TxBuf[15] = Mag[0] & 0xff;      //地磁 X
//			
//		Mag[1]=Mag[1]-21;
//		
//		TxBuf[16] = (Mag[1] >> 8) & 0xff;
//		TxBuf[17] = Mag[1] & 0xff;      //地磁 Y
//		
//		
//		MAG_data();
//	
//		TxBuf[18] = (Mag[2] >> 8) & 0xff;
//		TxBuf[19] = Mag[2] & 0xff;      //地磁 Z
//		
//		TxBuf[20] = ((short)(TEMP_280*100.0) >> 8) & 0xff;
//		TxBuf[21] = (short)(TEMP_280*100.0) & 0xff;      //温度
//				
//		
//		TxBuf[22] = (ADC_ConvertedValue >> 8) & 0xff;
//		TxBuf[23] = ADC_ConvertedValue & 0xff;      //电压
//		
//		TxBuf[24] = (Press >> 24) & 0xff;
//		TxBuf[25] = (Press >> 16) & 0xff;      
//		TxBuf[26] = (Press >> 8) & 0xff;
//		TxBuf[27] = Press & 0xff;      		//气压
		
		
	
	//接收数据
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
		RX_ADC_ConvertedValueLocal=(float) RX_ADC_ConvertedValue/4096*15.6; // 读取转换的AD值
		
		
//		printf("%d\n",Acelx);
//		printf("\n");
		
		TxBuf[20] = ((int16_t)accelerator_IN >> 8) & 0xff;
		TxBuf[21] = (int16_t)accelerator_IN & 0xff;      				//偏航PWM
//		Data_Send_Status();//上位姿态
//		sent_data_com();//上位机传输数据
//		send_line(Acel_filt.pit,Acel_filt.rol,Acel_filt.yaw,10,100,1000);//山外		
//		  send_line(Acel_filt.pit*100,Acel_filt.rol*100,Acel_filt.yaw*100,Gyro[0]*0.5,Gyro[1]*0.5,Gyro[2]*0.5);//山外
//		printf("电压为= %lf V \t\n",RX_ADC_ConvertedValueLocal) ;
//		printf("加速度:%8d%8d%8d\t\n",Acelx,Acely,Acelz);
//		printf("陀螺仪:%8d%8d%8d\t\n",Gyrox,Gyroy,Gyroz);
//		printf("地磁场:%8d%8d%8d\t\n",Magx,Magy,Magz);
//		printf("陀螺温度%d\t",Temp);
//		printf("温度为 %lf\t\n",RX_TEMP_280);
//		printf("接收次数为 %d\t",RX_num);
//		printf("发送次数为 %d\t",rx_num);
//		printf("\t气压为 %d帕\n\n",RX_Press);
		
}
