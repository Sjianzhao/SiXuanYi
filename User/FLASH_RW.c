#include "FLASH_RW.h"

/*0x0800 0000 ~ 0x0801 0000 *///64k���� 2kһҳ
/*0x0801 0000 ~ 0x0808 0000 *///437keeprom 2kһҳ 0x800һҳ

//0x0801 0000~0x0801 07FF		  2Kbyte
//0x0801 0800~0x0801 0FFF 		2Kbyte
//0x0801 1000~0x0801 17FF 		2Kbyte
//0x0801 1800~0x0801 1FFF 		2Kbyte

//2K���Դ�512�� int������

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE; //Flash����״̬����

uint8_t flash_data[FLASH_DATA_LONG][4];//���ݻ���

int FLASH_MAG_X_DATA;
int FLASH_MAG_Y_DATA;          //�ش�ԭ��У׼
int FLASH_MAG_Z_DATA;

int FLASH_Gyro_X_DATA;
int FLASH_Gyro_Y_DATA;         //�Ƕ�ԭ��У׼
int FLASH_Gyro_Z_DATA;

int FLASH_Acel_X_DATA;
int FLASH_Acel_Y_DATA;         //���ٶ�ԭ��У׼
int FLASH_Acel_Z_DATA;

int FLASH_INPID_P_DATA;
int FLASH_INPID_I_DATA;       //�ڻ�PID
int FLASH_INPID_D_DATA;

int FLASH_OUTPID_P_DATA;
int FLASH_OUTPID_I_DATA;     //�⻷PID
int FLASH_OUTPID_D_DATA;

int FLASH_YAWINPID_P_DATA;
int FLASH_YAWINPID_I_DATA;       //YAW�ڻ�PID
int FLASH_YAWINPID_D_DATA;

int FLASH_YAWOUTPID_P_DATA;
int FLASH_YAWOUTPID_I_DATA;     //YAW�⻷PID
int FLASH_YAWOUTPID_D_DATA;

/**********************��������****************************

д�����ڵ����ݵ�flash��			     write_flash_data();			
��ȡflash�д�ŵ����ݵ�������			read_flash_data();
                                                                
		ReadNum = ReadFlashNBtye(0, Temp_Data,4); //��ȡ����

		WriteFlashOneWord(0,0x12345678); //д������
*************************************************************
*Name: ReadFlashNBtye 

*Function: ���ڲ�Flash��ȡN�ֽ����� 

*Input: ReadAddress�����ݵ�ַ��ƫ�Ƶ�ַ��ReadBuf������ָ�� ReadNum����ȡ�ֽ��� 

*Output: ��ȡ���ֽ��� 

*Author: ValerianFan 
****************************************************************/ 

int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum) 

{ 

		int DataNum = 0; 

		ReadAddress = (uint32_t)STARTADDR + ReadAddress*4; 

		while(DataNum < ReadNum) 

		{ 

				*(ReadBuf + DataNum) = *(__IO uint8_t*) ReadAddress++; 

				DataNum++; 

		} 

		return DataNum; 

} 

				  
//	  //д����
//		FLASH_Unlock();  //����FLASH��̲���������
//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//�����־λ
//     /*********************************************************************************
//           //               FLASH_FLAG_BSY            FLASHæ��־λ
//          //               FLASH_FLAG_EOP            FLASH����������־λ
//          //               FLASH_FLAG_PGERR            FLASH��д�����־λ
//          //               FLASH_FLAG_WRPRTERR       FLASHҳ��д��������꾻         
//      **********************************************************************************/
//      FLASH_ErasePage(FLASH_START_ADDR);     //����ָ����ַҳ
//     FLASH_ProgramHalfWord(FLASH_START_ADDR+(addr+i)*2,dat); //��ָ��ҳ��addr��ַ��ʼд
//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//�����־λ
//     FLASH_Lock();    //����FLASH��̲���������

//	//������
//	u16 value;
//     value = *(u16*)(FLASH_START_ADDR+(addr*2));//��ָ��ҳ��addr��ַ��ʼ��

/**************************************************************** 

*Name: WriteFlashOneWord 

*Function: ���ڲ�Flashд��32λ���� 

*Input: WriteAddress�����ݵ�ַ��ƫ�Ƶ�ַ��WriteData��д������ 

*Output: NULL 

*Author: ValerianFan 

****************************************************************/ 



void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData) 

{ 

		FLASH_UnlockBank1();//����flash 

//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//�����־λ		
//	   FLASH_ErasePage(STARTADDR + WriteAddress);     //����ָ����ַҳ
//     FLASH_ProgramHalfWord(STARTADDR + WriteAddress+2,WriteData); //��ָ��ҳ��addr��ַ��ʼд
//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//�����־λ
	



		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);//�����־λ 
	
		FLASHStatus = FLASH_ErasePage(STARTADDR);  //����ָ����ַҳ
//	   FLASH_ErasePage(STARTADDR + WriteAddress);     //����ָ����ַҳ

		if(FLASHStatus == FLASH_COMPLETE)   //��ָ��ҳ��addr��ַ��ʼд

		{ 

				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_MAG_X_ADD*4, FLASH_MAG_X_DATA); //flash.c ��API���� 
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_MAG_Y_ADD*4, FLASH_MAG_Y_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_MAG_Z_ADD*4, FLASH_MAG_Z_DATA);
			
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_Gyro_X_ADD*4, FLASH_Gyro_X_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_Gyro_Y_ADD*4, FLASH_Gyro_Y_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_Gyro_Z_ADD*4, FLASH_Gyro_Z_DATA);
			
			  FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_Acel_X_ADD*4, FLASH_Acel_X_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_Acel_Y_ADD*4, FLASH_Acel_Y_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_Acel_Z_ADD*4, FLASH_Acel_Z_DATA);
			
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_INPID_P_ADD*4, FLASH_INPID_P_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_INPID_I_ADD*4, FLASH_INPID_I_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_INPID_D_ADD*4, FLASH_INPID_D_DATA);
			
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_OUTPID_P_ADD*4, FLASH_OUTPID_P_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_OUTPID_I_ADD*4, FLASH_OUTPID_I_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_OUTPID_D_ADD*4, FLASH_OUTPID_D_DATA);
			
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_YAWINPID_P_ADD*4, FLASH_YAWINPID_P_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_YAWINPID_I_ADD*4, FLASH_YAWINPID_I_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_YAWINPID_D_ADD*4, FLASH_YAWINPID_D_DATA);
			
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_YAWOUTPID_P_ADD*4, FLASH_YAWOUTPID_P_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_YAWOUTPID_I_ADD*4, FLASH_YAWOUTPID_I_DATA);
				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_YAWOUTPID_D_ADD*4, FLASH_YAWOUTPID_D_DATA);
			
				//FLASHStatus = FLASH_ProgramWord(StartAddress+4, 0x56780000); //��Ҫд���������ʱ���� 

				//FLASHStatus = FLASH_ProgramWord(StartAddress+8, 0x87650000); //��Ҫд���������ʱ���� 

		} 

		FLASH_LockBank1(); //���� flash

} 

/**************************/
/*��ȡflash�д�ŵ�����*/
/**************************/

void read_flash_data(void)
{
//		uint8_t flash_data[FLASH_DATA_LONG][4];
		ReadFlashNBtye(0, flash_data[0], FLASH_DATA_LONG*4);//��ȡ���ݣ���ȡͷ��ַ�������ַ����ȡ���ȣ�
	
		FLASH_MAG_X_DATA=data_turn(FLASH_MAG_X_ADD);//������������
		FLASH_MAG_Y_DATA=data_turn(FLASH_MAG_Y_ADD);          //�ش�ԭ��У׼
		FLASH_MAG_Z_DATA=data_turn(FLASH_MAG_Z_ADD);

		FLASH_Gyro_X_DATA=data_turn(FLASH_Gyro_X_ADD);
		FLASH_Gyro_Y_DATA=data_turn(FLASH_Gyro_Y_ADD);         //�Ƕ�ԭ��У׼
		FLASH_Gyro_Z_DATA=data_turn(FLASH_Gyro_Z_ADD);

		FLASH_Acel_X_DATA=data_turn(FLASH_Acel_X_ADD);
		FLASH_Acel_Y_DATA=data_turn(FLASH_Acel_Y_ADD);         //���ٶ�ԭ��У׼
		FLASH_Acel_Z_DATA=data_turn(FLASH_Acel_Z_ADD);

		FLASH_INPID_P_DATA=data_turn(FLASH_INPID_P_ADD);
		FLASH_INPID_I_DATA=data_turn(FLASH_INPID_I_ADD);       //�ڻ�PID
		FLASH_INPID_D_DATA=data_turn(FLASH_INPID_D_ADD);

		FLASH_OUTPID_P_DATA=data_turn(FLASH_OUTPID_P_ADD);
		FLASH_OUTPID_I_DATA=data_turn(FLASH_OUTPID_I_ADD);     //�⻷PID
		FLASH_OUTPID_D_DATA=data_turn(FLASH_OUTPID_D_ADD);
		
		FLASH_YAWINPID_P_DATA=data_turn(FLASH_YAWINPID_P_ADD);
		FLASH_YAWINPID_I_DATA=data_turn(FLASH_YAWINPID_I_ADD);       //ƫ���ڻ�PID
		FLASH_YAWINPID_D_DATA=data_turn(FLASH_YAWINPID_D_ADD);

		FLASH_YAWOUTPID_P_DATA=data_turn(FLASH_YAWOUTPID_P_ADD);
		FLASH_YAWOUTPID_I_DATA=data_turn(FLASH_YAWOUTPID_I_ADD);     //ƫ���⻷PID
		FLASH_YAWOUTPID_D_DATA=data_turn(FLASH_YAWOUTPID_D_ADD);

}

/**************************/
/*д��flash�д�ŵ�����*/
/**************************/

void write_flash_data(void)
{
//		uint8_t flash_data[FLASH_DATA_LONG][4];
		WriteFlashOneWord(0,0); //д������ (��ʼ��ַ��32λ������)
		
}

/**************************/
/*�����е���������*/
/*�����ȡ�����ݣ�����int������*/
/**************************/

int data_turn(uint16_t FLASH_DATA)
{
	  int data;

		data=(flash_data[FLASH_DATA][3]<<24)|(flash_data[FLASH_DATA][2]<<16)|(flash_data[FLASH_DATA][1]<<8)|flash_data[FLASH_DATA][0];//��ȡ���ݵȴ�ʹ��

    return data;
}


void Flash_data_init(void)
{

//		extern T_float_xyz Mag_corr;//У׼��
		extern short Xoffset,Yoffset,Zoffset;  //����
		extern short offset_GX,offset_GY,offset_GZ;  //����
		extern short offset_AX,offset_AY,offset_AZ;  //����
	
		extern int external_KP; 			//�⻷
		extern int external_KI;
		extern int external_KD;

		extern int interior_KP; 			//�ڲ�
		extern int interior_KI;
		extern int interior_KD;
	
		extern int YAWexternal_KP; 			//ƫ���⻷
		extern int YAWexternal_KI;
		extern int YAWexternal_KD;

		extern int YAWinterior_KP; 			//ƫ���ڲ�
		extern int YAWinterior_KI;
		extern int YAWinterior_KD;
	
		extern int FLASH_MAG_X_DATA;
		extern int FLASH_MAG_Y_DATA;          //�ش�ԭ��У׼
		extern int FLASH_MAG_Z_DATA;
			
		extern int FLASH_Gyro_X_DATA;
		extern int FLASH_Gyro_Y_DATA;         //�Ƕ�ԭ��У׼
		extern int FLASH_Gyro_Z_DATA;

		extern int FLASH_Acel_X_DATA;
		extern int FLASH_Acel_Y_DATA;         //���ٶ�ԭ��У׼
		extern int FLASH_Acel_Z_DATA;
	
		extern int FLASH_INPID_P_DATA;
		extern int FLASH_INPID_I_DATA;       //�ڻ�PID
		extern int FLASH_INPID_D_DATA;

		extern int FLASH_OUTPID_P_DATA;
		extern int FLASH_OUTPID_I_DATA;     //�⻷PID
		extern int FLASH_OUTPID_D_DATA;
		
		extern float external_KP_f; 			//�⻷
		extern float external_KI_f;
		extern float external_KD_f;

		extern float interior_KP_f; 			//�ڲ�
		extern float interior_KI_f;
		extern float interior_KD_f;
		
		extern int FLASH_YAWINPID_P_DATA;
		extern int FLASH_YAWINPID_I_DATA;       //ƫ���ڻ�PID
		extern int FLASH_YAWINPID_D_DATA;

		extern int FLASH_YAWOUTPID_P_DATA;
		extern int FLASH_YAWOUTPID_I_DATA;     //ƫ���⻷PID
		extern int FLASH_YAWOUTPID_D_DATA;
		
		extern float YAWexternal_KP_f; 			//ƫ���⻷
		extern float YAWexternal_KI_f;
		extern float YAWexternal_KD_f;

		extern float YAWinterior_KP_f; 			//ƫ���ڲ�
		extern float YAWinterior_KI_f;
		extern float YAWinterior_KD_f;
			
		read_flash_data();//��ȡ��ŵ�flash����
		Xoffset=FLASH_MAG_X_DATA;
		Yoffset=FLASH_MAG_Y_DATA;          //��ŵ���������
		Zoffset=FLASH_MAG_Z_DATA;
			
		offset_GX=FLASH_Gyro_X_DATA;
		offset_GY=FLASH_Gyro_Y_DATA;         //�Ƕ�ԭ��У׼
		offset_GZ=FLASH_Gyro_Z_DATA;

		offset_AX=FLASH_Acel_X_DATA;
		offset_AY=FLASH_Acel_Y_DATA;         //���ٶ�ԭ��У׼
		offset_AZ=FLASH_Acel_Z_DATA;

		external_KP=FLASH_OUTPID_P_DATA;
		external_KI=FLASH_OUTPID_I_DATA;    //�⻷PID
		external_KD=FLASH_OUTPID_D_DATA; 
		                                
		interior_KP=FLASH_INPID_P_DATA;
		interior_KI=FLASH_INPID_I_DATA;      //�ڻ�PID
		interior_KD=FLASH_INPID_D_DATA;
		
		YAWexternal_KP=FLASH_YAWOUTPID_P_DATA;
		YAWexternal_KI=FLASH_YAWOUTPID_I_DATA;    //ƫ���⻷PID
		YAWexternal_KD=FLASH_YAWOUTPID_D_DATA; 
		                                
		YAWinterior_KP=FLASH_YAWINPID_P_DATA;
		YAWinterior_KI=FLASH_YAWINPID_I_DATA;      //ƫ���ڻ�PID
		YAWinterior_KD=FLASH_YAWINPID_D_DATA;
//		                                interior_KP=100;
//                                      external_KP=0;
		
		external_KP_f=(float)external_KP/1000.0; 			//�⻷
		external_KI_f=(float)external_KI/1000.0;
		external_KD_f=(float)external_KD/100.0;

		interior_KP_f=(float)interior_KP/1000.0; 			//�ڻ� ����
		interior_KI_f=(float)interior_KI/1000.0;
		interior_KD_f=(float)interior_KD/1000.0;
		
		YAWexternal_KP_f=(float)YAWexternal_KP/1000.0; 			//ƫ���⻷
		YAWexternal_KI_f=(float)YAWexternal_KI/1000.0;
		YAWexternal_KD_f=(float)YAWexternal_KD/1000.0;

		YAWinterior_KP_f=(float)YAWinterior_KP/1000.0; 			//ƫ���ڻ� ����
		YAWinterior_KI_f=(float)YAWinterior_KI/1000.0;
		YAWinterior_KD_f=(float)YAWinterior_KD/1000.0;

//			printf("\n\n������%d,\t%d,\t%d\n\n",Xoffset,Yoffset,Zoffset);
//			printf("�� �Ȳ�����%d,\t%d,\t%d\n\n",offset_GX,offset_GY,offset_GZ);
//			printf("���ٶȲ�����%d,\t%d,\t%d\n\n",offset_AX,offset_AY,offset_AZ);
//			write_flash_data();			
			
}
