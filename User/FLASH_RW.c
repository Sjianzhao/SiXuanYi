#include "FLASH_RW.h"

/*0x0800 0000 ~ 0x0801 0000 *///64k程序 2k一页
/*0x0801 0000 ~ 0x0808 0000 *///437keeprom 2k一页 0x800一页

//0x0801 0000~0x0801 07FF		  2Kbyte
//0x0801 0800~0x0801 0FFF 		2Kbyte
//0x0801 1000~0x0801 17FF 		2Kbyte
//0x0801 1800~0x0801 1FFF 		2Kbyte

//2K可以存512个 int型数据

volatile FLASH_Status FLASHStatus = FLASH_COMPLETE; //Flash操作状态变量

uint8_t flash_data[FLASH_DATA_LONG][4];//数据缓存

int FLASH_MAG_X_DATA;
int FLASH_MAG_Y_DATA;          //地磁原点校准
int FLASH_MAG_Z_DATA;

int FLASH_Gyro_X_DATA;
int FLASH_Gyro_Y_DATA;         //角度原点校准
int FLASH_Gyro_Z_DATA;

int FLASH_Acel_X_DATA;
int FLASH_Acel_Y_DATA;         //加速度原点校准
int FLASH_Acel_Z_DATA;

int FLASH_INPID_P_DATA;
int FLASH_INPID_I_DATA;       //内环PID
int FLASH_INPID_D_DATA;

int FLASH_OUTPID_P_DATA;
int FLASH_OUTPID_I_DATA;     //外环PID
int FLASH_OUTPID_D_DATA;

int FLASH_YAWINPID_P_DATA;
int FLASH_YAWINPID_I_DATA;       //YAW内环PID
int FLASH_YAWINPID_D_DATA;

int FLASH_YAWOUTPID_P_DATA;
int FLASH_YAWOUTPID_I_DATA;     //YAW外环PID
int FLASH_YAWOUTPID_D_DATA;

/**********************操作函数****************************

写入现在的数据到flash中			     write_flash_data();			
读取flash中存放的数据到程序中			read_flash_data();
                                                                
		ReadNum = ReadFlashNBtye(0, Temp_Data,4); //读取数据

		WriteFlashOneWord(0,0x12345678); //写入数据
*************************************************************
*Name: ReadFlashNBtye 

*Function: 从内部Flash读取N字节数据 

*Input: ReadAddress：数据地址（偏移地址）ReadBuf：数据指针 ReadNum：读取字节数 

*Output: 读取的字节数 

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

				  
//	  //写操作
//		FLASH_Unlock();  //解锁FLASH编程擦除控制器
//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//清除标志位
//     /*********************************************************************************
//           //               FLASH_FLAG_BSY            FLASH忙标志位
//          //               FLASH_FLAG_EOP            FLASH操作结束标志位
//          //               FLASH_FLAG_PGERR            FLASH编写错误标志位
//          //               FLASH_FLAG_WRPRTERR       FLASH页面写保护错误标净         
//      **********************************************************************************/
//      FLASH_ErasePage(FLASH_START_ADDR);     //擦除指定地址页
//     FLASH_ProgramHalfWord(FLASH_START_ADDR+(addr+i)*2,dat); //从指定页的addr地址开始写
//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//清除标志位
//     FLASH_Lock();    //锁定FLASH编程擦除控制器

//	//读操作
//	u16 value;
//     value = *(u16*)(FLASH_START_ADDR+(addr*2));//从指定页的addr地址开始读

/**************************************************************** 

*Name: WriteFlashOneWord 

*Function: 向内部Flash写入32位数据 

*Input: WriteAddress：数据地址（偏移地址）WriteData：写入数据 

*Output: NULL 

*Author: ValerianFan 

****************************************************************/ 



void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData) 

{ 

		FLASH_UnlockBank1();//解锁flash 

//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//清除标志位		
//	   FLASH_ErasePage(STARTADDR + WriteAddress);     //擦除指定地址页
//     FLASH_ProgramHalfWord(STARTADDR + WriteAddress+2,WriteData); //从指定页的addr地址开始写
//     FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//清除标志位
	



		FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);//清除标志位 
	
		FLASHStatus = FLASH_ErasePage(STARTADDR);  //擦除指定地址页
//	   FLASH_ErasePage(STARTADDR + WriteAddress);     //擦除指定地址页

		if(FLASHStatus == FLASH_COMPLETE)   //从指定页的addr地址开始写

		{ 

				FLASHStatus = FLASH_ProgramWord(STARTADDR + FLASH_MAG_X_ADD*4, FLASH_MAG_X_DATA); //flash.c 中API函数 
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
			
				//FLASHStatus = FLASH_ProgramWord(StartAddress+4, 0x56780000); //需要写入更多数据时开启 

				//FLASHStatus = FLASH_ProgramWord(StartAddress+8, 0x87650000); //需要写入更多数据时开启 

		} 

		FLASH_LockBank1(); //上锁 flash

} 

/**************************/
/*获取flash中存放的数据*/
/**************************/

void read_flash_data(void)
{
//		uint8_t flash_data[FLASH_DATA_LONG][4];
		ReadFlashNBtye(0, flash_data[0], FLASH_DATA_LONG*4);//读取数据（读取头地址，缓存地址，读取长度）
	
		FLASH_MAG_X_DATA=data_turn(FLASH_MAG_X_ADD);//电子罗盘数据
		FLASH_MAG_Y_DATA=data_turn(FLASH_MAG_Y_ADD);          //地磁原点校准
		FLASH_MAG_Z_DATA=data_turn(FLASH_MAG_Z_ADD);

		FLASH_Gyro_X_DATA=data_turn(FLASH_Gyro_X_ADD);
		FLASH_Gyro_Y_DATA=data_turn(FLASH_Gyro_Y_ADD);         //角度原点校准
		FLASH_Gyro_Z_DATA=data_turn(FLASH_Gyro_Z_ADD);

		FLASH_Acel_X_DATA=data_turn(FLASH_Acel_X_ADD);
		FLASH_Acel_Y_DATA=data_turn(FLASH_Acel_Y_ADD);         //加速度原点校准
		FLASH_Acel_Z_DATA=data_turn(FLASH_Acel_Z_ADD);

		FLASH_INPID_P_DATA=data_turn(FLASH_INPID_P_ADD);
		FLASH_INPID_I_DATA=data_turn(FLASH_INPID_I_ADD);       //内环PID
		FLASH_INPID_D_DATA=data_turn(FLASH_INPID_D_ADD);

		FLASH_OUTPID_P_DATA=data_turn(FLASH_OUTPID_P_ADD);
		FLASH_OUTPID_I_DATA=data_turn(FLASH_OUTPID_I_ADD);     //外环PID
		FLASH_OUTPID_D_DATA=data_turn(FLASH_OUTPID_D_ADD);
		
		FLASH_YAWINPID_P_DATA=data_turn(FLASH_YAWINPID_P_ADD);
		FLASH_YAWINPID_I_DATA=data_turn(FLASH_YAWINPID_I_ADD);       //偏航内环PID
		FLASH_YAWINPID_D_DATA=data_turn(FLASH_YAWINPID_D_ADD);

		FLASH_YAWOUTPID_P_DATA=data_turn(FLASH_YAWOUTPID_P_ADD);
		FLASH_YAWOUTPID_I_DATA=data_turn(FLASH_YAWOUTPID_I_ADD);     //偏航外环PID
		FLASH_YAWOUTPID_D_DATA=data_turn(FLASH_YAWOUTPID_D_ADD);

}

/**************************/
/*写入flash中存放的数据*/
/**************************/

void write_flash_data(void)
{
//		uint8_t flash_data[FLASH_DATA_LONG][4];
		WriteFlashOneWord(0,0); //写入数据 (开始地址，32位的数据)
		
}

/**************************/
/*缓存中的数据整理*/
/*输入读取的数据，返回int型数据*/
/**************************/

int data_turn(uint16_t FLASH_DATA)
{
	  int data;

		data=(flash_data[FLASH_DATA][3]<<24)|(flash_data[FLASH_DATA][2]<<16)|(flash_data[FLASH_DATA][1]<<8)|flash_data[FLASH_DATA][0];//获取数据等待使用

    return data;
}


void Flash_data_init(void)
{

//		extern T_float_xyz Mag_corr;//校准后
		extern short Xoffset,Yoffset,Zoffset;  //补偿
		extern short offset_GX,offset_GY,offset_GZ;  //补偿
		extern short offset_AX,offset_AY,offset_AZ;  //补偿
	
		extern int external_KP; 			//外环
		extern int external_KI;
		extern int external_KD;

		extern int interior_KP; 			//内部
		extern int interior_KI;
		extern int interior_KD;
	
		extern int YAWexternal_KP; 			//偏航外环
		extern int YAWexternal_KI;
		extern int YAWexternal_KD;

		extern int YAWinterior_KP; 			//偏航内部
		extern int YAWinterior_KI;
		extern int YAWinterior_KD;
	
		extern int FLASH_MAG_X_DATA;
		extern int FLASH_MAG_Y_DATA;          //地磁原点校准
		extern int FLASH_MAG_Z_DATA;
			
		extern int FLASH_Gyro_X_DATA;
		extern int FLASH_Gyro_Y_DATA;         //角度原点校准
		extern int FLASH_Gyro_Z_DATA;

		extern int FLASH_Acel_X_DATA;
		extern int FLASH_Acel_Y_DATA;         //加速度原点校准
		extern int FLASH_Acel_Z_DATA;
	
		extern int FLASH_INPID_P_DATA;
		extern int FLASH_INPID_I_DATA;       //内环PID
		extern int FLASH_INPID_D_DATA;

		extern int FLASH_OUTPID_P_DATA;
		extern int FLASH_OUTPID_I_DATA;     //外环PID
		extern int FLASH_OUTPID_D_DATA;
		
		extern float external_KP_f; 			//外环
		extern float external_KI_f;
		extern float external_KD_f;

		extern float interior_KP_f; 			//内部
		extern float interior_KI_f;
		extern float interior_KD_f;
		
		extern int FLASH_YAWINPID_P_DATA;
		extern int FLASH_YAWINPID_I_DATA;       //偏航内环PID
		extern int FLASH_YAWINPID_D_DATA;

		extern int FLASH_YAWOUTPID_P_DATA;
		extern int FLASH_YAWOUTPID_I_DATA;     //偏航外环PID
		extern int FLASH_YAWOUTPID_D_DATA;
		
		extern float YAWexternal_KP_f; 			//偏航外环
		extern float YAWexternal_KI_f;
		extern float YAWexternal_KD_f;

		extern float YAWinterior_KP_f; 			//偏航内部
		extern float YAWinterior_KI_f;
		extern float YAWinterior_KD_f;
			
		read_flash_data();//读取存放的flash数据
		Xoffset=FLASH_MAG_X_DATA;
		Yoffset=FLASH_MAG_Y_DATA;          //存放到数据里面
		Zoffset=FLASH_MAG_Z_DATA;
			
		offset_GX=FLASH_Gyro_X_DATA;
		offset_GY=FLASH_Gyro_Y_DATA;         //角度原点校准
		offset_GZ=FLASH_Gyro_Z_DATA;

		offset_AX=FLASH_Acel_X_DATA;
		offset_AY=FLASH_Acel_Y_DATA;         //加速度原点校准
		offset_AZ=FLASH_Acel_Z_DATA;

		external_KP=FLASH_OUTPID_P_DATA;
		external_KI=FLASH_OUTPID_I_DATA;    //外环PID
		external_KD=FLASH_OUTPID_D_DATA; 
		                                
		interior_KP=FLASH_INPID_P_DATA;
		interior_KI=FLASH_INPID_I_DATA;      //内环PID
		interior_KD=FLASH_INPID_D_DATA;
		
		YAWexternal_KP=FLASH_YAWOUTPID_P_DATA;
		YAWexternal_KI=FLASH_YAWOUTPID_I_DATA;    //偏航外环PID
		YAWexternal_KD=FLASH_YAWOUTPID_D_DATA; 
		                                
		YAWinterior_KP=FLASH_YAWINPID_P_DATA;
		YAWinterior_KI=FLASH_YAWINPID_I_DATA;      //偏航内环PID
		YAWinterior_KD=FLASH_YAWINPID_D_DATA;
//		                                interior_KP=100;
//                                      external_KP=0;
		
		external_KP_f=(float)external_KP/1000.0; 			//外环
		external_KI_f=(float)external_KI/1000.0;
		external_KD_f=(float)external_KD/100.0;

		interior_KP_f=(float)interior_KP/1000.0; 			//内环 数据
		interior_KI_f=(float)interior_KI/1000.0;
		interior_KD_f=(float)interior_KD/1000.0;
		
		YAWexternal_KP_f=(float)YAWexternal_KP/1000.0; 			//偏航外环
		YAWexternal_KI_f=(float)YAWexternal_KI/1000.0;
		YAWexternal_KD_f=(float)YAWexternal_KD/1000.0;

		YAWinterior_KP_f=(float)YAWinterior_KP/1000.0; 			//偏航内环 数据
		YAWinterior_KI_f=(float)YAWinterior_KI/1000.0;
		YAWinterior_KD_f=(float)YAWinterior_KD/1000.0;

//			printf("\n\n补偿：%d,\t%d,\t%d\n\n",Xoffset,Yoffset,Zoffset);
//			printf("角 度补偿：%d,\t%d,\t%d\n\n",offset_GX,offset_GY,offset_GZ);
//			printf("加速度补偿：%d,\t%d,\t%d\n\n",offset_AX,offset_AY,offset_AZ);
//			write_flash_data();			
			
}
