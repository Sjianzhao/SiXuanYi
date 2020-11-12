#include "stm32f10x.h"

#include "stm32f10x_flash.h" //flash操作接口文件（在库文件中），必须要包含 


#define STARTADDR 0x08010000 //STM32F103RB 其他型号基本适用，未测试  64k存程序 

int ReadFlashNBtye(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadNum);
void WriteFlashOneWord(uint32_t WriteAddress,uint32_t WriteData);

#define FLASH_DATA_LONG 21         //数据个数

#define FLASH_MAG_X_ADD  0
#define FLASH_MAG_Y_ADD  1          //地磁原点校准
#define FLASH_MAG_Z_ADD  2

#define FLASH_Gyro_X_ADD  3
#define FLASH_Gyro_Y_ADD  4         //角度原点校准
#define FLASH_Gyro_Z_ADD  5

#define FLASH_Acel_X_ADD  6
#define FLASH_Acel_Y_ADD  7         //加速度原点校准
#define FLASH_Acel_Z_ADD  8

#define FLASH_INPID_P_ADD   9
#define FLASH_INPID_I_ADD   10       //内环PID
#define FLASH_INPID_D_ADD   11

#define FLASH_OUTPID_P_ADD   12
#define FLASH_OUTPID_I_ADD   13     //外环PID
#define FLASH_OUTPID_D_ADD   14

#define FLASH_YAWINPID_P_ADD   15
#define FLASH_YAWINPID_I_ADD   16       //内环PID
#define FLASH_YAWINPID_D_ADD   17

#define FLASH_YAWOUTPID_P_ADD   18
#define FLASH_YAWOUTPID_I_ADD   19     //外环PID
#define FLASH_YAWOUTPID_D_ADD   20





void read_flash_data(void);
void write_flash_data(void);
int data_turn(uint16_t FLASH_DATA);

void Flash_data_init(void);
