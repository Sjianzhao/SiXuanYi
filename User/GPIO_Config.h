#include "stm32f10x.h"
#include <stdio.h>


#define USART1_DR_Base  0x40013804		// 0x40013800 + 0x04 = 0x40013804
#define SENDBUFF_SIZE   5000

void USART1_Config(void);
void USART3_Config(void);
void USART1_DMA_Config(void);
void send_byte_USART1(u8 ch);
void send_USART1(unsigned char *DataToSend , u8 data_num);

//int fputc(int ch, FILE *f);
//void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);

void SPI_DATA_Init(void);

#define SPI_FLASH_SPI                           SPI1

#define SPI_FLASH_SPI_CLK                       RCC_APB2Periph_SPI1
#define SPI_FLASH_SPI_SCK_PIN                   GPIO_Pin_5                  /* PA.05 */
#define SPI_FLASH_SPI_SCK_GPIO_PORT             GPIOA                       /* GPIOA */

#define SPI_FLASH_SPI_SCK_GPIO_CLK              RCC_APB2Periph_GPIOA
#define SPI_FLASH_SPI_MISO_PIN                  GPIO_Pin_6                  /* PA.06 */
#define SPI_FLASH_SPI_MISO_GPIO_PORT            GPIOA                       /* GPIOA */

#define SPI_FLASH_SPI_MISO_GPIO_CLK             RCC_APB2Periph_GPIOA
#define SPI_FLASH_SPI_MOSI_PIN                  GPIO_Pin_7                  /* PA.07 */
#define SPI_FLASH_SPI_MOSI_GPIO_PORT            GPIOA                       /* GPIOA */

//#define SPI_FLASH_SPI_MOSI_GPIO_CLK             RCC_APB2Periph_GPIOA
//#define SPI_FLASH_CS_PIN                        GPIO_Pin_4                  /* PA.04 */
//#define SPI_FLASH_CS_GPIO_PORT                  GPIOA                       /* GPIOA */

#define SPI_FLASH_CS_GPIO_CLK                   RCC_APB2Periph_GPIOA


//#define SPI_FLASH_CLK_LOW()       GPIO_ResetBits(GPIOA, GPIO_Pin_5)
//#define SPI_FLASH_CLK_HIGH()      GPIO_SetBits(GPIOA, GPIO_Pin_5)

//#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(GPIOA, GPIO_Pin_4)
//#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(GPIOA, GPIO_Pin_4)

//#define SPI_FLASH_CE_LOW()       GPIO_ResetBits(GPIOC, GPIO_Pin_4)    //CE
//#define SPI_FLASH_CE_HIGH()      GPIO_SetBits(GPIOC, GPIO_Pin_4)

#define SPI_FLASH_CLK_LOW()       GPIOA->BRR	=0x20 
#define SPI_FLASH_CLK_HIGH()      GPIOA->BSRR	=0x20 

#define SPI_FLASH_CS_LOW()       GPIOA->BRR	=0x10 
#define SPI_FLASH_CS_HIGH()      GPIOA->BSRR	=0x10 

#define SPI_FLASH_CE_LOW()       GPIOC->BRR	=0x10   //CE
#define SPI_FLASH_CE_HIGH()      GPIOC->BSRR	=0x10 

//#define OLED_DC_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_5)//DC
//#define OLED_DC_Set() GPIO_SetBits(GPIOD,GPIO_Pin_5)

u8 SPI_FLASH_SendByte(u8 byte);
u32 SPI_FLASH_ReadDeviceID(void);


#define W25X_WriteEnable		      0x06 
#define W25X_WriteDisable		      0x04 
#define W25X_ReadStatusReg		    0x05 
#define W25X_WriteStatusReg		    0x01 
#define W25X_ReadData			        0x03 
#define W25X_FastReadData		      0x0B 
#define W25X_FastReadDual		      0x3B 
#define W25X_PageProgram		      0x02 
#define W25X_BlockErase			      0xD8 
#define W25X_SectorErase		      0x20 
#define W25X_ChipErase			      0xC7 
#define W25X_PowerDown			      0xB9 
#define W25X_ReleasePowerDown	    0xAB 
#define W25X_DeviceID			        0xAB 
#define W25X_ManufactDeviceID   	0x90 
#define W25X_JedecDeviceID		    0x9F 

#define WIP_Flag                  0x01  /* Write In Progress (WIP) flag */

#define Dummy_Byte                0xFF
