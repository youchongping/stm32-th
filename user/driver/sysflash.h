#ifndef __SYSFLASH_H
#define __SYSFLASH_H
#include "stm32f10x.h"
#define Page_Address_Base (0x08000000)
#define Page(x) (Page_Address_Base+x*2*1024 )//2k bytes per page ,f103ve has 512k, so x is 0~255

#define APP_UPDATE_FLAG_ADDRESS (Page(62) )

#define APP_UPDATE_FLAG_SET (0x05050505 )
#define APP_UPDATE_FLAG_RESET (0x0 )
#define APP_UPDATE_NO_PROGRAM (0xFFFFFFFF )

#define MCU_ID 0x1FFFF7E8 //STM32 UUID
#define FLASH_SIZE_BASE 0x1FFFF7E0 //FLASH SIZE READ

s8 Flash_Program_bytes(uint32_t Address,u8 *buf,u16 bufsize);
uint32_t FLASH_ReadWord(uint32_t address);
void Read_MCU_ID(u8* id,u8 id_len);
void ReadFlashSize(void);
void WriteUpdateFlag(u32 flag);
#endif

