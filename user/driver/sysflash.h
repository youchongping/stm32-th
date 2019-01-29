#ifndef __SYSFLASH_H
#define __SYSFLASH_H
#include "stm32f10x.h"
#define Page_Address_Base (0x08000000)
#define Page(x) (Page_Address_Base+x*1*1024 )//1k bytes per page ,f101v8t6 has 64k, so x is 0~63
#define APP_UPDATE_FLAG_ADDRESS (Page(62) )
#define APP_UPDATE_FLAG_SET (0x05050505 )
#define APP_UPDATE_FLAG_RESET (0x0 )
#define APP_UPDATE_NO_PROGRAM (0xFFFFFFFF )
typedef __packed struct
{
	u8 giftnum;
	u8 GiftIsOutFlag;
}Gift_t;

extern Gift_t Gift;
//static  s8 Flash_Program_bytes(uint32_t *Address,u8 *buf,u16 *bufsize);
static uint32_t FLASH_ReadWord(uint32_t address);
void Read_DEVID(u8 *buf);
s8 WriteGiftNumToFlash(u8 *num);
u8 ReadGiftNumFromFlash(void);
void GiftRecord(void);
void WriteUpdateFlag(u32 flag);
#endif

