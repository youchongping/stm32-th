#include "sysflash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart_dma.h"
#include "sys.h"
#define GIFT_Base (Page(63) )
#define DEVID_base 0x1FFFF7E8 //DEVID起始地址 全球唯一ID
Gift_t Gift;

/*basic of flash*/
#if 0
static s8 Flash_Program_bytes(uint32_t *Address,u8 *buf,u16 *bufsize)
{
	u32 Data =0;
	
	 while((*bufsize)>0)
	 {
		  Data=(*buf)+ ((*(buf+1))<<8) + ((*(buf+2))<<16) + ((*(buf+3))<<24);
			FLASH_ProgramWord(*Address,Data);
		  delay_us(100);
			if(FLASH_ReadWord(*Address)!=Data){return -1;}
			(*bufsize)-=4;
			 buf+=4;
			(*Address)+=4;
	 }
	return 0;
}
#endif
/*read word*/
static uint32_t FLASH_ReadWord(uint32_t address)
{
  uint32_t temp1,temp2;
  temp1=*(__IO uint16_t*)address; 
  temp2=*(__IO uint16_t*)(address+2); 
  return (temp2<<16)+temp1;
}

/*Application of flash*/
s8 WriteGiftNumToFlash(u8 *num)
{
	u32 p=0;
	p=GIFT_Base;
	FLASH_Unlock();         //解锁写保护
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//清除标志位
	FLASH_ErasePage(p);
	FLASH_ProgramWord((u32)p,(u32)(*num));
	FLASH_Lock();
	return 0;
}


u8 ReadGiftNumFromFlash(void)
{
	return (u8)FLASH_ReadWord(GIFT_Base);
}

void WriteUpdateFlag(u32 flag)
{
	u32 address;
	address = APP_UPDATE_FLAG_ADDRESS;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//清除标志位
	FLASH_ErasePage(address);
	FLASH_ProgramWord((u32)address,(u32)(flag));
	FLASH_Lock();
}

void GiftRecord(void)
{
	if(Gift.GiftIsOutFlag)
		{
			Gift.giftnum=ReadGiftNumFromFlash();
		  Gift.giftnum--;
			WriteGiftNumToFlash(&Gift.giftnum);
		  printf("GIFT IS OUT!\x0d\x0a");
			Gift.GiftIsOutFlag = 0 ;
		}
}


