#include "sysflash.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uart_dma.h"
#include "sys.h"
#include "timer.h"

/*basic of flash*/
#if 1
 s8 Flash_Program_bytes(uint32_t Address,u8 *buf,u16 bufsize)
{
	u32 Data =0;
	FLASH_Unlock();        
	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//
	FLASH_ErasePage(Address);
	 while(bufsize>0)
	 {
		  Data=(*buf)+ ((*(buf+1))<<8) + ((*(buf+2))<<16) + ((*(buf+3))<<24);
			FLASH_ProgramWord(Address,Data);
		  delay_us(100);
			if(FLASH_ReadWord(Address)!=Data){return -1;}
			 bufsize-=4;
			 buf+=4;
			 Address+=4;
	 }
	 FLASH_Lock();
	return 0;
}
#endif

uint32_t FLASH_ReadWord(uint32_t address)
{
  uint32_t temp1,temp2;
  temp1=*(__IO uint16_t*)address; 
  temp2=*(__IO uint16_t*)(address+2); 
  return (temp2<<16)+temp1;
}

/*application of flash*/
void Read_MCU_ID(u8* id,u8 id_len)
{
	u32 buffer,p;
	u8 buf[12];
	memset(buf,0,sizeof(buf));
	p=MCU_ID;
	buffer=FLASH_ReadWord(p);
	buf[0]=buffer;
	buf[1]=buffer>>8;
	buf[2]=buffer>>16;
	buf[3]=buffer>>24;
	buffer=FLASH_ReadWord(p+4);
	buf[4]=buffer;
	buf[5]=buffer>>8;
	buf[6]=buffer>>16;
	buf[7]=buffer>>24;
	buffer=FLASH_ReadWord(p+8);
	buf[8]=buffer;
	buf[9]=buffer>>8;
	buf[10]=buffer>>16;
	buf[11]=buffer>>24;
	memset(id,0,id_len);
	memcpy(id,buf,id_len);
	
	u8 i;
	printf("MCU id:");
  for(i=0;i<12;i++)
	printf("%02x",buf[i]);
	printf("\r\n");
}
void ReadFlashSize(void)
{
	u16 buffer=0;
	buffer = *(__IO uint16_t*)FLASH_SIZE_BASE;
	printf("MCU flash size:%d k\r\n",buffer);
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


