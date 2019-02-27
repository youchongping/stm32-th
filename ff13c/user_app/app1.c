#include "ff.h"
#include <stdio.h>
#include <string.h>
#include "app1.h"
void fs_test(void)
{
	FATFS fs;           /* Filesystem object */
    FIL fil;            /* File object */
    FRESULT res;  /* API result code */
    UINT bw;            /* Bytes written */
    BYTE work[FF_MAX_SS]; /* Work area (larger is better for processing time) */
	BYTE mm[50];
	UINT i;
	
	printf(" fs test start\r\n");

	res = f_mkfs("0:", FM_FAT, 0, work, sizeof (work));
	if (res)
	{
		printf("f_mkfs failed\r\n");
		return ;
	}
	else
	{
		printf("f_mkfs scucess\r\n");
	}

	res = f_mount(&fs, "0:", 0);
	if (res)
	{
		printf("f_mount fail\r\n");
	}
	else
	{
		printf("f_mount OK\r\n");
	}
	/* Create a file as new */
	res = f_open(&fil, "0:/hello.txt", FA_CREATE_NEW|FA_WRITE|FA_READ);
	if (res)
	{
		printf("f_open failed\r\n");
	}
	else
	{
		printf("f_open scucess\r\n");
	}
	/* Write a message */
	res = f_write(&fil, "Hello,World!", 12, &bw);
	//printf("res write:%d\r\n",res);
	if (bw == 12)
	{
		printf("f_write OK!\r\n");
	}
	else
	{
		printf("f_write failed\r\n");
	}
	res = f_size(&fil);
	printf("f_size:%d Bytes.\r\n",res);
	memset(mm,0x0,50);
	f_lseek(&fil,0);
	res = f_read(&fil,mm,12,&i);
	if (res == FR_OK)
	{
		printf("f_read OK !\r\n");
		printf("f_read size:%d Bytes.\r\n",i);
	}
	else
	{
		printf("f_read failed \r\n");
	}
	printf("f_read data:%s",mm);

	/* Close the file */
	f_close(&fil);
	f_mount(0, "0:", 0);
}