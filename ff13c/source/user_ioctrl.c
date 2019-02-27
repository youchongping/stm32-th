#include "user_ioctrl.h"
#include "ff.h"
#include "bsp_spi_flash.h"
#include "diskio.h"
/*use MMC ic is w25q32*/
DWORD get_fattime (void)
{

	return	  ((DWORD)(2019 - 1980) << 25)	/* Year = 2013 */
			| ((DWORD)1 << 21)				/* Month = 1 */
			| ((DWORD)1 << 16)				/* Day_m = 1*/
			| ((DWORD)0 << 11)				/* Hour = 0 */
			| ((DWORD)0 << 5)				/* Min = 0 */
			| ((DWORD)0 >> 1);				/* Sec = 0 */
}

/*
The disk_write function is called to write data to the sector(s) of storage device.
pdrv
    Physical drive number to identify the target device.
buff
    Pointer to the first item of the byte array to be written. The size of data to be written is sector size * count bytes.
sector
    Start sector number in 32-bit LBA.
count
    Number of sectors to write. 
*/
int MMC_disk_write(const BYTE* buff, DWORD sector, UINT count)
{
	sf_WriteBuffer(( BYTE*)buff, sector << 12, count<<12);          
  return RES_OK;  
}
/*
The disk_read function is called to read data from the sector(s) of storage device.
pdrv
    Physical drive number to identify the target device.
buff
    Pointer to the first item of the byte array to store read data. Size of read data will be the sector size * count bytes.
sector
    Start sector number in 32-bit LBA.
count
    Number of sectors to read. 
*/
int MMC_disk_read(BYTE* buff, DWORD sector, UINT count)
{
	sf_ReadBuffer(buff, sector << 12, count<<12); //beacuse 1 sector is 4KB,so <<12.         
  return RES_OK;          
}
int RAM_disk_status(void)
{
	return 0;
}
int MMC_disk_initialize(void)
{
	bsp_InitSFlash();              
	return RES_OK;
}
int MMC_disk_status(void)
{
              
	return 0;
}

/*no those device*/
/*************************************************************************************/
int USB_disk_status(void)
{
	return 0;
}
int RAM_disk_initialize(void)
{
	return 0;
}

int USB_disk_initialize(void)
{
	return 0;
}

int RAM_disk_read(BYTE* buff, DWORD sector, UINT count)
{
	return 0;
}

int USB_disk_read(BYTE* buff, DWORD sector, UINT count)
{
	return 0;
}

int RAM_disk_write(const BYTE* buff, DWORD sector, UINT count)
{
	return 0;
}

int USB_disk_write(const BYTE* buff, DWORD sector, UINT count)
{
	return 0;
}