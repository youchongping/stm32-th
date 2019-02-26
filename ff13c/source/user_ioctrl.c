#include "user_ioctrl.h"
#include "ff.h"
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

int RAM_disk_status(void)
{
	return 0;
}
int MMC_disk_status(void)
{
	return 0;
}
int USB_disk_status(void)
{
	return 0;
}
int RAM_disk_initialize(void)
{
	return 0;
}
int MMC_disk_initialize(void)
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
int MMC_disk_read(BYTE* buff, DWORD sector, UINT count)
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
int MMC_disk_write(const BYTE* buff, DWORD sector, UINT count)
{
	return 0;
}
int USB_disk_write(const BYTE* buff, DWORD sector, UINT count)
{
	return 0;
}