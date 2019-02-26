#ifndef USER_IOCTRL_H
#define USER_IOCTRL_H
#include "ff.h"
DWORD get_fattime (void);
int RAM_disk_status(void);
int MMC_disk_status(void);
int USB_disk_status(void);
int RAM_disk_initialize(void);
int MMC_disk_initialize(void);
int USB_disk_initialize(void);
int RAM_disk_read(BYTE* buff, DWORD sector, UINT count);
int MMC_disk_read(BYTE* buff, DWORD sector, UINT count);
int USB_disk_read(BYTE* buff, DWORD sector, UINT count);
int RAM_disk_write(const BYTE* buff, DWORD sector, UINT count);
int MMC_disk_write(const BYTE* buff, DWORD sector, UINT count);
int USB_disk_write(const BYTE* buff, DWORD sector, UINT count);
#endif