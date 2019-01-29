#ifndef _MAIN_H_
#define _MAIN_H_
#include "FreeRTOS.h"
#include "semphr.h"
#define FIREWARE_VERSION "ver1.0.0d"
extern void  app_printf(char *format, ...);
extern SemaphoreHandle_t  xMutex ;
extern QueueHandle_t public_queque ;
#endif