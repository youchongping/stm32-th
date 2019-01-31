#ifndef _MAIN_H_
#define _MAIN_H_
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#define FIREWARE_VERSION "ver1.0.0d"
enum human_detect_enum
{
	EVENT_IR_DETECTED = (1<<0),
};

extern void  app_printf(char *format, ...);
extern SemaphoreHandle_t  xMutex ;
extern QueueHandle_t public_queque ;
extern QueueHandle_t cc1101_queque ;
extern EventGroupHandle_t cc1101_event_group;
extern EventGroupHandle_t human_detect_event_group;
#endif