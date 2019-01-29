
#ifndef _BASE_CONF_H
#define _BASE_CONF_H

#include <string.h>
#include "stm32f10x.h"


#define RT_ERROR	 (-1)
#define RT_EOK	 (0)

#define EXTI_Line_ALL     ((uint32_t)0xFFFFF)

#define swapInt32(value)   	((((value) & 0x000000FF) << 24) |			\
 								(((value) & 0x0000FF00) << 8) |  			\
 			  		 			(((value) & 0x00FF0000) >> 8) |  			\
 			   					(((value) & 0xFF000000) >> 24) )

#define swapInt16(value)	   ((((value) & 0x00FF) << 8) | (((value) & 0xFF00) >> 8) )			


void RCC_Configuration(void);
void SysTick_Configuration(void);
u16 GetCurrentTime(void);
u16 TimeRunOut(u16 TimePre);
void Delay_ms(u16 nCount);
void stm32Reset(void);
void IWDG_Init(u16 ms);
void IWDG_Feed(void);
void MCO_Init(void);

#endif
