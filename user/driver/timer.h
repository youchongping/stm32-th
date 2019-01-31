#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void TIM3_Int_Init(void);
void TIM2_Int_Init(void);
void delay_us(uint32_t us_cnt); 
void delay_ms(uint32_t ms_cnt);
#endif
