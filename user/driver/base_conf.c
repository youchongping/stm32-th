#include "base_conf.h"
#include "stm32f10x.h"
#include "stdio.h"

RCC_ClocksTypeDef RCC_ClocksFreq;

u16 gTimeBase_1ms=0;


void RCC_Configuration(void)
{
	//SystemInit();//do not call this function ! not must call,called in startup_stm32f10x_md.s before main() !!!!!!caultion will change SCB->VTOR = 0X8000000;FUCK!!!!
	RCC_ClockSecuritySystemCmd(ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | \
												 RCC_APB2Periph_GPIOA | \
												 RCC_APB2Periph_GPIOB | \
												 RCC_APB2Periph_GPIOE | \
												 RCC_APB2Periph_GPIOC | \
												 RCC_APB2Periph_GPIOD,ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | \
												 RCC_APB1Periph_TIM4,ENABLE);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	
	
}
//counter clk is 40khz
void IWDG_Init(u16 ms)
{
	  u16 rlr=0;
	  rlr = ms *40 / 256;
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 
    IWDG_SetPrescaler(6);   //prer 
    IWDG_SetReload(rlr);     
    IWDG_ReloadCounter();    
    IWDG_Enable();  //
}

void IWDG_Feed(void)
{
    IWDG_ReloadCounter();    /*reload*/
}

void SysTick_Configuration(void)
{
	RCC_GetClocksFreq(&RCC_ClocksFreq);
	if(SysTick_Config(RCC_ClocksFreq.SYSCLK_Frequency/1000))//1ms 
	{
		/**capture error*/
		
		while(1);
	}
}


u16 GetCurrentTime(void)
{
	return gTimeBase_1ms;
}


u16 TimeRunOut(u16 TimePre)
{
	u16 timelapse;
	if(TimePre<gTimeBase_1ms)
	{
		timelapse=gTimeBase_1ms-TimePre;
	}
	else
	{
		timelapse=0x10000+gTimeBase_1ms-TimePre;//when u16 overflow
	}
	return timelapse;
}

//do not use in interrupt
void Delay_ms(u16 nCount)
{
	u16 temptime;
	temptime=gTimeBase_1ms;
	while(TimeRunOut(temptime)<=nCount);
}

void stm32Reset(void)
{
	__set_FAULTMASK(1);     // turn off all interrupt
	NVIC_SystemReset();		// reset
}

void MCO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	  RCC_MCOConfig(RCC_MCO_HSE ); 
}

