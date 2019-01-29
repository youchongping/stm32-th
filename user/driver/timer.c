#include "timer.h"
#include "led.h"

void TIM3_Int_Init()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = (100-1); 
	TIM_TimeBaseStructure.TIM_Prescaler =(36-1); 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
 
  while((TIM3->SR & TIM_FLAG_Update)!=SET){};
    TIM3->SR = (uint16_t)~TIM_FLAG_Update;				 
}
void delay_us(uint32_t us_cnt)
{
    TIM3->CNT = us_cnt-1;
    TIM3->CR1 |= TIM_CR1_CEN;    
    while((TIM3->SR & TIM_FLAG_Update)!=SET);
    TIM3->SR = (uint16_t)~TIM_FLAG_Update;
    TIM3->CR1 &= ~TIM_CR1_CEN;
}










