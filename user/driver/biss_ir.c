#include "biss_ir.h"
#include <stdio.h>
#include "led.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "timer.h"
#include "lcd.h"
xTimerHandle ir_timer  = NULL;
void biss_ir_gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	
	EXTI_InitTypeDef 	EXTI_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = BISS_VOUT_PIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 
	GPIO_Init(BISS_VOUT_GPIO, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(BISS_VOUT_PORT_SRC,BISS_VOUT_EXTI_PIN_SRC);

	EXTI_InitStructure.EXTI_Line = BISS_VOUT_EXTI_LINE; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = BISS_VOUT_EXTI_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}
void ir_timer_callback( xTimerHandle  xTimer )
{
		GPIO_WriteBit(LCD_BL_GPIO,LCD_BL_PIN,Bit_RESET);
}
void biss_ir_init()
{
		biss_ir_gpio_init();
		if(ir_timer == NULL)
        ir_timer = xTimerCreate("ir_timer ",  10*1000/portTICK_RATE_MS, pdTRUE, ( void * ) 0, ir_timer_callback);
		if(ir_timer != NULL)
				xTimerStart( ir_timer, 0 );
}


void EXTI2_IRQHandler(void)
{	
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  if(EXTI_GetFlagStatus(BISS_VOUT_EXTI_LINE)==SET)
  {
    
		xResult = xEventGroupSetBitsFromISR(human_detect_event_group,EVENT_IR_DETECTED,&xHigherPriorityTaskWoken);
		if(xResult != pdFAIL)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
    EXTI_ClearITPendingBit(BISS_VOUT_EXTI_LINE);
  }
}