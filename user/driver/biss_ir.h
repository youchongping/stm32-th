#ifndef _BISS_IR_H
#define _BISS_IR_H
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "timer.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#define BISS_VOUT_GPIO 						GPIOE
#define BISS_VOUT_PIN  						GPIO_Pin_2
#define BISS_VOUT_PORT_SRC        GPIO_PortSourceGPIOE
#define BISS_VOUT_EXTI_PIN_SRC    GPIO_PinSource2
#define BISS_VOUT_EXTI_LINE       EXTI_Line2
#define BISS_VOUT_EXTI_IRQn       EXTI2_IRQn

extern u8 ir_disable;
extern void biss_ir_init(void);
extern xTimerHandle ir_timer;
#endif