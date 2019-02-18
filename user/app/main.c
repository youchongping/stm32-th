#include <stdio.h> 
#include <stdarg.h>
#include "main.h"
#include "sysflash.h"
#include "base_conf.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "GUI.h"
#include "gui_task.h"
#include "key.h"
#include "led.h"
#include "uart_dma.h"
#include "bsp_ds18b20.h"
#include "timer.h"
#include "cc1101.h"
#include "event_groups.h"
#include "biss_ir.h"
#include "lcd.h"
#include "bsp_ts_ft5x06.h"
#include "bsp_i2c_gpio.h"
#include "bsp_touch.h"
#include <stdlib.h>
#include "adc.h"
#include "cJSON.h"
SemaphoreHandle_t  xMutex = NULL;
QueueHandle_t public_queque = NULL;
QueueHandle_t cc1101_queque = NULL;
EventGroupHandle_t cc1101_event_group = NULL;
EventGroupHandle_t human_detect_event_group = NULL;
void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
}
void cJson_init(void)
{
	cJSON_Hooks hooks;
	hooks.malloc_fn = pvPortMalloc;
	hooks.free_fn = vPortFree;
	cJSON_InitHooks(&hooks);
}
static void app_ObjCreate (void)
{
  xMutex = xSemaphoreCreateMutex();
  public_queque = xQueueCreate(4,20);	
  cc1101_queque = xQueueCreate(4,sizeof(CC1101_PACKET *));
  cc1101_event_group = xEventGroupCreate();
	human_detect_event_group = xEventGroupCreate();
}
void  app_printf(char *format, ...)
{
    char  buf_str[200 + 1];
    va_list   v_args;
    va_start(v_args, format);
   (void)vsnprintf((char*)&buf_str[0],(size_t) sizeof(buf_str),(char const *) format,v_args);
    va_end(v_args);
		xSemaphoreTake(xMutex, portMAX_DELAY);
    printf("%s", buf_str);
   	xSemaphoreGive(xMutex);
}
void bsp_init(void)
{
  u8 mcu_id[12];
	SCB->VTOR = (FLASH_BASE|0x0000); /* Vector Table Relocation in Internal FLASH. */
	RCC_Configuration(); 
	SysTick_Configuration();
	NVIC_Configuration();
	
  //IWDG_Init(12000);//max time 12000ms
	serial_init();
	TIM3_Int_Init();
	TIM2_Int_Init();
  ds18b20_init();
  led_gpio_init();
	biss_ir_init();
	bsp_InitI2C();
	TOUCH_InitHard();
	Adc_Init();
	
	Read_MCU_ID(mcu_id,sizeof(mcu_id));
	ReadFlashSize();
	
}

static void gui_task(void *pvParameters)
{
	while (1) 
	{
		MainTask();
	}
}


void human_detect_task(void* param )
{
	uint16_t return_bits;
	while(1)
	{
		
		return_bits = xEventGroupWaitBits(human_detect_event_group, EVENT_IR_DETECTED,  pdTRUE,  pdFALSE,  portMAX_DELAY);
		if((return_bits & EVENT_IR_DETECTED) == EVENT_IR_DETECTED) 
		{

		}
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

void tp_task(void *param)
{  
  while(1)
   {
		if(g_tFT5X06.Enable == 1)
		{
				FT5X06_OnePiontScan();
		}
		vTaskDelay(10 / portTICK_RATE_MS);
	}
}

int main(void)
{
		app_ObjCreate();
	  cJson_init();
		bsp_init();
		app_printf("bsp_init ok!\n");
		xTaskCreate(uart_task,"uart_task",256,NULL,6,NULL);
		xTaskCreate(human_detect_task,"human_detect_task",256,NULL,6,NULL);
		xTaskCreate(gui_task,"gui_task",2048,NULL,2,NULL);
		xTaskCreate(key_task,"key_task",1024,NULL,6,NULL);
		xTaskCreate(led_task,"alive_check_task",256,NULL,6,NULL);
		xTaskCreate(cc1101_task,"cc1101_task",1024,NULL,7,NULL);
		xTaskCreate(tp_task,"tp_task",256,NULL,7,NULL);
		xTaskCreate(adc_task,"adc_task",256,NULL,5,NULL);
		vTaskStartScheduler();
		
}

