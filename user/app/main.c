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
#include "ft6x06.h"
#include "uart_dma.h"
#include "bsp_ds18b20.h"
#include "timer.h"
SemaphoreHandle_t  xMutex = NULL;
QueueHandle_t public_queque = NULL;
void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
}
static void app_ObjCreate (void)
{
  xMutex = xSemaphoreCreateMutex();
	public_queque = xQueueCreate(4,20);	
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

	SCB->VTOR = (FLASH_BASE|0x0000); /* Vector Table Relocation in Internal FLASH. */
	RCC_Configuration(); 
	SysTick_Configuration();
	NVIC_Configuration();
	
  //IWDG_Init(12000);//max time 12000ms
	serial_init();
	TIM3_Int_Init();
  ds18b20_init();
	//FT5216_Init();
  led_gpio_init();	
	set_green_led(ON);
	set_red_led(ON);
	
}
static void gui_task(void *pvParameters)
{
	while (1) 
	{
		MainTask();
	}
}

static void alive_check_task(void* param)
{
	set_red_led(OFF);
	while(1)
	{
		set_green_led(ON);
		vTaskDelay(100 / portTICK_RATE_MS);
		set_green_led(OFF);
		vTaskDelay(1000 / portTICK_RATE_MS);
		
	}
	
}
int main(void)
{
	app_ObjCreate();
	bsp_init();
	app_printf("bsp_init ok!\n");
  xTaskCreate(uart_task,"uart_task",256,NULL,6,NULL);
	xTaskCreate(gui_task,"gui_task",2048,NULL,4,NULL);
	xTaskCreate(key_task,"key_task",256,NULL,6,NULL);
	xTaskCreate(alive_check_task,"alive_check_task",256,NULL,6,NULL);
	vTaskStartScheduler();
	
	
}

