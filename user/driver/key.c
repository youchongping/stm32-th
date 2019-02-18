#include "key.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stdio.h> 
#include "cc1101.h"
#include "main.h"

extern CC1101_PACKET cc1101_tx_test ;
CC1101_PACKET *p_tx_test;
void user_reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}
void key_gpio_init(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;              
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

   GPIO_InitStructure.GPIO_Pin =  KEY1_PIN ;
   GPIO_Init(KEY1_GPIO, &GPIO_InitStructure);

	 GPIO_InitStructure.GPIO_Pin =  KEY2_PIN ;
   GPIO_Init(KEY2_GPIO, &GPIO_InitStructure);

	 GPIO_InitStructure.GPIO_Pin =  KEY3_PIN ;
   GPIO_Init(KEY3_GPIO, &GPIO_InitStructure);
}
u8 key_scan(void)
{
	u8 key_value = 0 ;
	u8 i=3;
	while(i--)
	{
		key_value |= ((GPIO_ReadInputDataBit(KEY1_GPIO,KEY1_PIN)==0)?1:0)<<0;
		key_value |= ((GPIO_ReadInputDataBit(KEY2_GPIO,KEY2_PIN)==0)?1:0)<<1;
		key_value |= ((GPIO_ReadInputDataBit(KEY3_GPIO,KEY3_PIN)==0)?1:0)<<2;
		vTaskDelay(50 / portTICK_RATE_MS);
	}
	return key_value;
}
void key_deal(u8 *key_value)
{

	switch(*key_value)
	{
		case KEY1_PRESSED:
			printf("key 1 pressed \n");
			break;
		case KEY2_PRESSED:
			p_tx_test = &cc1101_tx_test;
			if(xQueueSend(cc1101_queque, (void *)&p_tx_test, 0) == pdPASS){}
			printf("key 2 pressed \n");
			break;
		case KEY3_PRESSED:
			user_reset();
			printf("key 3 pressed \n");
			break;
		
		default:{break;}	
    		
	}

	*key_value = 0 ;
}

void key_task(void* param)
{
	  key_gpio_init();
	  u8 key_value =0 ;
		while(1)
		{
			key_value =key_scan();
			if(key_value!=0)
			{
				key_deal(&key_value);
			}
			vTaskDelay(100 / portTICK_RATE_MS);
		}
}
