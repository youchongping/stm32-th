#ifndef _KEY_H_
#define _KEY_H_
#include "stm32f10x.h"
#define KEY1_PIN 	GPIO_Pin_1
#define KEY1_GPIO GPIOC
#define KEY2_PIN 	GPIO_Pin_2
#define KEY2_GPIO GPIOC
#define KEY3_PIN 	GPIO_Pin_3
#define KEY3_GPIO GPIOC

#define KEY1_PRESSED ((1<<0))
#define KEY2_PRESSED ((1<<1))
#define KEY3_PRESSED ((1<<2))
void key_task(void* param);
#endif