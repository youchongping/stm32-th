#ifndef _LED_H_
#define _LED_H_
#include "stm32f10x.h"

#define ON 1
#define OFF 0

#define GREEN_LED_PIN 	GPIO_Pin_0
#define GREEN_LED_GPIO GPIOA
#define RED_LED_PIN		GPIO_Pin_1
#define RED_LED_GPIO 	GPIOA

extern  void led_gpio_init(void);
extern void set_green_led(char state);
extern void set_red_led(char state);

#endif