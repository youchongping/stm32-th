#include "led.h"
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR ^= GPIO_Pin;
}
void led_gpio_init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;              
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN ;
    GPIO_Init(GREEN_LED_GPIO, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = RED_LED_PIN ;
    GPIO_Init(RED_LED_GPIO, &GPIO_InitStructure);
}

void set_green_led(char state)
{

	GPIO_WriteBit(GREEN_LED_GPIO,GREEN_LED_PIN,(BitAction)state);
}	
void set_red_led(char state)
{

	GPIO_WriteBit(RED_LED_GPIO,RED_LED_PIN,(BitAction)state);
}	
