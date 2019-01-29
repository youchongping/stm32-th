#ifndef __LCD_h__
#define __LCD_h__
#include "stm32f10x.h"
#include "sys.h"
/* LCD  CS connect FMSC-NE1 */
/* LCD  RS connect FMSC-A16*/
#define LCD_BASE   ((uint32_t)(0x60000000))
#define LCD_REG16  (*((volatile u16 *)(LCD_BASE  ))) 
#define LCD_DAT16  (*((volatile u16 *)(LCD_BASE|(1<<17))))
#define LCD_RESET		PAout(15)

#define LCD_DB_0_3 GPIOD
#define LCD_DB_4_12 GPIOE
#define LCD_DB_13_15 GPIOD

#define LCD_RD_PIN 	GPIO_Pin_4
#define LCD_RD_GPIO 	GPIOD
#define LCD_CS_PIN 	GPIO_Pin_7
#define LCD_CS_GPIO 	GPIOD
#define LCD_RS_PIN 	GPIO_Pin_11
#define LCD_RS_GPIO 	GPIOD
#define LCD_WR_PIN 	GPIO_Pin_5
#define LCD_WR_GPIO 	GPIOD

#define LCD_RST_PIN 	GPIO_Pin_15
#define LCD_RST_GPIO 	GPIOA
#define LCD_BL_PIN 	GPIO_Pin_2
#define LCD_BL_GPIO 	GPIOB
#define LCD_IM0_PIN 	GPIO_Pin_3
#define LCD_IM0_GPIO  GPIOD
#define LCD_IM1_PIN 	GPIO_Pin_2
#define LCD_IM1_GPIO  GPIOD
#define LCD_IM2_PIN 	GPIO_Pin_12
#define LCD_IM2_GPIO  GPIOC


extern void user_LCD_init(void);


#endif
