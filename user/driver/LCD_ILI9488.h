
#ifndef _LCD_ILI9488_H
#define _LCD_ILI9488_H

#include "stm32f10x.h"
#include "lcd.h"
#include "LCD_ILI9488.h"

#define ILI9488_BASE       LCD_BASE

#define ILI9488_DMA		(ILI9488_BASE + (1 << (15 + 1)))
#define ILI9488_REG		LCD_REG16
#define ILI9488_RAM		LCD_DAT16	

extern uint16_t g_ChipID;
extern uint16_t g_LcdHeight;			
extern uint16_t g_LcdWidth ;		
extern uint8_t s_ucBright;					
extern uint8_t g_LcdDirection;	
		

/* 可供外部模块调用的函数 */
uint32_t ILI9488_ReadID(void);
void ILI9488_InitHard(void);
void ILI9488_DispOn(void);
void ILI9488_DispOff(void);
void ILI9488_ClrScr(uint16_t _usColor);
void ILI9488_PutPixel(uint16_t _usX, uint16_t _usY, uint16_t _usColor);
uint16_t ILI9488_GetPixel(uint16_t _usX, uint16_t _usY);
void ILI9488_DrawLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usY2 , uint16_t _usColor);
void ILI9488_DrawHLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usX2 , uint16_t _usColor);
void ILI9488_DrawHColorLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor);
void ILI9488_DrawHTransLine(uint16_t _usX1 , uint16_t _usY1, uint16_t _usWidth, const uint16_t *_pColor);
void ILI9488_DrawRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);
void ILI9488_FillRect(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t _usColor);
void ILI9488_DrawCircle(uint16_t _usX, uint16_t _usY, uint16_t _usRadius, uint16_t _usColor);
void ILI9488_DrawBMP(uint16_t _usX, uint16_t _usY, uint16_t _usHeight, uint16_t _usWidth, uint16_t *_ptr);

void ILI9488_SetBackLight(uint8_t _bright);
void ILI9488_SetDirection(uint8_t _ucDir);

void ILI9488_DrawVLine(uint16_t _usX1 , uint16_t _usY1 , uint16_t _usY2 , uint16_t _usColor);

void ILI9488_InitHard(void);
#endif

