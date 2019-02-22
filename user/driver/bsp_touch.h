
#ifndef __BSP_TOUCH_H
#define __BSP_TOUCH_H

#define CALIB_POINT_COUNT	4		/* 2 = 2点校准； 4 = 四点校准 */

#define TOUCH_FIFO_SIZE		20
#include "stm32f10x.h"
typedef struct
{
	uint16_t XYChange;	/* X, Y 是否交换  */
	uint8_t Enable;		/* 触摸检测使能标志 */
	uint8_t Event[TOUCH_FIFO_SIZE];	/* 触摸事件 */
	int16_t XBuf[TOUCH_FIFO_SIZE];	/* 触摸坐标缓冲区 */
	int16_t YBuf[TOUCH_FIFO_SIZE];	/* 触摸坐标缓冲区 */
	uint8_t Read;					/* 缓冲区读指针 */
	uint8_t Write;					/* 缓冲区写指针 */
}TOUCH_T;

typedef struct
{
	uint32_t ParamVer;			
	uint8_t ucBackLight;
	uint8_t TouchDirection;	
	uint8_t XYChange;		
}
PARAM_T;

extern PARAM_T g_tParam;
/* 触摸事件 */
enum
{
	TOUCH_NONE = 0,		/* 无触摸 */
	TOUCH_DOWN = 1,		/* 按下 */
	TOUCH_MOVE = 2,		/* 移动 */
	TOUCH_RELEASE = 3	/* 释放 */
};

extern TOUCH_T g_tTP;

/* 供外部调用的函数声明 */
void TOUCH_InitHard(void);
uint8_t TOUCH_GetKey(int16_t *_pX, int16_t *_pY);
void TOUCH_CelarFIFO(void);
uint8_t TOUCH_InRect(uint16_t _usX, uint16_t _usY,
uint16_t _usRectX, uint16_t _usRectY, uint16_t _usRectH, uint16_t _usRectW);
void TOUCH_PutKey(uint8_t _ucEvent, uint16_t _usX, uint16_t _usY);
void TOUCH_CapScan(void);


#endif

