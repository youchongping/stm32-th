
#ifndef __BSP_TOUCH_H
#define __BSP_TOUCH_H

#define CALIB_POINT_COUNT	4		/* 2 = 2��У׼�� 4 = �ĵ�У׼ */

#define TOUCH_FIFO_SIZE		20
#include "stm32f10x.h"
typedef struct
{
	uint16_t XYChange;	/* X, Y �Ƿ񽻻�  */
	uint8_t Enable;		/* �������ʹ�ܱ�־ */
	uint8_t Event[TOUCH_FIFO_SIZE];	/* �����¼� */
	int16_t XBuf[TOUCH_FIFO_SIZE];	/* �������껺���� */
	int16_t YBuf[TOUCH_FIFO_SIZE];	/* �������껺���� */
	uint8_t Read;					/* ��������ָ�� */
	uint8_t Write;					/* ������дָ�� */
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
/* �����¼� */
enum
{
	TOUCH_NONE = 0,		/* �޴��� */
	TOUCH_DOWN = 1,		/* ���� */
	TOUCH_MOVE = 2,		/* �ƶ� */
	TOUCH_RELEASE = 3	/* �ͷ� */
};

extern TOUCH_T g_tTP;

/* ���ⲿ���õĺ������� */
void TOUCH_InitHard(void);
uint8_t TOUCH_GetKey(int16_t *_pX, int16_t *_pY);
void TOUCH_CelarFIFO(void);
uint8_t TOUCH_InRect(uint16_t _usX, uint16_t _usY,
uint16_t _usRectX, uint16_t _usRectY, uint16_t _usRectH, uint16_t _usRectW);
void TOUCH_PutKey(uint8_t _ucEvent, uint16_t _usX, uint16_t _usY);
void TOUCH_CapScan(void);


#endif

