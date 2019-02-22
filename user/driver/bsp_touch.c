#include "bsp_gt911.h"
#include "GUI.h"
#include "bsp_i2c_gpio.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "bsp_touch.h"
#include "lcd.h"
#include "LCD_ILI9488.h"
#include "timer.h"

/* TSC2046 �ڲ�ADCͨ���� */
#define ADC_CH_X	1		/* Xͨ��������Xλ�� */
#define ADC_CH_Y	5		/* Yͨ��������Yλ�� */

/* ÿ1msɨ��һ������ */
#define DOWN_VALID		30	/* ����30ms ��, ��ʼͳ��ADC */
#define SAMPLE_COUNT	10	/* ���º������ɼ�40������ */

/*
������У׼�������Ļ�����Ľǵ�ƫ������
��1���� �� x1 = CALIB_OFFSET, y1 = CALIB_OFFSET
��2���� �� x2 = LCD_GetWidth() - CALIB_OFFSET, y2 = LCD_GetHeight() - CALIB_OFFSET
*/
#define CALIB_OFFSET	20
#define TP_X1	CALIB_OFFSET
#define TP_Y1	CALIB_OFFSET

#define TP_X2	(LCD_GetWidth() - CALIB_OFFSET)
#define TP_Y2	(LCD_GetHeight() - CALIB_OFFSET)

#define TP_X3	CALIB_OFFSET
#define TP_Y3	(LCD_GetHeight() - CALIB_OFFSET)

#define TP_X4	(LCD_GetWidth() - CALIB_OFFSET)
#define TP_Y4	CALIB_OFFSET

/* ��ЧADCֵ���ж�����. ̫�ӽ�ADC�ٽ�ֵ��������Ϊ��Ч */
#define ADC_VALID_OFFSET	2

//#define WaitTPReady() while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
#define WaitTPReady() {}

/* ����ģ���õ���ȫ�ֱ��� */
TOUCH_T g_tTP;
PARAM_T g_tParam;

void TOUCH_InitHard(void)
{

    g_tTP.Enable = 0;
	  u8 i;
		/* FTϵ�е��ݴ������� : 4.3��id = 0x55    5.0��id = 0x0A  7.0��id = 0x06 */
	for (i = 0; i < 5; i++)
	{
		if (i2c_CheckDevice(GT911_I2C_ADDR) == 0)
		{
			g_GT911.i2c_addr = GT911_I2C_ADDR;
			/*
			ϵͳ�ϵ��Ҫ����һ��ʱ����ӳ٣���ΪFTϵ�е��ݴ���оƬ�ϵ�����ֱ�Ӽ���оƬ�����Ƕ�
			ȡ��ID���ǲ��ԣ���Ҫ�ӳ�һ��ʱ����ٶ�ȡ������ȷ�ġ�
			*/
			delay_ms(100);
			GT911_InitHard();
			break;
		}	
		if (i2c_CheckDevice(GT911_I2C_ADDR2) == 0)
		{
			g_GT911.i2c_addr = GT911_I2C_ADDR2;
			/*
			ϵͳ�ϵ��Ҫ����һ��ʱ����ӳ٣���ΪFTϵ�е��ݴ���оƬ�ϵ�����ֱ�Ӽ���оƬ�����Ƕ�
			ȡ��ID���ǲ��ԣ���Ҫ�ӳ�һ��ʱ����ٶ�ȡ������ȷ�ġ�
			*/
			delay_ms(100);
			GT911_InitHard();
			break;
		}	
	}
	  g_tTP.Enable = 1;
		return;
}


/*
*********************************************************************************************************
*	�� �� ��: TOUCH_PutKey
*	����˵��: ��1������������ֵѹ�봥��FIFO�����������ڵ��败������
*	��    ��: _usX, _usY ����ֵ
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TOUCH_PutKey(uint8_t _ucEvent, uint16_t _usX, uint16_t _usY)
{
	uint16_t xx, yy;
	uint16_t x = 0, y = 0;

	g_tTP.Event[g_tTP.Write] = _ucEvent;

	
  if (g_GT911.Enable == 1)
	{
		xx = _usX;
		yy = _usY;
	}	
	
	/* ��������������ʶ�� */
	if (g_tParam.TouchDirection > 3)
	{
		g_tParam.TouchDirection  = 0;
	}
	switch (g_tParam.TouchDirection)
	{
		case 0:	/* У׼����ʱ����Ļ����Ϊ0 */
			if (g_LcdDirection == 0)		/* ���� */
			{
				x = xx;
				y = yy;
			}
			else if (g_LcdDirection == 1)	/* ����180��*/
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			else if (g_LcdDirection == 2)	/* ���� */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			else if (g_LcdDirection == 3)	/* ����180�� */
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			break;

		case 1:	/* У׼����ʱ����Ļ����Ϊ1 */
			if (g_LcdDirection == 0)		/* ���� */
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			else if (g_LcdDirection == 1)	/* ����180��*/
			{
				x = xx;
				y = yy;
			}
			else if (g_LcdDirection == 2)	/* ���� */
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			else if (g_LcdDirection == 3)	/* ����180�� */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			break;

		case 2:	/* У׼����ʱ����Ļ����Ϊ2 */
			if (g_LcdDirection == 0)		/* ���� */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			else if (g_LcdDirection == 1)	/* ����180��*/
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			else if (g_LcdDirection == 2)	/* ���� */
			{
				x = xx;
				y = yy;
			}
			else if (g_LcdDirection == 3)	/* ����180�� */
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			break;

		case 3:	/* У׼����ʱ����Ļ����Ϊ3 */
			if (g_LcdDirection == 0)		/* ���� */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			else if (g_LcdDirection == 1)	/* ����180��*/
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			else if (g_LcdDirection == 2)	/* ���� */
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			else if (g_LcdDirection == 3)	/* ����180�� */
			{
				x = xx;
				y = yy;
			}
			break;

		default:
			g_tParam.TouchDirection = 0;	/* ���������Чʱ������Ϊȱʡ�ĺ��� */
			break;
	}

	g_tTP.XBuf[g_tTP.Write] = x;
	g_tTP.YBuf[g_tTP.Write] = y;

	if (++g_tTP.Write  >= TOUCH_FIFO_SIZE)
	{
		g_tTP.Write = 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: TOUCH_GetKey
*	����˵��: �Ӵ���FIFO��������ȡһ������ֵ��
*	��    ��:  ��
*	�� �� ֵ:
*			TOUCH_NONE      ��ʾ���¼�
*			TOUCH_DOWN      ����
*			TOUCH_MOVE      �ƶ�
*			TOUCH_RELEASE	�ͷ�
*********************************************************************************************************
*/
uint8_t TOUCH_GetKey(int16_t *_pX, int16_t *_pY)
{
	uint8_t ret;

	if (g_tTP.Read == g_tTP.Write)
	{
		return TOUCH_NONE;
	}
	else
	{
		ret = g_tTP.Event[g_tTP.Read];
		*_pX = g_tTP.XBuf[g_tTP.Read];
		*_pY = g_tTP.YBuf[g_tTP.Read];

		if (++g_tTP.Read >= TOUCH_FIFO_SIZE)
		{
			g_tTP.Read = 0;
		}
		return ret;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: TOUCH_CelarFIFO
*	����˵��: �������FIFO������
*	��    ��:  ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TOUCH_CelarFIFO(void)
{
	__set_PRIMASK(1);  		/* ���ж� */
	g_tTP.Write = g_tTP.Read;
	__set_PRIMASK(0);  		/* ���ж� */
}

/*
*********************************************************************************************************
*	�� �� ��: TOUCH_InRect
*	����˵��: �жϵ�ǰ�����Ƿ�λ�ھ��ο���
*	��    ��:  _usX, _usY: ��������
*			_usRectX,_usRectY: �������
*			_usRectH��_usRectW : ���θ߶ȺͿ��
*	�� �� ֵ: 1 ��ʾ�ڷ�Χ��
*********************************************************************************************************
*/
uint8_t TOUCH_InRect(uint16_t _usX, uint16_t _usY,
	uint16_t _usRectX, uint16_t _usRectY, uint16_t _usRectH, uint16_t _usRectW)
{
	if ((_usX > _usRectX) && (_usX < _usRectX + _usRectW)
		&& (_usY > _usRectY) && (_usY < _usRectY + _usRectH))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


/*
*********************************************************************************************************
*	�� �� ��: TOUCH_CapScan
*	����˵��: I2C�ӿڵ��ݴ�����ɨ�躯�������� bsp_Idle()ִ�У�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void TOUCH_CapScan(void)
{
	
	
	if (g_GT911.Enable == 1)
	{
		GT911_Scan();
		return;
	}
}




