/*
*********************************************************************************************************
*
*	模块名称 : 电阻式触摸板驱动模块
*	文件名称 : bsp_touch.c
*	版    本 : V1.8
*	说    明 : 驱动TS2046芯片 和 RA8875内置触摸，支持I2C接口电容触摸
*	修改记录 :
*		版本号  日期        作者    说明
*       v1.0    2012-07-06 armfly  ST固件库V3.5.0版本。
*		v1.1    2012-10-22 armfly  增加4点校准
*		v1.2    2012-11-07 armfly  解决4点校准的XY交换分支的bug
*		v1.3    2012-12-17 armfly  触摸校准函数增加入口参数:等待时间
*		V1.4    2013-07-26 armfly  更改 TOUCH_DataFilter() 滤波算法
*		V1.5    2013-07-32 armfly  修改TOUCH_WaitRelease(),计数器需要清零
*		V1.6    2014-10-20 armfly
*					(1) 修改 TOUCH_PutKey() 函数，实现触摸屏的横屏和竖屏动态切换.
*					(2) param 结构增加校准时当前的屏幕方向变量 TouchDirection
*					(3) 调试3.5寸的触摸芯片。修改SPI相关配置函数。
*					(4) 由于触摸芯片TSC2046和串行FLASH,NRF24L01,MP3等模块共享SPI总线。因此需要
*						在触摸中断函数中判断总线冲突. 增加函数 bsp_SpiBusBusy() 判忙.
*					(5) TSC2046增加软件模拟SPI (软件模拟方式方便SPI设备共享)
*		V1.7    2015-01-02 armfly  计划将触摸扫描由1ms一次修改为10ms一次。未定。
*				2015-04-21 armfly 修改 TOUCH_InitHard() 函数。GT811_InitHard() 执行后直接return
*		V1.8	2015-10-30 armfly 增加 4.3寸电容触摸 FT5x06
*					(1) 添加 void TOUCH_CapScan(void) 函数
*					(2) 修改 TOUCH_PutKey() 函数
*					(3) g_tTP.Write = g_tTP.Read = 0; 移到电容触摸处理函数之前做，避免电容屏首个触点被忽略。
*
*	Copyright (C), 2015-2020, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp_ts_ft5x06.h"
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

/*
【1】安富莱STM32-V5开发板 + 3.5寸显示模块， 显示模块上的触摸芯片为 TSC2046或其兼容芯片
	PI10   --> TP_CS
	PD3/LCD_BUSY   --> TP_BUSY
	PB3/SPI3_SCK   --> TP_SCK
	PB4/SPI3_MISO   --> TP_MISO
	PB5/SPI3_MOSI   --> TP_MOSI
	PI3/TP_INT   --> TP_PEN_INT

【2】安富莱STM32开发板 + 4.3寸或7寸显示模块（内置RA8875芯片)
	RA8875内置触摸屏接口，因此直接通过FSMC访问RA8875相关寄存器即可。


	本程序未使用触笔中断功能。在1ms周期的 Systick定时中断服务程序中对触摸ADC值进行采样和
	滤波处理。当触笔按下超过40ms后，开始采集ADC值（每1ms采集1次，连续采集10次），然后对
	这10个样本进行排序，丢弃2头的样本，对中间一段样本进行求和并计算平均值。

	采样2点校准方法，校准后保存2个校准点的ADC值，实际工作时，根据2点直线方程计算屏幕坐标。
	校准参数有保存接口，本程序主要用于演示，未做保存功能。
	大家可以自己修改  TOUCH_SaveParam() 和 TOUCH_LoadParam() 两个函数实现保存功能。

*/
/* TSC2046 内部ADC通道号 */
#define ADC_CH_X	1		/* X通道，测量X位置 */
#define ADC_CH_Y	5		/* Y通道，测量Y位置 */

/* 每1ms扫描一次坐标 */
#define DOWN_VALID		30	/* 按下30ms 后, 开始统计ADC */
#define SAMPLE_COUNT	10	/* 按下后连续采集40个样本 */

/*
触摸屏校准点相对屏幕像素四角的偏移像素
第1个点 ： x1 = CALIB_OFFSET, y1 = CALIB_OFFSET
第2个点 ： x2 = LCD_GetWidth() - CALIB_OFFSET, y2 = LCD_GetHeight() - CALIB_OFFSET
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

/* 有效ADC值的判断门限. 太接近ADC临界值的坐标认为无效 */
#define ADC_VALID_OFFSET	2

//#define WaitTPReady() while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
#define WaitTPReady() {}

/* 触屏模块用到的全局变量 */
TOUCH_T g_tTP;
PARAM_T g_tParam;

/*
*********************************************************************************************************
*	函 数 名: bsp_InitTouch
*	功能说明: 配置STM32和触摸相关的口线，使能硬件SPI1, 片选由软件控制
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void TOUCH_InitHard(void)
{
    g_tTP.Enable = 0;
	
		/* FT系列电容触摸触摸 : 4.3寸id = 0x55    5.0寸id = 0x0A  7.0寸id = 0x06 */
		if (i2c_CheckDevice(FT5X06_I2C_ADDR) == 0)
		{
			uint8_t id;
			/*
			系统上电后要先做一段时间的延迟，因为FT系列电容触摸芯片上电后可以直接检查出芯片，但是读
			取的ID号是不对，需要延迟一段时间后再读取才是正确的。
			*/
			delay_ms(100);
			id = FT5X06_ReadID();
			printf("tp id:0x%02x \r\n",id);
			FT5X06_InitHard();
		}	
	  g_tTP.Enable = 1;
		return;
}


/*
*********************************************************************************************************
*	函 数 名: TOUCH_PutKey
*	功能说明: 将1个触摸点坐标值压入触摸FIFO缓冲区。用于电阻触摸屏。
*	形    参: _usX, _usY 坐标值
*	返 回 值: 无
*********************************************************************************************************
*/
void TOUCH_PutKey(uint8_t _ucEvent, uint16_t _usX, uint16_t _usY)
{
	uint16_t xx, yy;
	uint16_t x = 0, y = 0;

	g_tTP.Event[g_tTP.Write] = _ucEvent;

	
  if (g_tFT5X06.Enable == 1)
	{
		xx = _usX;
		yy = _usY;
	}	
	
	/* 横屏和竖屏方向识别 */
	if (g_tParam.TouchDirection > 3)
	{
		g_tParam.TouchDirection  = 0;
	}
	switch (g_tParam.TouchDirection)
	{
		case 0:	/* 校准触摸时，屏幕方向为0 */
			if (g_LcdDirection == 0)		/* 横屏 */
			{
				x = xx;
				y = yy;
			}
			else if (g_LcdDirection == 1)	/* 横屏180°*/
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			else if (g_LcdDirection == 2)	/* 竖屏 */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			else if (g_LcdDirection == 3)	/* 竖屏180° */
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			break;

		case 1:	/* 校准触摸时，屏幕方向为1 */
			if (g_LcdDirection == 0)		/* 横屏 */
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			else if (g_LcdDirection == 1)	/* 横屏180°*/
			{
				x = xx;
				y = yy;
			}
			else if (g_LcdDirection == 2)	/* 竖屏 */
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			else if (g_LcdDirection == 3)	/* 竖屏180° */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			break;

		case 2:	/* 校准触摸时，屏幕方向为2 */
			if (g_LcdDirection == 0)		/* 横屏 */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			else if (g_LcdDirection == 1)	/* 横屏180°*/
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			else if (g_LcdDirection == 2)	/* 竖屏 */
			{
				x = xx;
				y = yy;
			}
			else if (g_LcdDirection == 3)	/* 竖屏180° */
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			break;

		case 3:	/* 校准触摸时，屏幕方向为3 */
			if (g_LcdDirection == 0)		/* 横屏 */
			{
				y = xx;
				x = g_LcdWidth - yy - 1;
			}
			else if (g_LcdDirection == 1)	/* 横屏180°*/
			{
				y = g_LcdHeight - xx - 1;
				x = yy;
			}
			else if (g_LcdDirection == 2)	/* 竖屏 */
			{
				x = g_LcdWidth - xx - 1;
				y = g_LcdHeight - yy - 1;
			}
			else if (g_LcdDirection == 3)	/* 竖屏180° */
			{
				x = xx;
				y = yy;
			}
			break;

		default:
			g_tParam.TouchDirection = 0;	/* 方向参数无效时，纠正为缺省的横屏 */
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
*	函 数 名: TOUCH_GetKey
*	功能说明: 从触摸FIFO缓冲区读取一个坐标值。
*	形    参:  无
*	返 回 值:
*			TOUCH_NONE      表示无事件
*			TOUCH_DOWN      按下
*			TOUCH_MOVE      移动
*			TOUCH_RELEASE	释放
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
*	函 数 名: TOUCH_CelarFIFO
*	功能说明: 清除触摸FIFO缓冲区
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void TOUCH_CelarFIFO(void)
{
	__set_PRIMASK(1);  		/* 关中断 */
	g_tTP.Write = g_tTP.Read;
	__set_PRIMASK(0);  		/* 开中断 */
}

/*
*********************************************************************************************************
*	函 数 名: TOUCH_InRect
*	功能说明: 判断当前坐标是否位于矩形框内
*	形    参:  _usX, _usY: 输入坐标
*			_usRectX,_usRectY: 矩形起点
*			_usRectH、_usRectW : 矩形高度和宽度
*	返 回 值: 1 表示在范围内
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
*	函 数 名: TOUCH_CapScan
*	功能说明: I2C接口电容触摸板扫描函数，放在 bsp_Idle()执行！
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void TOUCH_CapScan(void)
{
	
	
	if (g_tFT5X06.Enable == 1)
	{
		FT5X06_Scan();
		return;
	}
}



/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
