/*
*********************************************************************************************************
*
*	模块名称 : 电容触摸芯片GT811驱动程序
*	文件名称 : bsp_ct811.c
*	版    本 : V1.0
*	说    明 : GT811触摸芯片驱动程序。
*	修改记录 :
*		版本号   日期        作者     说明
*		V1.0    2014-12-25  armfly   正式发布
*
*	Copyright (C), 2014-2015, 安富莱电子 www.armfly.com
*********************************************************************************************************
*/
#include "bsp_i2c_gpio.h"
#include "GUI.h"
#include "bsp_gt811.h"
#include "bsp_touch.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "main.h"
#include "timer.h"
#include <string.h>
#if 1
	#define printf_gt811dbg printf
#else
	#define printf_gt811dbg(...)
#endif
/*interrupt*/
#define TP_INT_IRQn            EXTI1_IRQn 
#define TP_INT_IRQ_LINE        EXTI_Line1
/*pin*/
#define TP_INT_GPIO GPIOB
#define TP_INT_PIN  GPIO_Pin_1
#define TP_RST_GPIO GPIOB
#define TP_RST_PIN  GPIO_Pin_0
#define TP_RST_SET(state) GPIO_WriteBit(TP_RST_GPIO,TP_RST_PIN,(BitAction)state)
#define TP_INT_SET(state) GPIO_WriteBit(TP_INT_GPIO,TP_INT_PIN,(BitAction)state)
/*reg*/
#define GT911_TOUCH_NUM_REG    0x804C
#define GT911_TOUCH_STATUS_REG 0x814E
#define GT911_TOUCH_INT_MODE   0x804d

struct reg_s
{
	u16 reg_addr;
	u8  reg_value;
};
struct reg_s reg_config[]=
{
	{0x8048,(480&(0x00ff))},//x max output value,low byte
	{0x8049,(480&(0xff00))>>8},//x max output value,high byte
	{0x804a,(320&(0x00ff))},//y max output value,low byte
	{0x804b,(320&(0xff00))>>8},//y max output value,high byte
	{0x804c,0x01},//max out put point
	{0x804d,0x35},//bit3:xy_swap ,bit2:sito ,bit[0~1]:INT mode
	{0x804e,0x00},//
	{0x804f,0x02},//shake_count ,bit[7~4]:realease,bit[3~0]:press
	{0x8050,0x08},//filter
	{0x8051,0x28},//large touch
	{0x8052,0x0a},//noise_reduation
	{0x8053,0x5a},//touch level
	{0x8054,0x46},//leave level
	{0x8055,0x03},//lowpower control
	{0x8056,0x05},//reflesh rate
	{0x8057,0x00},//x threshold
	{0x8058,0x00},//y threshold
	
	
};
struct onepoint_info
{
	u8 track_id;
	u16 x;
	u16 y;
	u16 size; 
};
static void GT811_WriteReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen);
static uint8_t GT811_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen);

GT811_T g_GT811;
//rst int
void gt911_gpio_init(u8 int_mode)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef 	EXTI_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;              
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Pin = TP_RST_PIN ;
  GPIO_Init(TP_RST_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =  TP_INT_PIN ;
	GPIO_Init(TP_INT_GPIO, &GPIO_InitStructure);
	
	if(int_mode != 0)
	{
		GPIO_InitStructure.GPIO_Pin =  TP_INT_PIN ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;              
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_Init(TP_INT_GPIO, &GPIO_InitStructure);
		
		EXTI_InitStructure.EXTI_Line = TP_INT_IRQ_LINE; 
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);
	
	  NVIC_InitStructure.NVIC_IRQChannel = TP_INT_IRQn; 
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; 
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	  NVIC_Init(&NVIC_InitStructure);
	}
}
void EXTI1_IRQHandler(void)
{	
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  if(EXTI_GetFlagStatus(TP_INT_IRQ_LINE)==SET) 
  {
    
		xResult = xEventGroupSetBitsFromISR(human_detect_event_group,EVENT_TP_DETECTED,&xHigherPriorityTaskWoken);
		if(xResult != pdFAIL)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
    EXTI_ClearITPendingBit(TP_INT_IRQ_LINE);
  }
}
void gt911_param_init(void)
{
	u8 i;
	u8 reg_value=0;
	printf("before config param: \r\n");
	//read
	for(i=0;i<sizeof(reg_config)/sizeof(struct reg_s);i++)
	{
		GT811_WriteReg(reg_config[i].reg_addr, &reg_config[i].reg_value, 1);
	}
	//write
	for(i=0;i<sizeof(reg_config)/sizeof(struct reg_s);i++)
	{
		GT811_ReadReg(reg_config[i].reg_addr, &reg_value, 1);
		printf("reg :0x%04x value: %#x \r\n",reg_config[i].reg_addr,reg_value);
	}
	//read
	printf("after config param: \r\n");
	for(i=0;i<sizeof(reg_config)/sizeof(struct reg_s);i++)
	{
		GT811_ReadReg(reg_config[i].reg_addr, &reg_value, 1);
		printf("reg :0x%04x value: %#x \r\n",reg_config[i].reg_addr,reg_value);
	}
}

void tp_power_up(u8 add_sel)
{
	gt911_gpio_init(0);
	delay_ms(20);
	if(add_sel == 0)
	{
		g_GT811.i2c_addr = GT811_I2C_ADDR3;
		TP_RST_SET(0);
	  TP_INT_SET(0);
		delay_us(100);
		TP_INT_SET(1);
		delay_ms(2);
		TP_RST_SET(1);
		delay_ms(10);
		TP_INT_SET(0);
		delay_ms(100);
	}
	else
	{
		g_GT811.i2c_addr = GT811_I2C_ADDR1;
		TP_RST_SET(0);
	  TP_INT_SET(0);
		delay_ms(2);
		TP_INT_SET(1);
		delay_ms(100);
	}
	gt911_gpio_init(1);
	
}
/*
*********************************************************************************************************
*	函 数 名: GT811_InitHard
*	功能说明: 配置触摸芯片.  在调用该函数前，请先执行 bsp_touch.c 中的函数 bsp_DetectLcdType() 识别id
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void GT811_InitHard(void)
{
#if 1

	u8 int_mode = 0 ;
	GT811_ReadReg(GT911_TOUCH_INT_MODE, &int_mode, 1);
	printf("GT911 INT mode : %#x\r\n", int_mode);
	printf("GT811_ProducrID : %#x\r\n", GT811_ProducrID());
	printf("GT811_FirewareVersion : %#x\r\n", GT811_FirewareVersion());
#endif

	g_GT811.TimerCount = 0;
	
	gt911_param_init();
	g_GT811.TimerCount = 0;
	g_GT811.Enable = 1;
}

/*
*********************************************************************************************************
*	函 数 名: GT811_ReadVersion
*	功能说明: 获得GT811的芯片版本
*	形    参: 无
*	返 回 值: 16位版本
*********************************************************************************************************
*/
uint32_t GT811_ProducrID(void)
{
	uint8_t buf;
	u32 id = 0;

	GT811_ReadReg(0x8140, &buf, 1);
  id |= buf;
	GT811_ReadReg(0x8141, &buf, 1);
  id |= buf<<8;
	GT811_ReadReg(0x8142, &buf, 1);
  id |= buf<<16;
	GT811_ReadReg(0x8143, &buf, 1);
  id |= buf<<24;
	return id;
}
uint16_t GT811_FirewareVersion(void)
{
	uint8_t buf;
	u16 ver = 0;

	GT811_ReadReg(0x8144, &buf, 1);
  ver |= buf;
	GT811_ReadReg(0x8145, &buf, 1);
  ver |= buf<<8;
	return ver;
}

/*
*********************************************************************************************************
*	函 数 名: GT811_WriteReg
*	功能说明: 写1个或连续的多个寄存器
*	形    参: _usRegAddr : 寄存器地址
*			  _pRegBuf : 寄存器数据缓冲区
*			 _ucLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
static void GT811_WriteReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	uint8_t i;

    i2c_Start();					/* 总线开始信号 */

    i2c_SendByte(g_GT811.i2c_addr);	/* 发送设备地址+写信号 */
	i2c_WaitAck();

    i2c_SendByte(_usRegAddr >> 8);	/* 地址高8位 */
	i2c_WaitAck();

    i2c_SendByte(_usRegAddr);		/* 地址低8位 */
	i2c_WaitAck();

	for (i = 0; i < _ucLen; i++)
	{
	    i2c_SendByte(_pRegBuf[i]);		/* 寄存器数据 */
		i2c_WaitAck();
	}

    i2c_Stop();                   			/* 总线停止信号 */
}

/*
*********************************************************************************************************
*	函 数 名: GT811_ReadReg
*	功能说明: 读1个或连续的多个寄存器
*	形    参: _usRegAddr : 寄存器地址
*			  _pRegBuf : 寄存器数据缓冲区
*			 _ucLen : 数据长度
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
static uint8_t GT811_ReadReg(uint16_t _usRegAddr, uint8_t *_pRegBuf, uint8_t _ucLen)
{
	int16_t i;
    i2c_Start();					/* 总线开始信号 */
    
    i2c_SendByte(g_GT811.i2c_addr);	/* 发送设备地址+写信号 */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	
	}

    i2c_SendByte(_usRegAddr >> 8);	/* 地址高8位 */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	
	}

    i2c_SendByte(_usRegAddr);		/* 地址低8位 */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	
	}

	i2c_Start();
    i2c_SendByte(g_GT811.i2c_addr + 0x01);	/* 发送设备地址+读信号 */
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	
	}

	for (i = 0; i < _ucLen - 1; i++)
	{
	    _pRegBuf[i] = i2c_ReadByte();	/* 读寄存器数据 */
		i2c_Ack();
	}

	/* 最后一个数据 */
	 _pRegBuf[i] = i2c_ReadByte();		/* 读寄存器数据 */

	i2c_NAck();

    i2c_Stop();							/* 总线停止信号 */
	
	return 1;
	
cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	i2c_Stop();
	
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: GT811_Timer1ms
*	功能说明: 每隔1ms调用1次
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void GT811_Timer1ms(void)
{
	g_GT811.TimerCount++;
}

/*
*********************************************************************************************************
*	函 数 名: GT811_OnePiontScan
*	功能说明: 读取GT811触摸数据，这里仅读取一个触摸点。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
GUI_PID_STATE State;
void GT811_OnePiontScan(void)
{
	static uint8_t s_tp_down = 0;
	u8 reg_value;
	u8 i;
	u8 x,y,size;
	struct onepoint_info point[5];
	memset(point,0,sizeof(point));

	GT811_ReadReg(GT911_TOUCH_STATUS_REG, &reg_value, 1);
	printf("status reg:0x%x value:0x%x \r\n",GT911_TOUCH_STATUS_REG,reg_value);
	if((reg_value&0x80) == 0)
	{
		//printf_gt811dbg("no touch num \r\n");
		return;
	}
	else	
	{
		printf_gt811dbg("touch num:%d \r\n",reg_value&0x0f);
	}

	
		GT811_ReadReg(0x814f,&point[i].track_id,1);
		GT811_ReadReg(0x8150,&x,1);
		point[i].x = x;
		GT811_ReadReg(0x8151,&x,1);
		point[i].x |= x<<8;
		GT811_ReadReg(0x8152,&y,1);
		point[i].y = y;
		GT811_ReadReg(0x8153,&y,1);
		point[i].y |= y<<8;	
		GT811_ReadReg(0x8154,&size,1);
		point[i].size = size;
		GT811_ReadReg(0x8155,&size,1);
		point[i].size |= size<<8;	
		printf("track_id:%x,x:%x,y:%x,size:%x \r\n",point[i].track_id,point[i].x,point[i].y,point[i].size);

	if (s_tp_down == 0)
	{
		s_tp_down = 1;
		State.x = x;
		State.y = y;
		State.Pressed = 1;
		GUI_PID_StoreState(&State);
	}
	else
	{
		State.x = x;
		State.y = y;
		State.Pressed = 1;
		GUI_PID_StoreState(&State);
	}
	reg_value =0 ;
  GT811_WriteReg(GT911_TOUCH_STATUS_REG,&reg_value,1);
	GT811_ReadReg(GT911_TOUCH_STATUS_REG, &reg_value, 1);
	printf("after status reg:0x%x value:0x%x \r\n",GT911_TOUCH_STATUS_REG,reg_value);
#if 0
	printf("%5d,%5d,%3d\r\n",  g_GT811.X0, g_GT811.Y0, g_GT811.P0);
#endif	
}

/*
*********************************************************************************************************
*	函 数 名: GT811_Scan
*	功能说明: 读取GT811触摸数据。读取全部的数据，需要 720us左右。放在 bsp_Idle()中执行
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void GT811_Scan(void)
{
	uint8_t buf[48];
	//uint8_t i;
	static uint8_t s_tp_down = 0;
	uint16_t x, y;
	static uint16_t x_save, y_save;

	if (g_GT811.Enable == 0)
	{
		return;
	}
	
	/* 20ms 执行一次 */
	if (g_GT811.TimerCount < 20)
	{
		return;
	}

	g_GT811.TimerCount = 0;
	
	//GT811_ReadReg(GT811_READ_XY_REG, buf, 1);		
	if ((buf[0] & 0x01) == 0)
	{
		if (s_tp_down == 1)
		{
			s_tp_down = 0;
			TOUCH_PutKey(TOUCH_RELEASE, x_save, y_save);
		}
		return;
	}
					
	//GT811_ReadReg(GT811_READ_XY_REG + 1, &buf[1], 33);
	
	/*
	0x721  R  TouchpointFlag  Sensor_ID  key  tp4  tp3  tp2  tp1  tp0
	0x722  R  Touchkeystate     0  0  0  0  key4  key3  key2  key1

	0x723  R  Point0Xh  触摸点 0，X 坐标高 8 位
	0x724  R  Point0Xl  触摸点 0，X 坐标低 8 位
	0x725  R  Point0Yh  触摸点 0，Y 坐标高 8 位
	0x726  R  Point0Yl  触摸点 0，Y 坐标低 8 位
	0x727  R  Point0Pressure  触摸点 0，触摸压力

	0x728  R  Point1Xh  触摸点 1，X 坐标高 8 位
	0x729  R  Point1Xl  触摸点 1，X 坐标低 8 位
	0x72A  R  Point1Yh  触摸点 1，Y 坐标高 8 位
	0x72B  R  Point1Yl  触摸点 1，Y 坐标低 8 位
	0x72C  R  Point1Pressure  触摸点 1，触摸压力

	0x72D  R  Point2Xh  触摸点 2，X 坐标高 8 位
	0x72E  R  Point2Xl  触摸点 2，X 坐标低 8 位
	0x72F  R  Point2Yh  触摸点 2，Y 坐标高 8 位
	0x730  R  Point2Yl  触摸点 2，Y 坐标低 8 位
	0x731  R  Point2Pressure  触摸点 2，触摸压力

	0x732  R  Point3Xh  触摸点 3，X 坐标高 8 位
	0x733-0x738  R    Reserve  none
	0x739  R  Point3Xl  触摸点 3，X 坐标低 8 位
	0x73A  R  Point3Yh  触摸点 3，Y 坐标高 8 位
	0x73B  R  Point3Yl  触摸点 3，Y 坐标低 8 位
	0x73C  R  Point3Pressure  触摸点 3，触摸压力

	0x73D  R  Point4Xh  触摸点 4，X 坐标高 8 位
	0x73E  R  Point4Xl  触摸点 4，X 坐标低 8 位
	0x73F  R  Point4Yh  触摸点 4，Y 坐标高 8 位
	0x740  R  Point4Yl  触摸点 4，Y 坐标低 8 位
	0x741  R  Point4Pressure  触摸点 4，触摸压力

	0x742  R  Data_check_sum  Data check Sum
	*/

	g_GT811.TouchpointFlag = buf[0];
	g_GT811.Touchkeystate = buf[1];

	g_GT811.X0 = ((uint16_t)buf[2] << 8) + buf[3];
	g_GT811.Y0 = ((uint16_t)buf[4] << 8) + buf[5];
	g_GT811.P0 = buf[6];

	g_GT811.X1 = ((uint16_t)buf[7] << 8) + buf[8];
	g_GT811.Y1 = ((uint16_t)buf[9] << 8) + buf[10];
	g_GT811.P1 = buf[11];

	g_GT811.X2 = ((uint16_t)buf[12] << 8) + buf[13];
	g_GT811.Y2 = ((uint16_t)buf[14] << 8) + buf[15];
	g_GT811.P2 = buf[16];

	/* 触摸点3的地址不连续 */
	g_GT811.X3 = ((uint16_t)buf[17] << 8) + buf[24];
	g_GT811.Y3 = ((uint16_t)buf[25] << 8) + buf[26];
	g_GT811.P3 = buf[27];

	g_GT811.X4 = ((uint16_t)buf[28] << 8) + buf[29];
	g_GT811.Y4 = ((uint16_t)buf[30] << 8) + buf[31];
	g_GT811.P4 = buf[32];

	/* 检测按下 */
	{
		/* 坐标转换 :
			电容触摸板左下角是 (0，0);  右上角是 (479，799)
			需要转到LCD的像素坐标 (左上角是 (0，0), 右下角是 (799，479)
		*/


		/* 必须判断值域 */
		if (g_GT811.i2c_addr == GT811_I2C_ADDR1)
		{
			x = g_GT811.Y0;
			y = 479 - g_GT811.X0;
			
			if (x > 800)
			{
				x = 800;
			}
			
			if (y > 480)
			{
				y = 480;
			}
		}
		else
		{
			x = g_GT811.Y0;
			y = 599 - g_GT811.X0;
		
			if (x > 1024)
			{
				x = 1024;
			}
			
			if (y > 600)
			{
				y = 600;
			}
		}
	}
	
	if (s_tp_down == 0)
	{
		s_tp_down = 1;
		
		TOUCH_PutKey(TOUCH_DOWN, x, y);
	}
	else
	{
		TOUCH_PutKey(TOUCH_MOVE, x, y);
	}
	x_save = x;	/* 保存坐标，用于释放事件 */
	y_save = y;

#if 0
	for (i = 0; i < 34; i++)
	{
		printf("%02X ", buf[i]);
	}
	printf("\r\n");

	printf("(%5d,%5d,%3d) ",  g_GT811.X0, g_GT811.Y0, g_GT811.P0);
	printf("(%5d,%5d,%3d) ",  g_GT811.X1, g_GT811.Y1, g_GT811.P1);
	printf("(%5d,%5d,%3d) ",  g_GT811.X2, g_GT811.Y2, g_GT811.P2);
	printf("(%5d,%5d,%3d) ",  g_GT811.X3, g_GT811.Y3, g_GT811.P3);
	printf("(%5d,%5d,%3d) ",  x, y, g_GT811.P4);
	printf("\r\n");
#endif	
}

/*
*********************************************************************************************************
*	函 数 名: GT811_ReadSensorID
*	功能说明: 识别显示模块类别。读取GT811 SensorID引脚状态，有3个状态，悬空，接电源，接地。
*	形    参: 无
*	返 回 值: 显示模块类别, 0, 1, 2
*********************************************************************************************************
*/
uint8_t GT811_ReadSensorID(void)
{
	uint8_t value;
	
	/* 	0x721  R  TouchpointFlag      Sensor_ID  key  tp4  tp3  tp2  tp1  tp0 */
	GT811_ReadReg(0x721, &value, 1);
	
	return (value >> 6);
}


/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
