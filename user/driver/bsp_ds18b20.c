/*
*********************************************************************************************************
*
*	ģ������ : DS18B20 ����ģ��(1-wire �����¶ȴ�������
*	�ļ����� : bsp_ds18b20.c
*	��    �� : V1.0
*	˵    �� : DS18B20��CPU֮�����1��GPIO�ӿڡ�
*
*	�޸ļ�¼ :
*		�汾��  ����         ����     ˵��
*		V1.0    2014-01-24  armfly  ��ʽ����
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp_ds18b20.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timer.h"
#include "timers.h"
#include "main.h"
#include <stdio.h>
/*
	DS18B20 ����ֱ�Ӳ鵽STM32-V5�������U16 (3P) ����.

    DS18B20     STM32F407������
	  VCC   ------  3.3V
	  DQ    ------  PB1   (���������� 4.7K ��������)
	  GND   ------  GND
*/

/* ����GPIO�˿� */
#define RCC_DQ		RCC_AHB1Periph_GPIOE
#define PORT_DQ		GPIOE
#define PIN_DQ		GPIO_Pin_0

#if 1 /* �⺯����ʽ */
	#define DQ_0()		GPIO_ResetBits(PORT_DQ, PIN_DQ)
	#define DQ_1()		GPIO_SetBits(PORT_DQ, PIN_DQ)

	/* �ж�DQ�����Ƿ�Ϊ�� */
	#define DQ_IS_LOW()	(GPIO_ReadInputDataBit(PORT_DQ, PIN_DQ) == Bit_RESET)
#else	/* ֱ�Ӳ����Ĵ���������ٶ� */
	#define DQ_0()		PORT_DQ->BSRRH = PIN_DQ
	#define DQ_1()		PORT_DQ->BSRRL = PIN_DQ

	/* �ж�DQ�����Ƿ�Ϊ�� */
	#define DQ_IS_LOW()	((PORT_DQ->IDR & PIN_DQ) == 0)
#endif

u16 tempratrue_now;
xTimerHandle xTimer_tempratrue_measure_timer =NULL;
/*
*********************************************************************************************************
*	�� �� ��: bsp_InitDS18B20
*	����˵��: ����STM32��GPIO��SPI�ӿڣ��������� DS18B20
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitDS18B20(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;


	DQ_1();

	/* ����DQΪ��©��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;		/* ��Ϊ����� */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/* IO������ٶ� */

	GPIO_InitStructure.GPIO_Pin = PIN_DQ;
	GPIO_Init(PORT_DQ, &GPIO_InitStructure);
	
}

/*
*********************************************************************************************************
*	�� �� ��: DS18B20_Reset
*	����˵��: ��λDS18B20�� ����DQΪ�ͣ���������480us��Ȼ��ȴ�
*	��    ��: ��
*	�� �� ֵ: 0 ʧ�ܣ� 1 ��ʾ�ɹ�
*********************************************************************************************************
*/
uint8_t DS18B20_Reset(void)
{
	/*
		��λʱ��, ��DS18B20 page 15

		������������DQ���������� 480us
		Ȼ���ͷ�DQ���ȴ�DQ�������������ߣ�Լ 15-60us
		DS18B20 ������DQΪ�� 60-240us�� ����źŽ� presence pulse  (��λ����,��ʾDS18B20׼������ ���Խ�������)
		���������⵽�����Ӧ���źţ���ʾDS18B20��λ�ɹ�
	*/

	uint8_t i;
	uint16_t k;

	taskENTER_CRITICAL();/* ��ֹȫ���ж� */

	/* ��λ�����ʧ���򷵻�0 */
	for (i = 0; i < 1; i++)
	{
		DQ_0();				/* ����DQ */
		delay_us(520);	/* �ӳ� 520uS�� Ҫ������ӳٴ��� 480us */
		DQ_1();				/* �ͷ�DQ */

		delay_us(15);	/* �ȴ�15us */

		/* ���DQ��ƽ�Ƿ�Ϊ�� */
		for (k = 0; k < 10; k++)
		{
			if (DQ_IS_LOW())
			{
				break;
			}
			delay_us(10);	/* �ȴ�65us */
		}
		if (k >= 10)
		{
			continue;		/* ʧ�� */
		}

		/* �ȴ�DS18B20�ͷ�DQ */
		for (k = 0; k < 30; k++)
		{
			if (!DQ_IS_LOW())
			{
				break;
			}
			delay_us(10);	/* �ȴ�65us */
		}
		if (k >= 30)
		{
			continue;		/* ʧ�� */
		}

		break;
	}

	taskEXIT_CRITICAL();	/* ʹ��ȫ���ж� */

	delay_us(5);

	if (i >= 1)
	{
		return 0;
	}

	return 1;
}

/*
*********************************************************************************************************
*	�� �� ��: DS18B20_WriteByte
*	����˵��: ��DS18B20д��1�ֽ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void DS18B20_WriteByte(uint8_t _val)
{
	/*
		д����ʱ��, ��DS18B20 page 16
	*/
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		DQ_0();
		delay_us(2);

		if (_val & 0x01)
		{
			DQ_1();
		}
		else
		{
			DQ_0();
		}
		delay_us(60);
		DQ_1();
		delay_us(2);
		_val >>= 1;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: DS18B20_ReadByte
*	����˵��: ��DS18B20��ȡ1�ֽ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static uint8_t DS18B20_ReadByte(void)
{
	/*
		д����ʱ��, ��DS18B20 page 16
	*/
	uint8_t i;
	uint8_t read = 0;

	for (i = 0; i < 8; i++)
	{
		read >>= 1;

		DQ_0();
		delay_us(3);
		DQ_1();
		delay_us(3);

		if (DQ_IS_LOW())
		{
			;
		}
		else
		{
			read |= 0x80;
		}
		delay_us(60);
	}

	return read;
}

/*
*********************************************************************************************************
*	�� �� ��: DS18B20_ReadTempReg
*	����˵��: ���¶ȼĴ�����ֵ��ԭʼ���ݣ�
*	��    ��: ��
*	�� �� ֵ: �¶ȼĴ������� ������16�õ� 1���϶ȵ�λ, Ҳ����С����ǰ�������)
*********************************************************************************************************
*/
int16_t DS18B20_ReadTempReg(void)
{
	uint8_t temp1, temp2;

	/* ���߸�λ */
	if (DS18B20_Reset() == 0)
	{
		return 0;
	}		

	DS18B20_WriteByte(0xcc);	/* ������ */
	DS18B20_WriteByte(0x44);	/* ��ת������ */

	DS18B20_Reset();		/* ���߸�λ */

	DS18B20_WriteByte(0xcc);	/* ������ */
	DS18B20_WriteByte(0xbe);

	temp1 = DS18B20_ReadByte();	/* ���¶�ֵ���ֽ� */
	temp2 = DS18B20_ReadByte();	/* ���¶�ֵ���ֽ� */

	return ((temp2 << 8) | temp1);	/* ����16λ�Ĵ���ֵ */
}

/*
*********************************************************************************************************
*	�� �� ��: DS18B20_ReadID
*	����˵��: ��DS18B20��ROM ID�� �����ϱ���ֻ��1��оƬ
*	��    ��: _id �洢ID
*	�� �� ֵ: 0 ��ʾʧ�ܣ� 1��ʾ��⵽��ȷID
*********************************************************************************************************
*/
uint8_t DS18B20_ReadID(uint8_t *_id)
{
	uint8_t i;

	/* ���߸�λ */
	if (DS18B20_Reset() == 0)
	{
		return 0;
	}

	DS18B20_WriteByte(0x33);	/* ������ */
	for (i = 0; i < 8; i++)
	{
		_id[i] = DS18B20_ReadByte();
	}

	DS18B20_Reset();		/* ���߸�λ */
	
	return 1;
}

/*
*********************************************************************************************************
*	�� �� ��: DS18B20_ReadTempByID
*	����˵��: ��ָ��ID���¶ȼĴ�����ֵ��ԭʼ���ݣ�
*	��    ��: ��
*	�� �� ֵ: �¶ȼĴ������� ������16�õ� 1���϶ȵ�λ, Ҳ����С����ǰ�������)
*********************************************************************************************************
*/
int16_t DS18B20_ReadTempByID(uint8_t *_id)
{
	uint8_t temp1, temp2;
	uint8_t i;

	DS18B20_Reset();		/* ���߸�λ */

	DS18B20_WriteByte(0x55);	/* ������ */
	for (i = 0; i < 8; i++)
	{
		DS18B20_WriteByte(_id[i]);
	}
	DS18B20_WriteByte(0x44);	/* ��ת������ */

	DS18B20_Reset();		/* ���߸�λ */

	DS18B20_WriteByte(0x55);	/* ������ */
	for (i = 0; i < 8; i++)
	{
		DS18B20_WriteByte(_id[i]);
	}	
	DS18B20_WriteByte(0xbe);

	temp1 = DS18B20_ReadByte();	/* ���¶�ֵ���ֽ� */
	temp2 = DS18B20_ReadByte();	/* ���¶�ֵ���ֽ� */

	return ((temp2 << 8) | temp1);	/* ����16λ�Ĵ���ֵ */
}
void tempratrue_measure_callback( xTimerHandle  xTimer )
{
	static u16 temp;
	temp = DS18B20_ReadTempReg();
	if(tempratrue_now != temp)//(  ((temp - tempratrue_now) > 4) || ((tempratrue_now - temp) > 4) )  
	{
		tempratrue_now = temp;
		if(xQueueSend(public_queque, (void *)"temp_changed", 0) == pdPASS){}
	}
}
void tempratrue_measure_timer_init(void)
{
	if(xTimer_tempratrue_measure_timer == NULL)
        xTimer_tempratrue_measure_timer = xTimerCreate("xTimer_tempratrue_measure_timer ",  (1*1000)/portTICK_RATE_MS, pdTRUE, ( void * ) 0, tempratrue_measure_callback);
	if(xTimer_tempratrue_measure_timer != NULL)
				xTimerStart( xTimer_tempratrue_measure_timer, 0 );
}
void ds18b20_init(void)
{
	bsp_InitDS18B20();
	tempratrue_measure_timer_init();
	tempratrue_now = DS18B20_ReadTempReg();
	if(xQueueSend(public_queque, (void *)"temp_changed", 0) == pdPASS){}
}
/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
