#include "adc.h"
#include "timer.h"
#include "main.h"
#include <stdio.h>
u16 adc_value = 0;		

void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA1 ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

  ADC_TempSensorVrefintCmd(ENABLE); //internal channel open
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����
	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
 
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������

}				  
//���ADCֵ
//ch:ͨ��ֵ 0~3
static u16 Get_Adc(u8 ch)   
{
  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}

static u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		vTaskDelay(5/ portTICK_RATE_MS);
	}
	return temp_val/times;
} 	 

u16 get_adc_mv(u8 ch)
{
	u16 Vrefint = 1200;//use interal voltage 1.2v
	u16 ADrefint;
	u16 ADchx;
	ADrefint = Get_Adc_Average(ADC_Channel_Vrefint,3);
	ADchx =  Get_Adc_Average(ch,3);
	//printf("ADrefint:%d,ADchx:%d \r\n",ADrefint,ADchx);
	return (Vrefint * ADchx/ADrefint);
}

void adc_task(void *pvParameters)
{
  u16 pre_value;
	u16 now_value;
  while (1) 
	{
		now_value = get_adc_mv(BATT_CHANNEL);
		//printf("now_value:%d \r\n",now_value);
		
		if(now_value<=MIN_BATT_VOLTAGE_MV)
		{
			//printf("power is below %d mv!\r\n",MIN_BATT_VOLTAGE_MV*4);
			//adc_value = MIN_BATT_VOLTAGE_MV; //0%
			adc_value = 2*(MAX_BATT_VOLTAGE_MV-MIN_BATT_VOLTAGE_MV)/100 + MIN_BATT_VOLTAGE_MV;//2%
			if(xQueueSend(public_queque, (void *)"batt_changed", 0) == pdPASS){}
		}
		else if(now_value>=MAX_BATT_VOLTAGE_MV)
		{
			//printf("power is above %d mv!\r\n",MAX_BATT_VOLTAGE_MV*4);
			adc_value = MAX_BATT_VOLTAGE_MV;
			if(xQueueSend(public_queque, (void *)"batt_changed", 0) == pdPASS){}
		}
		else
		{
			adc_value = now_value;
		}
		
		
		if(((now_value - pre_value)>10) || (pre_value - now_value)>10)
		{
			if(xQueueSend(public_queque, (void *)"batt_changed", 0) == pdPASS){}
			pre_value = now_value;
		}
		
		vTaskDelay(5000 / portTICK_RATE_MS);
		
	}
}

