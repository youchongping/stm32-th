#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
#define BATT_CHANNEL ADC_Channel_10
#define MIN_BATT_VOLTAGE_MV (850)
#define MAX_BATT_VOLTAGE_MV (1030)
#define PERCENT(x) ((x - MIN_BATT_VOLTAGE_MV)*100 /(MAX_BATT_VOLTAGE_MV - MIN_BATT_VOLTAGE_MV))

extern 	u16 adc_value;

void Adc_Init(void);
void adc_task(void *pvParameters);

#endif 
