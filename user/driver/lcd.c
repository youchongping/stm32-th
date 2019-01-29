
//#include <intrins.h>
//#include <absacc.h>

#include "lcd.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "LCD_ILI9488.h"
static void LCD_WR_REG(u16 Data) {
  // ... TBD by user
	LCD_REG16 = Data;
}

u16 LCD_ReadReg(u16 LCD_Reg)
{										   
	LCD_WR_REG(LCD_Reg);		
	vTaskDelay(1/portTICK_RATE_MS);		  
	return LCD_DAT16;		
} 
u16 LCD_ReadData(void)
{
	vu16 ram;			
	ram=LCD_DAT16;	
	return ram;	 
}
void LCD_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //DB0~15 init
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;              
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_0 |GPIO_Pin_1 ;
    GPIO_Init(LCD_DB_0_3, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(LCD_DB_13_15, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 |  GPIO_Pin_8 |  GPIO_Pin_9 |  GPIO_Pin_10 |  GPIO_Pin_11 |  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
    GPIO_Init(LCD_DB_4_12, &GPIO_InitStructure);
    // cs rs rd wr 
	  GPIO_InitStructure.GPIO_Pin = LCD_CS_PIN | LCD_RS_PIN | LCD_RD_PIN | LCD_WR_PIN  ;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		//RST
		GPIO_InitStructure.GPIO_Pin = LCD_RST_PIN  ;
    GPIO_Init(LCD_RST_GPIO, &GPIO_InitStructure);
		//IM[2:0] = 010
		GPIO_InitStructure.GPIO_Pin = LCD_IM0_PIN  ;
    GPIO_Init(LCD_IM0_GPIO, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = LCD_IM1_PIN  ;
    GPIO_Init(LCD_IM1_GPIO, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = LCD_IM2_PIN  ;
    GPIO_Init(LCD_IM2_GPIO, &GPIO_InitStructure);
		GPIO_WriteBit(LCD_IM0_GPIO,LCD_IM0_PIN,Bit_RESET);
		GPIO_WriteBit(LCD_IM1_GPIO,LCD_IM1_PIN,Bit_SET);
		GPIO_WriteBit(LCD_IM2_GPIO,LCD_IM2_PIN,Bit_RESET);
		//LCD_BL
		GPIO_InitStructure.GPIO_Pin = LCD_BL_PIN  ;
    GPIO_Init(LCD_BL_GPIO, &GPIO_InitStructure);
		GPIO_WriteBit(LCD_BL_GPIO,LCD_BL_PIN,Bit_SET);

		

}
void LCD_FMSC_Init()
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef readWriteTiming;
    FSMC_NORSRAMTimingInitTypeDef writeTiming;  
	
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC,ENABLE); 
	
    readWriteTiming.FSMC_AddressSetupTime = 0x01;  //��ַ����ʱ�䣨ADDSET��Ϊ2��HCLK 1/36M=27ns
    readWriteTiming.FSMC_AddressHoldTime = 0x00;  //��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�
    readWriteTiming.FSMC_DataSetupTime = 0x0f;  // ���ݱ���ʱ��Ϊ16��HCLK,��ΪҺ������IC�Ķ����ݵ�ʱ���ٶȲ���̫�죬�����1289���IC��
    readWriteTiming.FSMC_BusTurnAroundDuration = 0x01;
    readWriteTiming.FSMC_CLKDivision = 0x00;
    readWriteTiming.FSMC_DataLatency = 0x00;
    readWriteTiming.FSMC_AccessMode = FSMC_AccessMode_A;  //ģʽA 
    

    writeTiming.FSMC_AddressSetupTime = 0x01;  //��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK  
    writeTiming.FSMC_AddressHoldTime = 0x00;  //��ַ����ʱ�䣨A
    writeTiming.FSMC_DataSetupTime = 0x08;  ////���ݱ���ʱ��Ϊ4��HCLK
    writeTiming.FSMC_BusTurnAroundDuration = 0x01;
    writeTiming.FSMC_CLKDivision = 0x00;
    writeTiming.FSMC_DataLatency = 0x00;
    writeTiming.FSMC_AccessMode = FSMC_AccessMode_A;  //ģʽA 

 
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;//  ��������ʹ��NE1 ��Ҳ�Ͷ�ӦBTCR[6],[7]��
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; // ���������ݵ�ַ
    FSMC_NORSRAMInitStructure.FSMC_MemoryType =FSMC_MemoryType_SRAM;// FSMC_MemoryType_SRAM;  //SRAM   
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;//�洢�����ݿ��Ϊ16bit   
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode =FSMC_BurstAccessMode_Disable;// FSMC_BurstAccessMode_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait=FSMC_AsynchronousWait_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;  
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable; //  �洢��дʹ��
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;   
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable; // ��дʹ�ò�ͬ��ʱ��
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable; 
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming; //��дʱ��
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &writeTiming;  //дʱ��

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);  //��ʼ��FSMC����

    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  // ʹ��BANK1
}

void LCD_Reset()
{
		LCD_RESET=1;
		vTaskDelay(1 / portTICK_RATE_MS);
		LCD_RESET=0;
		vTaskDelay(10 / portTICK_RATE_MS);
		LCD_RESET=1;
		vTaskDelay(120 / portTICK_RATE_MS);		
}
void user_LCD_init(void)
{
		LCD_GPIO_Init();
		LCD_FMSC_Init();
		//LCD_Reset();
	  ILI9488_InitHard();
}