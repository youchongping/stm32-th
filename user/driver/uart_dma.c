#include "uart_dma.h"
#include <stdio.h> 
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
struct uart_rx_s
{
	u16	usart_Timeout;
	u16  usart_cnt;
	u16	usart_cntlast;
	u8	usart_Data[UART_MAX_RECV_LEN];
};

struct uart_rx_s   uart1_rx;
struct uart_rx_s   uart2_rx; 
struct uart_rx_s   uart3_rx;

Rcv_t wifi_5g;
Rcv_t debug_rcv;

/*USART1_RX  -- DMA1_Channel5 is physically together */
void DMA1_5_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel5); 
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART1_DR; 
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)uart1_rx.usart_Data; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	DMA_InitStructure.DMA_BufferSize = UART_MAX_RECV_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); 
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE); 
	DMA_Cmd(DMA1_Channel5, ENABLE);         
}

void USART1_Configuration(void)
{
	 GPIO_InitTypeDef GPIO_InitStructure;	
	 USART_InitTypeDef USART_InitStructure;  
	 NVIC_InitTypeDef NVIC_InitStructure;

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	 GPIO_Init(GPIOA, &GPIO_InitStructure);  
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
	 GPIO_Init(GPIOA, &GPIO_InitStructure);  

	 NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	 
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	 NVIC_Init(&NVIC_InitStructure); 

	 USART_InitStructure.USART_BaudRate            = 115200  ;
	 USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	 USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	 USART_InitStructure.USART_Parity              = USART_Parity_No ;
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	 USART_Init(USART1, &USART_InitStructure);

	 USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	 USART_Cmd(USART1, ENABLE);
}

 

 /*USART2_RX  -- DMA1_Channel6 is physically together */
 void DMA1_6_Init(void)
 {
	 DMA_InitTypeDef DMA_InitStructure;
	 DMA_DeInit(DMA1_Channel6); 
	 DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART2_DR; 
	 DMA_InitStructure.DMA_MemoryBaseAddr = (u32)uart2_rx.usart_Data;
	 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	 DMA_InitStructure.DMA_BufferSize = UART_MAX_RECV_LEN;
	 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	 DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; 
	 DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	 DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
	 DMA_Init(DMA1_Channel6, &DMA_InitStructure); 
	
	 USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE); 
	 DMA_Cmd(DMA1_Channel6, ENABLE); 
 }
	
 void USART2_Configuration(void)
 {
	 GPIO_InitTypeDef GPIO_InitStructure;	 
	 USART_InitTypeDef USART_InitStructure;  
	 NVIC_InitTypeDef NVIC_InitStructure;
	                      
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	 GPIO_Init(GPIOA, &GPIO_InitStructure);   
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	 GPIO_Init(GPIOA, &GPIO_InitStructure);  

	 NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		 
	 NVIC_Init(&NVIC_InitStructure); 

	 USART_InitStructure.USART_BaudRate			= 115200  ;
	 USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
	 USART_InitStructure.USART_StopBits			= USART_StopBits_1;
	 USART_InitStructure.USART_Parity				= USART_Parity_No ;
	 USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 USART_InitStructure.USART_Mode				= USART_Mode_Rx | USART_Mode_Tx;
	 USART_Init(USART2, &USART_InitStructure);
	 
	 USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	 USART_Cmd(USART2, ENABLE); 

 }

 /*USART3_RX -- DMA1_Channel3 is physically together*/
 void DMA1_3_Init(void)
 {
	 DMA_InitTypeDef DMA_InitStructure; 
	 DMA_DeInit(DMA1_Channel3); 
	 DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)SRC_USART3_DR; 
	 DMA_InitStructure.DMA_MemoryBaseAddr = (u32)uart3_rx.usart_Data; 
	 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
	 DMA_InitStructure.DMA_BufferSize = UART_MAX_RECV_LEN; 
	 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
	 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	 DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	 DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 
	 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
	 DMA_Init(DMA1_Channel3, &DMA_InitStructure); 
	 
	 USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); 
	 DMA_Cmd(DMA1_Channel3, ENABLE); 
 }
 	
 void USART3_Configuration(void)
 {
	  GPIO_InitTypeDef GPIO_InitStructure;	 
	  USART_InitTypeDef USART_InitStructure;  
	  NVIC_InitTypeDef NVIC_InitStructure;

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);  
	  /* USART3 GPIO config */	 
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	  GPIO_Init(GPIOC, &GPIO_InitStructure);  
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	  GPIO_Init(GPIOC, &GPIO_InitStructure);  

	  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	 
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		  
	  NVIC_Init(&NVIC_InitStructure); 
		
	  USART_InitStructure.USART_BaudRate			= 115200  ;
	  USART_InitStructure.USART_WordLength			= USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits			= USART_StopBits_1;
	  USART_InitStructure.USART_Parity				= USART_Parity_No ;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode				= USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART3, &USART_InitStructure);
	 
	  USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
	  USART_Cmd(USART3, ENABLE);
	 
 }

void serial_init(void)
{
	//USART1_Configuration();
	//DMA1_5_Init();
	
	USART2_Configuration();
	DMA1_6_Init();
	
	USART3_Configuration();
	DMA1_3_Init();

}

void USART2_IRQHandler(void)
{
	u8 lenth =0 ;
	
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!= RESET)
	{
		DMA_Cmd(DMA1_Channel6, DISABLE); 
		DMA_ClearFlag(DMA1_FLAG_TC6);
		lenth=USART2->DR;//clear USART_IT_IDLE
        lenth=USART2->SR;//clear USART_IT_IDLE

		uart2_rx.usart_cnt = UART_MAX_RECV_LEN - DMA_GetCurrDataCounter(DMA1_Channel6);
        memset(wifi_5g.rxdata,0,sizeof(wifi_5g.rxdata));
		wifi_5g.rxcnt = uart2_rx.usart_cnt ;
		memcpy(wifi_5g.rxdata,uart2_rx.usart_Data,uart2_rx.usart_cnt);
		memset(uart2_rx.usart_Data,0,sizeof(uart2_rx.usart_Data));
		uart2_rx.usart_cnt = 0 ;
		
		DMA_SetCurrDataCounter(DMA1_Channel6,UART_MAX_RECV_LEN);
		DMA_Cmd(DMA1_Channel6, ENABLE); 
		wifi_5g.rxflag = 1;
		
	}
}
void USART3_IRQHandler(void)
{
	u8 lenth =0 ;
	
	if(USART_GetITStatus(USART3,USART_IT_IDLE)!= RESET)
	{
		DMA_Cmd(DMA1_Channel3, DISABLE); 
		DMA_ClearFlag(DMA1_FLAG_TC3);
		lenth=USART3->DR;//clear USART_IT_IDLE
        lenth=USART3->SR;//clear USART_IT_IDLE

		uart3_rx.usart_cnt = UART_MAX_RECV_LEN - DMA_GetCurrDataCounter(DMA1_Channel6);
        memset(debug_rcv.rxdata,0,sizeof(wifi_5g.rxdata));
		debug_rcv.rxcnt = uart3_rx.usart_cnt ;
		memcpy(debug_rcv.rxdata,uart3_rx.usart_Data,uart3_rx.usart_cnt);
		memset(uart3_rx.usart_Data,0,sizeof(uart3_rx.usart_Data));
		uart3_rx.usart_cnt = 0 ;

		
		DMA_SetCurrDataCounter(DMA1_Channel3,UART_MAX_RECV_LEN);
		DMA_Cmd(DMA1_Channel3, ENABLE); 
		debug_rcv.rxflag = 1;
		
	}
	
}


/* 3 SendFunctions*/
void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data)  
{  
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);  
	USART_SendData(USARTx, Data);  
}  

void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str)    
{    
	while (0 != *str)    
	{    
		UART_PutChar(USARTx, *str);    
		str++;    
	}
}  

void UART_PutHex(USART_TypeDef* USARTx, uint8_t *pdata,u16 datalen)    
{    
	u16 i=0;
	for(i=0;i<datalen;i++)
	{    
		UART_PutChar(USARTx, *(pdata+i));        
	}    
}  

int fputc(int ch, FILE *f)
{
	USART_SendData(USART3, (uint8_t) ch);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	
	return ch;
}
void wifi_data_process(u8* buffer,u8 buf_lenth)
{
	char i=0;
	for(i=0;i<buf_lenth;i++)
    printf("%02x",buffer[i]);
}
void uart_task(void* param)
{
	 while(1)
	 {
		 if(wifi_5g.rxflag == 1)
		 {
			wifi_data_process((u8*)wifi_5g.rxdata,wifi_5g.rxcnt); 
			wifi_5g.rxflag = 0;
		 }
		 vTaskDelay(20 / portTICK_RATE_MS);
	 }
}


