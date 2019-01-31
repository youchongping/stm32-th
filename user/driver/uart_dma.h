#ifndef _UART_DMA_H_
#define _UART_DMA_H_

#include "stm32f10x.h"
#include <string.h>
#define UART_MAX_RECV_LEN		( 128	)				
#define UART_MAX_SEND_LEN		( 128	)				
#define UART_5G USART2
#define UART_DEBUG USART3
typedef  struct{
	u16 rxcnt; 
	u8 rxdata[UART_MAX_RECV_LEN];
	u8 rxflag;
}Rcv_t;

#define SRC_USART1_DR (&(USART1->DR)) 
#define SRC_USART2_DR (&(USART2->DR)) 
#define SRC_USART3_DR (&(USART3->DR)) 
extern Rcv_t wifi_5g;
extern Rcv_t debug_rcv;

extern void serial_init(void);
extern void UART_PutChar(USART_TypeDef* USARTx, uint8_t Data)  ;
extern void UART_PutStr (USART_TypeDef* USARTx, uint8_t *str)  ;  
extern void UART_PutHex(USART_TypeDef* USARTx, uint8_t *pdata,u16 datalen)   ;
extern void uart_task(void* param);

#endif

