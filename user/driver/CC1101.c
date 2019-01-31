/*******************************************************************************
* EXTI Function is place in stm8l_it.c
*******************************************************************************/
#include "cc1101.h"
#include "stm32f10x.h"
#include "string.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "timer.h"
#include "timers.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
//GDO0 PC5
#define CC1101_GDO0_GPIO_PORT           GPIOC
#define CC1101_GDO0_GPIO_PINS           GPIO_Pin_5
#define CC1101_GDO0_EXTI_PORT_SRC       GPIO_PortSourceGPIOC
#define CC1101_GDO0_EXTI_PIN_SRC        GPIO_PinSource5
#define CC1101_GDO0_EXTI_LINE           EXTI_Line5
#define CC1101_GDO0_EXTI_TRIGGER        EXTI_Trigger_Falling
#define CC1101_GDO0_EXTI_ID             EXTI9_5_IRQn
//GDO2 PC4
#define CC1101_GDO2_GPIO_PORT           GPIOC
#define CC1101_GDO2_GPIO_PINS           GPIO_Pin_4
//CSN PA4
#define CC1101_CSN_GPIO_PORT            GPIOA
#define CC1101_CSN_GPIO_PINS            GPIO_Pin_4
//CLK PA5
#define CC1101_CLK_GPIO_PORT            GPIOA
#define CC1101_CLK_GPIO_PINS            GPIO_Pin_5
//MISO PA6
#define CC1101_MISO_GPIO_PORT           GPIOA
#define CC1101_MISO_GPIO_PINS           GPIO_Pin_6
//MOSI PA7
#define CC1101_MOSI_GPIO_PORT           GPIOA
#define CC1101_MOSI_GPIO_PINS           GPIO_Pin_7

#define CC1101_SPI                      SPI1

#define CC1101_MISO_VALUE()             (GPIO_ReadInputDataBit(CC1101_MISO_GPIO_PORT,CC1101_MISO_GPIO_PINS))
#define CC1101_GDO0_VALUE()             (GPIO_ReadInputDataBit(CC1101_GDO0_GPIO_PORT,CC1101_GDO0_GPIO_PINS))
#define CC1101_GDO2_VALUE()             (GPIO_ReadInputDataBit(CC1101_GDO2_GPIO_PORT,CC1101_GDO2_GPIO_PINS))
#define CC1101_CSN_SET_LOW()            (GPIO_ResetBits(CC1101_CSN_GPIO_PORT,CC1101_CSN_GPIO_PINS))
#define CC1101_CSN_SET_HIGH()           (GPIO_SetBits(CC1101_CSN_GPIO_PORT,CC1101_CSN_GPIO_PINS))


const RF_SETTINGS rfSettings = {
    0x2F,  // IOCFG2        GDO2 Output Pin Configuration
    0x06,  // IOCFG0        GDO0 Output Pin Configuration
    0x47,  // FIFOTHR       RX FIFO and TX FIFO Thresholds
    0xD3,  // SYNC1         Sync Word, High Byte
    0x91,  // SYNC0         Sync Word, Low Byte
    0xFF,  // PKTLEN        Packet Length
    0x04,  // PKTCTRL1      Packet Automation Control
    0x05,  // PKTCTRL0      Packet Automation Control
    RF_ROOT_ADDR,  // ADDR          Device Address
    0x05,  // CHANNR        Channel Number
    0x06,  // FSCTRL1       Frequency Synthesizer Control
    0x00,  // FSCTRL0       Frequency Synthesizer Control
    0x10,  // FREQ2         Frequency Control Word, High Byte
    0x89,  // FREQ1         Frequency Control Word, Middle Byte
    0xD8,  // FREQ0         Frequency Control Word, Low Byte
    RF_BW_REG|0x06,  // MDMCFG4       Modem Configuration
    0x83,  // MDMCFG3       Modem Configuration
    0x13,  // MDMCFG2       Modem Configuration
    0x22,  // MDMCFG1       Modem Configuration
    0xF8,  // MDMCFG0       Modem Configuration
    0x15,  // DEVIATN       Modem Deviation Setting
    0x18,  // MCSM0         Main Radio Control State Machine Configuration
    0x16,  // FOCCFG        Frequency Offset Compensation Configuration
    0x6C,  // BSCFG         Bit Synchronization Configuration
    0x03,  // AGCCTRL2      AGC Control
    0x40,  // AGCCTRL1      AGC Control
    0x91,  // AGCCTRL0      AGC Control
    0x56,  // FREND1        Front End RX Configuration
    0x10,  // FREND0        Front End TX Configuration
    0xE9,  // FSCAL3        Frequency Synthesizer Calibration
    0x2A,  // FSCAL2        Frequency Synthesizer Calibration
    0x00,  // FSCAL1        Frequency Synthesizer Calibration
    0x1F,  // FSCAL0        Frequency Synthesizer Calibration
    0x59,  // FSTEST        Frequency Synthesizer Calibration Control
    0x81,  // TEST2         Various Test Settings
    0x35,  // TEST1         Various Test Settings
    0x09,  // TEST0         Various Test Settings
};
/* Center Channel Freq 15-433.00MHz ; 25-435.00MHz for test */
const uint8_t CC1101_RFCH_RegVal[5] = {5,10,15,20,RF_TEST_CHANNEL_REG};

// the Timer to Clear CC1101 Err Status
xTimerHandle t_cc1101_norespond_clear_timer  = NULL;

volatile CC_MODE ge_CC1101_Mode = CC_MODE_RESET;
//------------------------------------------------------------------------------
static void CC1101_WriteReg(uint8_t u8Addr,uint8_t u8Data);
static void CC1101_WriteRegs(uint8_t u8Addr,const uint8_t *pu8Data,uint8_t u8Len);
static uint8_t CC1101_ReadReg(uint8_t u8Addr);
static void CC1101_ReadRegs(uint8_t u8Addr,uint8_t *pu8Data,uint8_t u8Len);
static void CC1101_SendCmd(uint8_t u8Cmd);
static void CC1101_RegInit(RF_SETTINGS *pRfSettings);
static void t_cc1101_no_respond_clear_callback( xTimerHandle  xTimer );

//------------------------------------------------------------------------------
CC1101_PACKET cc1101_rx_buff;
CC1101_PACKET cc1101_tx_test ;
RUNNING_PARAM RunningParam;
void cc1101_task(void *param)
{
  CC1101_PACKET *pPacket ;

  uint16_t return_bits;
  CC1101_Config();
  CC1101_EnterRXMode();
  printf("cc1101 enter rx mode \r\n");
	memset(&cc1101_tx_test,0,sizeof(cc1101_tx_test));
	memcpy(cc1101_tx_test.u8Buff,"hello cc1101",strlen("hello cc1101"));
	cc1101_tx_test.u8DataLen = strlen("hello cc1101");
	//
	if(t_cc1101_norespond_clear_timer == NULL)
        t_cc1101_norespond_clear_timer = xTimerCreate("t_cc1101_norespond_clear_timer ",  60*1000/portTICK_RATE_MS, pdTRUE, ( void * ) 0, t_cc1101_no_respond_clear_callback);
	if(t_cc1101_norespond_clear_timer != NULL)
		xTimerStart( t_cc1101_norespond_clear_timer, 0 );

  return_bits = xEventGroupWaitBits(cc1101_event_group, FLAG_TEST_MODE,  pdTRUE,  pdFALSE,  0);
  if(return_bits & FLAG_TEST_MODE)
  {
        CC1101_ChangeRFCH(CC1101_RF_CH_TEST);
  }
  else if(RunningParam.sRFCH.bRFCHSet)
    CC1101_ChangeRFCH(RunningParam.sRFCH.u2RFCH);
  else
  {
    uint8_t u8Temp = 3;//rand()&0x3;
    //RunningParam_WriteRFCH(u8Temp);
		RunningParam.sRFCH.u2RFCH = u8Temp&0x03;
    CC1101_ChangeRFCH(RunningParam.sRFCH.u2RFCH);
  }
  
  while(1)
  {

		if(xQueueReceive(cc1101_queque, (void*)&pPacket , 0) == pdTRUE)
		{
			
		}
    if(pPacket != NULL)
    {
#if (DBG_PRINT_PACKET_INFO == 1)
      USART1_SendByte('s');
#endif
      CC1101_SendPacket(pPacket->u8Buff,pPacket->u8DataLen);
      CC1101_EnterRXMode();
      memset(&cc1101_rx_buff,0,sizeof(cc1101_rx_buff));
	    memcpy(&cc1101_rx_buff,pPacket,sizeof(cc1101_rx_buff));
			pPacket = NULL;
    }
     return_bits = xEventGroupWaitBits(cc1101_event_group, FLAG_CC1101_RECV_PACKET,  pdTRUE,  pdFALSE,  0);
     if((return_bits & FLAG_CC1101_RECV_PACKET) == FLAG_CC1101_RECV_PACKET)
     {
        CC1101_ReadPacket();
     }

     return_bits = xEventGroupWaitBits(cc1101_event_group, FLAG_CC1101_RESPOND,  pdTRUE,  pdFALSE,  0);
     if((return_bits & FLAG_CC1101_RESPOND) == FLAG_CC1101_RESPOND )
     {
        xEventGroupSetBits(cc1101_event_group,FLAG_CC1101_RESPOND);
        xTimerStart( t_cc1101_norespond_clear_timer, 0 );
     }   
    
    vTaskDelay(10 / portTICK_RATE_MS);
	}

}

void CC1101_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {.GPIO_Speed = GPIO_Speed_50MHz,};
  
  const SPI_InitTypeDef SPI_InitStructure = 
  {
    .SPI_Direction = SPI_Direction_2Lines_FullDuplex,
    .SPI_Mode = SPI_Mode_Master,
    .SPI_DataSize = SPI_DataSize_8b,
    .SPI_CPOL = SPI_CPOL_Low,
    .SPI_CPHA = SPI_CPHA_1Edge,
    .SPI_NSS = SPI_NSS_Soft,
    .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
    .SPI_FirstBit = SPI_FirstBit_MSB,
    .SPI_CRCPolynomial = 0x07,
  };
  
  const EXTI_InitTypeDef EXTI_InitStructure =
  {
    .EXTI_Mode = EXTI_Mode_Interrupt,
    .EXTI_Trigger = EXTI_Trigger_Rising,
    .EXTI_LineCmd = ENABLE,
    .EXTI_Line = CC1101_GDO0_EXTI_LINE,
  };
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = CC1101_GDO0_GPIO_PINS;          // GDO0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	
  GPIO_Init(CC1101_GDO0_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = CC1101_GDO2_GPIO_PINS;          // GDO2
	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(CC1101_GDO2_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = CC1101_CSN_GPIO_PINS;           // CSN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(CC1101_CSN_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = CC1101_CLK_GPIO_PINS;           // SCK
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(CC1101_CLK_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = CC1101_MOSI_GPIO_PINS;          // MOSI
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(CC1101_MOSI_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = CC1101_MISO_GPIO_PINS;          // MISO
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(CC1101_MISO_GPIO_PORT, &GPIO_InitStructure);
  
  SPI_I2S_DeInit(CC1101_SPI);
  SPI_Init(CC1101_SPI,(SPI_InitTypeDef*)&SPI_InitStructure);
  SPI_Cmd(CC1101_SPI,ENABLE);
  SPI_NSSInternalSoftwareConfig(CC1101_SPI,SPI_NSSInternalSoft_Set);
  
  CC1101_PowerUpReset();
  // GDO0 Interrupt
  GPIO_EXTILineConfig(CC1101_GDO0_EXTI_PORT_SRC,CC1101_GDO0_EXTI_PIN_SRC);
  EXTI_Init((EXTI_InitTypeDef*)&EXTI_InitStructure);

	NVIC_InitTypeDef 	NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}

static void CC1101_WriteReg(uint8_t u8Addr,uint8_t u8Data)
{
  CC1101_CSN_SET_LOW();
  __NOP();
  while(CC1101_MISO_VALUE());
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_TXE)==RESET);
  
  SPI_I2S_SendData(CC1101_SPI,u8Addr);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  SPI_I2S_ReceiveData(CC1101_SPI);
  SPI_I2S_SendData(CC1101_SPI,u8Data);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  SPI_I2S_ReceiveData(CC1101_SPI);
  
  CC1101_CSN_SET_HIGH();
}

static void CC1101_WriteRegs(uint8_t u8Addr,const uint8_t *pu8Data,uint8_t u8Len)
{
  if(u8Len==0)
    return;
  CC1101_CSN_SET_LOW();
  __NOP();
  while(CC1101_MISO_VALUE());
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_TXE)==RESET);
  
  SPI_I2S_SendData(CC1101_SPI,u8Addr|0x40);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  SPI_I2S_ReceiveData(CC1101_SPI);
  
  for(uint8_t i = 0;i < u8Len;i++)
  {
    SPI_I2S_SendData(CC1101_SPI,*(pu8Data+i));
    while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
    SPI_I2S_ReceiveData(CC1101_SPI);
  }
  CC1101_CSN_SET_HIGH();
}

static uint8_t CC1101_ReadReg(uint8_t u8Addr)
{
  uint8_t u8Temp;
  CC1101_CSN_SET_LOW();
  __NOP();
  while(CC1101_MISO_VALUE());
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_TXE)==RESET);
  
  SPI_I2S_SendData(CC1101_SPI,u8Addr|0x80);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  SPI_I2S_ReceiveData(CC1101_SPI);
  SPI_I2S_SendData(CC1101_SPI,0xFF);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  u8Temp = SPI_I2S_ReceiveData(CC1101_SPI);
  
  CC1101_CSN_SET_HIGH();
  return u8Temp;
}

static void CC1101_ReadRegs(uint8_t u8Addr,uint8_t *pu8Data,uint8_t u8Len)
{
  if(u8Len==0)
    return;
  CC1101_CSN_SET_LOW();
  __NOP();
  while(CC1101_MISO_VALUE());
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_TXE)==RESET);
  
  SPI_I2S_SendData(CC1101_SPI,u8Addr|0xC0);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  SPI_I2S_ReceiveData(CC1101_SPI);
  
  for(uint8_t i = 0;i < u8Len;i++)
  {
    SPI_I2S_SendData(CC1101_SPI,0xFF);
    while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
    *(pu8Data+i)=SPI_I2S_ReceiveData(CC1101_SPI);
  }
  CC1101_CSN_SET_HIGH();
}

static uint8_t CC1101_ReadStatus(uint8_t u8Addr)
{
  uint8_t u8Temp;
  CC1101_CSN_SET_LOW();
  __NOP();
  while(CC1101_MISO_VALUE());
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_TXE)==RESET);
  
  SPI_I2S_SendData(CC1101_SPI,u8Addr|0xC0);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  SPI_I2S_ReceiveData(CC1101_SPI);
  SPI_I2S_SendData(CC1101_SPI,0xFF);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  u8Temp = SPI_I2S_ReceiveData(CC1101_SPI);
  
  CC1101_CSN_SET_HIGH();
  return u8Temp;
}

static void CC1101_SendCmd(uint8_t u8Cmd)
{
  CC1101_CSN_SET_LOW();
  __NOP();
  while(CC1101_MISO_VALUE());
  
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_TXE)==RESET);
  SPI_I2S_SendData(CC1101_SPI,u8Cmd);
  while(SPI_I2S_GetFlagStatus(CC1101_SPI,SPI_I2S_FLAG_RXNE)==RESET);
  SPI_I2S_ReceiveData(CC1101_SPI);
  
  CC1101_CSN_SET_HIGH();
}
//------------------------------------------------------------------------------
static void CC1101_RegInit(RF_SETTINGS *pRfSettings)
{
  CC1101_WriteReg(CC1101_FSCTRL1,  pRfSettings->FSCTRL1);
  CC1101_WriteReg(CC1101_FSCTRL0,  pRfSettings->FSCTRL0);
  CC1101_WriteReg(CC1101_FREQ2,    pRfSettings->FREQ2);
  CC1101_WriteReg(CC1101_FREQ1,    pRfSettings->FREQ1);
  CC1101_WriteReg(CC1101_FREQ0,    pRfSettings->FREQ0);
  CC1101_WriteReg(CC1101_MDMCFG4,  pRfSettings->MDMCFG4);
  CC1101_WriteReg(CC1101_MDMCFG3,  pRfSettings->MDMCFG3);
  CC1101_WriteReg(CC1101_MDMCFG2,  pRfSettings->MDMCFG2);
  CC1101_WriteReg(CC1101_MDMCFG1,  pRfSettings->MDMCFG1);
  CC1101_WriteReg(CC1101_MDMCFG0,  pRfSettings->MDMCFG0);
  CC1101_WriteReg(CC1101_CHANNR,   pRfSettings->CHANNR);
  CC1101_WriteReg(CC1101_DEVIATN,  pRfSettings->DEVIATN);
  CC1101_WriteReg(CC1101_FREND1,   pRfSettings->FREND1);
  CC1101_WriteReg(CC1101_FREND0,   pRfSettings->FREND0);
  CC1101_WriteReg(CC1101_MCSM0 ,   pRfSettings->MCSM0 );
  CC1101_WriteReg(CC1101_FOCCFG,   pRfSettings->FOCCFG);
  CC1101_WriteReg(CC1101_BSCFG,    pRfSettings->BSCFG);
  CC1101_WriteReg(CC1101_AGCCTRL2, pRfSettings->AGCCTRL2);
  CC1101_WriteReg(CC1101_AGCCTRL1, pRfSettings->AGCCTRL1);
  CC1101_WriteReg(CC1101_AGCCTRL0, pRfSettings->AGCCTRL0);
  CC1101_WriteReg(CC1101_FSCAL3,   pRfSettings->FSCAL3);
  CC1101_WriteReg(CC1101_FSCAL2,   pRfSettings->FSCAL2);
  CC1101_WriteReg(CC1101_FSCAL1,   pRfSettings->FSCAL1);
  CC1101_WriteReg(CC1101_FSCAL0,   pRfSettings->FSCAL0);
  CC1101_WriteReg(CC1101_FSTEST,   pRfSettings->FSTEST);
  CC1101_WriteReg(CC1101_TEST2,    pRfSettings->TEST2);
  CC1101_WriteReg(CC1101_TEST1,    pRfSettings->TEST1);
  CC1101_WriteReg(CC1101_TEST0,    pRfSettings->TEST0);
  CC1101_WriteReg(CC1101_FIFOTHR,  pRfSettings->FIFOTHR);
  CC1101_WriteReg(CC1101_IOCFG2,   pRfSettings->IOCFG2);
  CC1101_WriteReg(CC1101_IOCFG0,   pRfSettings->IOCFG0);    
  CC1101_WriteReg(CC1101_PKTCTRL1, pRfSettings->PKTCTRL1);
  CC1101_WriteReg(CC1101_PKTCTRL0, pRfSettings->PKTCTRL0);
  CC1101_WriteReg(CC1101_ADDR,     pRfSettings->ADDR);
  CC1101_WriteReg(CC1101_PKTLEN,   pRfSettings->PKTLEN);
  CC1101_WriteReg(CC1101_SYNC1,    pRfSettings->SYNC1);
  CC1101_WriteReg(CC1101_SYNC0,    pRfSettings->SYNC0);
}

void CC1101_PowerUpReset(void)
{
  ge_CC1101_Mode = CC_MODE_RESET;
  // Reset
  CC1101_CSN_SET_HIGH();
  vTaskDelay(10 / portTICK_RATE_MS);
  CC1101_CSN_SET_LOW();
  vTaskDelay(10 / portTICK_RATE_MS);
  CC1101_CSN_SET_HIGH();
  CC1101_SendCmd(CC1101_SRES);
  // Reg Init
  CC1101_RegInit((RF_SETTINGS*)&rfSettings);
  CC1101_WriteReg(CC1101_PATABLE,CC1101_PA_POWER);
  CC1101_SendCmd(CC1101_SIDLE);
  ge_CC1101_Mode = CC_MODE_IDLE;
}

void CC1101_SendPacket(const uint8_t *pu8Data,uint8_t u8Len)
{
  uint8_t u8Temp;
  uint16_t u16Cnt;
  ge_CC1101_Mode = CC_MODE_TX;
  CC1101_SendCmd(CC1101_SIDLE);
  CC1101_SendCmd(CC1101_SFTX);
  CC1101_WriteReg(CC1101_TXFIFO,u8Len);
  CC1101_WriteRegs(CC1101_TXFIFO,pu8Data,u8Len);
  CC1101_SendCmd(CC1101_STX);
  // Wait while CC1101_GDO0=0
  u16Cnt = CC1101_TRANS_PACKET_TIMEOUT;
  while(--u16Cnt)
  {
    if(CC1101_GDO0_VALUE())
      break;
    else
      vTaskDelay(10 / portTICK_RATE_MS);
  }
  if(!CC1101_GDO0_VALUE())      goto ret_cc1101_sendpacket;
  // Wait while CC1101_GDO0=1
  u16Cnt = CC1101_TRANS_PACKET_TIMEOUT;
  while(--u16Cnt)
  {
    if(CC1101_GDO0_VALUE())
      vTaskDelay(10 / portTICK_RATE_MS);
    else
      break;
  }
  if(!CC1101_GDO0_VALUE())      goto ret_cc1101_sendpacket;
  // Wait while TXBYTES!=0
  u16Cnt = CC1101_TRANS_PACKET_TIMEOUT;
  while(--u16Cnt)
  {
    u8Temp = CC1101_ReadStatus(CC1101_TXBYTES)&0x7F;
    if(u8Temp)
      vTaskDelay(10 / portTICK_RATE_MS);
    else
      break;
  }
ret_cc1101_sendpacket:
  CC1101_SendCmd(CC1101_SIDLE);
  ge_CC1101_Mode = CC_MODE_IDLE;
}

void CC1101_ReadPacket(void)
{
  uint8_t u8PacketLen,u8ReadCnt,u8Temp;
  uint16_t u16Cnt;
  uint8_t u8Status[2];  // status[0]-RSSI;status[1]-LQI
  CC1101_PACKET *pPacket;
	pPacket = (CC1101_PACKET* )pvPortMalloc(sizeof(CC1101_PACKET));
	memset(pPacket,0,sizeof(CC1101_PACKET));
  // Wait while RXBYTES==0
  u16Cnt = CC1101_TRANS_PACKET_TIMEOUT;
  while(--u16Cnt)
  {
    u8Temp = CC1101_ReadStatus(CC1101_RXBYTES)&0x7F;
    if(u8Temp)
      break;
    else
      vTaskDelay(10 / portTICK_RATE_MS);
  }
	 if(u8Temp)
  {
		xEventGroupSetBits(cc1101_event_group,FLAG_CC1101_RESPOND);
		
    if(pPacket == NULL)
      goto ret_cc1101_readpacket;
    u8PacketLen = CC1101_ReadReg(CC1101_RXFIFO);
    if((u8PacketLen > CC1101_PACKET_SIZE)||(u8PacketLen == 0))
      goto ret_cc1101_readpacket;
    else
    {
      u8ReadCnt = 0;
      u16Cnt = CC1101_TRANS_PACKET_TIMEOUT;
      while(u8ReadCnt < u8PacketLen)
      {
        u8Temp = CC1101_ReadStatus(CC1101_RXBYTES)&0x7F;
        if(u8Temp)
        {
          *(pPacket->u8Buff+u8ReadCnt)=CC1101_ReadReg(CC1101_RXFIFO);
          u8ReadCnt++;
        }
        u16Cnt--;
        if(!u16Cnt)
        {
          goto ret_cc1101_readpacket;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
      }
      pPacket->u8DataLen=u8PacketLen;
      CC1101_ReadRegs(CC1101_RXFIFO,u8Status,2);		
    }
  }
ret_cc1101_readpacket:
	memset(&cc1101_rx_buff,0,sizeof(cc1101_rx_buff));
	memcpy(&cc1101_rx_buff,pPacket,sizeof(cc1101_rx_buff));
	if(cc1101_rx_buff.u8DataLen)
	{
		printf("cc1101 rcv(len:%d):%s \r\n",cc1101_rx_buff.u8DataLen,cc1101_rx_buff.u8Buff);
	}
  CC1101_EnterRXMode();
	vPortFree(pPacket);
}

void CC1101_Recovery(void)
{
  if(CC1101_ReadStatus(CC1101_RXBYTES))
  {
    CC1101_SendCmd(CC1101_SIDLE);
    CC1101_SendCmd(CC1101_SFRX);
    xEventGroupSetBits(cc1101_event_group,FLAG_CC1101_RECV_PACKET);
    ge_CC1101_Mode=CC_MODE_IDLE;
  }
}

inline void CC1101_EnterRXMode(void)
{
  CC1101_SendCmd(CC1101_SIDLE);
  CC1101_SendCmd(CC1101_SFRX);
  xEventGroupSetBits(cc1101_event_group,FLAG_CC1101_RECV_PACKET);
  vTaskDelay(20 / portTICK_RATE_MS);
  CC1101_SendCmd(CC1101_SRX);
  ge_CC1101_Mode = CC_MODE_RX;
}

void CC1101_ChangeRFCH(uint8_t u8RFCH)
{
  if(u8RFCH>=sizeof(CC1101_RFCH_RegVal))
    return;
  CC1101_SendCmd(CC1101_SIDLE);
  CC1101_WriteReg(CC1101_CHANNR,CC1101_RFCH_RegVal[u8RFCH]);
}

static void t_cc1101_no_respond_clear_callback( xTimerHandle  xTimer )
{
  uint16_t return_bits;
#if (DBG_PRINT_PACKET_INFO == 1)
  UART_PutChar(UART_DEBUG,'N');
#endif
   return_bits = xEventGroupWaitBits(cc1101_event_group, FLAG_CC1101_RECV_PACKET,  pdTRUE,  pdFALSE,  0);
   if((return_bits & FLAG_CC1101_RECV_PACKET) == FLAG_CC1101_RECV_PACKET)
   {
          CC1101_Recovery();
          CC1101_EnterRXMode();
    }

  
}

void EXTI9_5_IRQHandler(void)
{
	BaseType_t xResult;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  if(EXTI_GetFlagStatus(CC1101_GDO0_EXTI_LINE)==SET)
  {
    if(ge_CC1101_Mode == CC_MODE_RX)
		{
			xResult = xEventGroupSetBitsFromISR(cc1101_event_group,FLAG_CC1101_RECV_PACKET,&xHigherPriorityTaskWoken);
		}
		if(xResult != pdFAIL)
		{
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
    EXTI_ClearITPendingBit(CC1101_GDO0_EXTI_LINE);
  }
}
