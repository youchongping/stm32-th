#ifndef __BSP_GT911_H_
#define __BSP_GT911_H_
#include "stm32f10x.h"
/*
GT911 i2c address :0xBA/0xBB or 0x28/0x29  read/write
*/
#define GT911_I2C_ADDR 0x28
#define GT911_I2C_ADDR2 0xba
typedef struct
{
uint8_t Enable;
uint8_t i2c_addr;

uint8_t TouchpointFlag;

uint8_t Touchkey1trackid;
uint16_t X1;
uint16_t Y1;
uint16_t S1;

uint8_t Touchkey2trackid;
uint16_t X2;
uint16_t Y2;
uint16_t S2;

uint8_t Touchkey3trackid;
uint16_t X3;
uint16_t Y3;
uint16_t S3;

uint8_t Touchkey4trackid;
uint16_t X4;
uint16_t Y4;
uint16_t S4;

uint8_t Touchkey5trackid;
uint16_t X5;
uint16_t Y5;
uint16_t S5;
}GT911_T;
extern GT911_T g_GT911;
void GT911_InitHard(void);
void GT911_Scan(void);
void GT911_OnePiontScan(void);
#endif