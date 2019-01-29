#ifndef __FT5216_H
#define __FT5216_H	
#include "sys.h"	
	 
#define FT_RST    				PBout(0)	//FT5206��λ����
#define FT_INT    				PBin(1)	//FT5206�ж�����	
#define FT_RST_PIN				GPIO_Pin_0
#define FT_RST_GPIO				GPIOB
#define FT_INT_PIN				GPIO_Pin_1
#define FT_INT_GPIO				GPIOB
//I2C��д����	
#define FT_CMD_WR 				0X70    	//д����
#define FT_CMD_RD 				0X71		//������
  
//FT5216 ���ּĴ������� 
#define FT_DEVIDE_MODE 			0x00   		//FT5206ģʽ���ƼĴ���
#define FT_REG_NUM_FINGER       0x02		//����״̬�Ĵ���

#define FT_TP1_REG 				0X03	  	//��һ�����������ݵ�ַ
#define FT_TP2_REG 				0X09		//�ڶ������������ݵ�ַ
#define FT_TP3_REG 				0X0F		//���������������ݵ�ַ
#define FT_TP4_REG 				0X15		//���ĸ����������ݵ�ַ
#define FT_TP5_REG 				0X1B		//��������������ݵ�ַ  


#define	FT_ID_G_LIB_VERSION		0xA1		//�汾		
#define FT_ID_G_MODE 			0xA4   		//FT5206�ж�ģʽ���ƼĴ���
#define FT_ID_G_THGROUP			0x80   		//������Чֵ���üĴ���
#define FT_ID_G_PERIODACTIVE	0x88   		//����״̬�������üĴ���

struct tp
{
	u16 x;
	u16 y;
	u8  event;
	u8  id;
};

u8 FT5216_WR_Reg(u16 reg,u8 *buf,u8 len);
void FT5216_RD_Reg(u16 reg,u8 *buf,u8 len);
u8 FT5216_Init(void);
u8 FT5216_Scan(u8 mode);


#endif

















