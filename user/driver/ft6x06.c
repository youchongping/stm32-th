#include "ft6x06.h"
#include "ctiic.h"
#include "string.h" 
#include "stdio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "GUI.h"

//向ft6x06写入一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:写数据长度
//返回值:0,成功;1,失败.
u8 ft5x16_WR_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i;
	u8 ret=0;
	CT_IIC_Start();	 
	CT_IIC_Send_Byte(FT_CMD_WR);	//发送写命令 	 
	CT_IIC_Wait_Ack(); 	 										  		   
	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
	for(i=0;i<len;i++)
	{	   
    	CT_IIC_Send_Byte(buf[i]);  	//发数据
		ret=CT_IIC_Wait_Ack();
		if(ret)break;  
	}
    CT_IIC_Stop();					//产生一个停止条件	    
	return ret; 
}
//从ft6x06读出一次数据
//reg:起始寄存器地址
//buf:数据缓缓存区
//len:读数据长度			  
void ft5x16_RD_Reg(u16 reg,u8 *buf,u8 len)
{
	u8 i; 
 	CT_IIC_Start();	
 	CT_IIC_Send_Byte(FT_CMD_WR);   	//发送写命令 	 
	CT_IIC_Wait_Ack(); 	 										  		   
 	CT_IIC_Send_Byte(reg&0XFF);   	//发送低8位地址
	CT_IIC_Wait_Ack();  
 	CT_IIC_Start();  	 	   
	CT_IIC_Send_Byte(FT_CMD_RD);   	//发送读命令		   
	CT_IIC_Wait_Ack();	   
	for(i=0;i<len;i++)
	{	   
    	buf[i]=CT_IIC_Read_Byte(i==(len-1)?0:1); //发数据	  
	} 
    CT_IIC_Stop();//产生一个停止条件     
} 
//初始化ft6x06触摸屏
//返回值:0,初始化成功;1,初始化失败 

void ft_gpio_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;	
	EXTI_InitTypeDef 	EXTI_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	GPIO_InitStructure.GPIO_Pin = FT_RST_PIN;				 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(FT_RST_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(FT_RST_GPIO,FT_RST_PIN);
		
  GPIO_InitStructure.GPIO_Pin = FT_INT_PIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		 
	GPIO_Init(FT_INT_GPIO, &GPIO_InitStructure);

	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource1);//PB1 INT

	EXTI_InitStructure.EXTI_Line = EXTI_Line1; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}
u8 FT5216_Init(void)
{
	u8 temp[2];  		
  ft_gpio_init();
	CT_IIC_Init();      	//初始化电容屏的I2C总线  
	
	FT_RST=0;				//复位
	vTaskDelay(20 / portTICK_RATE_MS);
 	FT_RST=1;				//释放复位		    
	vTaskDelay(50 / portTICK_RATE_MS);  	
	temp[0]=0;
	ft5x16_WR_Reg(FT_DEVIDE_MODE,temp,1);	//进入正常操作模式 
	temp[0]=1;
	ft5x16_WR_Reg(FT_ID_G_MODE,temp,1);		//中断模式
	temp[0]=22;								//触摸有效值，22，越小越灵敏	
	ft5x16_WR_Reg(FT_ID_G_THGROUP,temp,1);	//设置触摸有效值
	temp[0]=12;								//激活周期，不能小于12，最大14
	ft5x16_WR_Reg(FT_ID_G_PERIODACTIVE,temp,1); 
	//读取版本号，参考值：0x3003
	ft5x16_RD_Reg(FT_ID_G_LIB_VERSION,&temp[0],2);  
	if(temp[0]==0X30&&temp[1]==0X03)//版本:0X3003
	{ 
		printf("CTP ID:%x\r\n",((u16)temp[0]<<8)+temp[1]);
		return 0;
	} 
	return 1;
}
void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
	{		
		EXTI_ClearITPendingBit(EXTI_Line1);   
		printf("have touch data \r\n");
	}
}


 





































