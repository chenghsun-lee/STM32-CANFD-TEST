/*----------------------------------------------------------------------------
 * Name:    CanDemo.c
 * Purpose: CAN example for MCBSTM32E
 * Note(s): possible defines set in "options for target - C/C++ - Define"
 *            __USE_LCD   - enable Output on LCD
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009-2013 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "stm32f10x.h"                            /* STM32F10x Definitions    */
#include "CAN.h"                                  /* STM32 CAN adaption layer */
#include "stdint.h"
#include "string.h"
#include <asf.h>
#include "app.h"

//led pin define
#define RCC_LED1 RCC_APB2Periph_GPIOA
#define PORT_LED1 GPIOA
#define LED1_PIN GPIO_Pin_15


#define RCC_LED2 RCC_APB2Periph_GPIOC
#define PORT_LED2 GPIOC
#define LED2_PIN GPIO_Pin_10

#define RCC_LED3 RCC_APB2Periph_GPIOC
#define PORT_LED3 GPIOC
#define LED3_PIN GPIO_Pin_11

#define RCC_LED4 RCC_APB2Periph_GPIOC
#define PORT_LED4 GPIOC
#define LED4_PIN GPIO_Pin_12


//swtich pin define
#define RCC_KEY1 RCC_APB2Periph_GPIOB
#define PORT_KEY1 GPIOB
#define KEY1_PIN GPIO_Pin_12

#define RCC_KEY2 RCC_APB2Periph_GPIOB
#define PORT_KEY2 GPIOB
#define KEY2_PIN GPIO_Pin_13

#define RCC_KEY3 RCC_APB2Periph_GPIOB
#define PORT_KEY3 GPIOB
#define KEY3_PIN GPIO_Pin_14

#define RCC_KEY4 RCC_APB2Periph_GPIOB
#define PORT_KEY4 GPIOB
#define KEY4_PIN GPIO_Pin_15

#define LED1_ON() GPIO_ResetBits(PORT_LED1,LED1_PIN)
#define LED1_OFF() GPIO_SetBits(PORT_LED1,LED1_PIN)

#define LED2_ON() GPIO_ResetBits(PORT_LED2,LED2_PIN)
#define LED2_OFF() GPIO_SetBits(PORT_LED2,LED2_PIN)

#define LED3_ON() GPIO_ResetBits(PORT_LED3,LED3_PIN)
#define LED3_OFF() GPIO_SetBits(PORT_LED3,LED3_PIN)

#define LED4_ON() GPIO_ResetBits(PORT_LED4,LED4_PIN)
#define LED4_OFF() GPIO_SetBits(PORT_LED4,LED4_PIN)


#define READ_SWTICH_1() (!GPIO_ReadInputDataBit(PORT_KEY1,KEY1_PIN))
#define READ_SWTICH_2() (!GPIO_ReadInputDataBit(PORT_KEY2,KEY2_PIN))
#define READ_SWTICH_3() (!GPIO_ReadInputDataBit(PORT_KEY3,KEY3_PIN))
#define READ_SWTICH_4() (!GPIO_ReadInputDataBit(PORT_KEY4,KEY4_PIN))
   
uint32_t CAN1_STATUS;
uint32_t CAN2_STATUS;


extern uint8_t CANFD_Send_Num ;
extern uint8_t CANFD_Recv_Num ;

extern uint8_t CAN_Send_Num ;
extern uint8_t CAN_Recv_Num ;

extern CAN_RX_MSGOBJ rxObj[];
extern uint8_t rxd[CAN_BUF_NUM][MAX_DATA_BYTES];
extern CAN_TX_MSGOBJ txObj;
extern uint8_t txd[];
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/

volatile uint32_t msTicks;                        /* counts 1ms timeTicks     */
 void SysTick_Handler(void) {

  msTicks++;                        /* increment counter necessary in Delay() */
	
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay_Ms (uint32_t dlyTicks) {
  uint32_t curTicks;
	curTicks = 0;
	msTicks = 0;
//  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}




void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	
	
	RCC_APB2PeriphClockCmd( RCC_KEY1|RCC_KEY2|RCC_KEY3|RCC_KEY4 , ENABLE); 	
	RCC_APB2PeriphClockCmd( RCC_LED1|RCC_LED2|RCC_LED3|RCC_LED4 , ENABLE); 	
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE); 	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	
	
	/**
	*	LED 
	*/					 
	GPIO_InitStructure.GPIO_Pin = LED1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED1, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED2, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED3, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LED4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(PORT_LED4, &GPIO_InitStructure);
	
	/**
	*	USB ENABLE -> PC5
	*/		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC,GPIO_Pin_5);

		/**
	*	DI -> PB12-PPB15
	*/	
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY1, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY2, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY3_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY3, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = KEY4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(PORT_KEY4, &GPIO_InitStructure);

	LED1_ON();
	LED2_ON();
	LED3_ON();
	LED4_ON();

}

void Uart2_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  USART_InitTypeDef USART_InitStructure; 
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
 
  /*
  *  USART2_TX -> PA2 , USART2_RX ->	PA3
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  USART_InitStructure.USART_BaudRate = 115200; 
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure); 
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  USART_ClearFlag(USART2,USART_FLAG_TC);
  
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  		
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  //  启动串口
  USART_Cmd(USART2, ENABLE);   
	
}


/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : \
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t aucRecv_Data_temp;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  			
    { 
			aucRecv_Data_temp = USART2->DR; 
		
		}
}


void USART2_Send(void)
{
	uint8_t i;
	for(i=0;i<5;i++)
	{
		 USART_SendData(USART2, 0x55);
		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
	}
}




/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void)  {

	uint8_t i;
	uint8_t len = 0;
	
	uint8_t pack_num=0;
	uint8_t send_num=0;
	
	uint8_t CAN_FD_Mode = 1;//默认CAN FD模式
	
	SysTick_Config(72000000 / 1000);         /* SysTick 100 usec IRQ       */

	GPIO_Configuration();
	
//	Unused_GPIO_Init_Function();
	


	CAN_GPIO_Config();
	CAN1_Config(1000);//波特率1000K
	CAN1_Filter_Config(0,0,0,0);


	Uart2_Init();
	
	
			// Initialize application
	APP_Initialize();
	APP_CANFDSPI_Init();
	
	while(1)
	{
		
		if(CANFD_Send_Num != CANFD_Recv_Num) //CAN FD 收到数据
		{			
			LED1_ON();

			printf("CANFD to CAN! CANFD RECV  CAN2.0B SEND\r\n");
			
			if(rxObj[CANFD_Send_Num].bF.ctrl.RTR == 0)//数据帧
			{
				CAN1_TxMessage.RTR = DATA_FRAME;
			}
			else //远程帧
			{
				CAN1_TxMessage.RTR = REMOTE_FRAME;
			}
		
			if(rxObj[CANFD_Send_Num].bF.ctrl.IDE == 0)//标准帧
			{
				CAN1_TxMessage.IDE = STANDARD_FORMAT;
					CAN1_TxMessage.StdId = rxObj[CANFD_Send_Num].bF.id.SID;
			}
			else //扩展帧
			{
					CAN1_TxMessage.IDE = EXTENDED_FORMAT;
					CAN1_TxMessage.ExtId = (rxObj[CANFD_Send_Num].bF.id.SID<<18)+ rxObj[CANFD_Send_Num].bF.id.EID;
			}
		
			len = DRV_CANFDSPI_DlcToDataBytes(rxObj[CANFD_Send_Num].bF.ctrl.DLC);
			
			if(len > 0)
			{
				if((len%8)==0)
					pack_num = len/8 ;	//8字节一包
				else
					pack_num = len/8 + 1;	//8字节一包
				
				
				send_num = 0;
				
				do
				{
			
					if(send_num == (pack_num-1))
					{
						CAN1_TxMessage.DLC = len - send_num*8 ;
					}
					else
					{
						CAN1_TxMessage.DLC = 8;
					}
				
					memcpy(CAN1_TxMessage.Data, &rxd[CANFD_Send_Num][send_num*8],CAN1_TxMessage.DLC);
				
					CAN1_wrMsg(CAN1_TxMessage);
					
					send_num++;
					
				}	while(send_num < pack_num);
				
			}
			
			CANFD_Send_Num++;
			if(CANFD_Send_Num >= CAN_BUF_NUM)
				CANFD_Send_Num = 0;
			
			LED1_OFF();
				
		}
		
		if(CAN_Send_Num != CAN_Recv_Num)
		{
			LED2_ON();
			printf("CAN to CANFD! CAN2.0B RECV  CANFD SEND\r\n");
			
				
			
			txObj.bF.ctrl.RTR = 	CAN1_RxMessage[CAN_Send_Num].RTR ;
			
			if(CAN1_RxMessage[CAN_Send_Num].IDE == 0)
			{
				txObj.bF.id.SID  = CAN1_RxMessage[CAN_Send_Num].StdId;
				
				txObj.bF.ctrl.IDE = 0;
			}
			else
			{
				txObj.bF.id.SID = CAN1_RxMessage[CAN_Send_Num].ExtId >>18;
				txObj.bF.id.EID = CAN1_RxMessage[CAN_Send_Num].ExtId;
				
				txObj.bF.ctrl.IDE = 1;
			}
			
			txObj.bF.ctrl.DLC = CAN1_RxMessage[CAN_Send_Num].DLC;
			
	

			if(CAN_FD_Mode == 1) //can fd 模式
			{
				txObj.bF.ctrl.BRS = 1;//1;//隐形表示可变速率1，显性表示不转换速率0
				txObj.bF.ctrl.FDF = 1;//1;//1 FD报文 0 CAN 报文
			}
			else
			{
				txObj.bF.ctrl.BRS = 0;//1;//隐形表示可变速率1，显性表示不转换速率0
				txObj.bF.ctrl.FDF = 0;//1;//1 FD报文 0 CAN 报文
			}
	
			for (i = 0; i <txObj.bF.ctrl.DLC; i++)
			{
					txd[i] = CAN1_RxMessage[CAN_Send_Num].Data[i];
			}	
							
			APP_TransmitMessageQueue();
			
			CAN_Send_Num++;
		if(CAN_Send_Num >= CAN_BUF_NUM)
				CAN_Send_Num = 0;
				
			LED2_OFF();
		}
		
	 }
}


int fputc(int ch,FILE *p) //函数默认的，在使用printf函数时自动调用
{
	USART_SendData(USART2,(u8)ch);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	return ch;
}

