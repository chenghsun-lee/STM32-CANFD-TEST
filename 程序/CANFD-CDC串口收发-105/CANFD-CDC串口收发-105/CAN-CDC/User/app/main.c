/*******************************************************************************
  Main:  Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Implementation of main().

  Description:
    .
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and 
any derivatives exclusively with Microchip products. It is your responsibility 
to comply with third party license terms applicable to your use of third party 
software (including open source software) that may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER EXPRESS, 
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES 
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE.

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER 
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF 
THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED 
BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID 
DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
//DOM-IGNORE-END

/*
* Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
*/


/*------------------------------------------------------------------------------
 *      RL-ARM - USB
 *------------------------------------------------------------------------------
 *      Name:    USBD_Demo.c
 *      Purpose: USB Device Demonstration
 *      Rev.:    V4.70
 *------------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2013 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <RTL.h>
#include <rl_usb.h>
#include <asf.h>
#include "app.h"
#include <stdio.h>


void IO_Init (void);
extern	volatile uint32_t APP_Rx_ptr_in ;
extern char APP_Rx_Buffer  [1024];
extern char APP_Tx_Buffer  [1024];
void CAN_FD_TX_Process(uint32_t len);

void	APP_Initialize(void);
void	APP_Tasks(void);

//预留的引脚测试使用
void Unused_GPIO_Init_Function(void)
{
 
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE );
    PWR_BackupAccessCmd( ENABLE );/* 允许修改RTC和后备寄存器*/
    RCC_LSEConfig( RCC_LSE_OFF ); /* 关闭外部低速时钟,PC14+PC15可以用作普通IO*/
    BKP_TamperPinCmd(DISABLE);  /* 关闭入侵检测功能,PC13可以用作普通IO*/

	  GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_3 | GPIO_Pin_4|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_13 |GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
		GPIO_SetBits(GPIOA,GPIO_Pin_8);
		GPIO_SetBits(GPIOA,GPIO_Pin_9);
		GPIO_SetBits(GPIOA,GPIO_Pin_10);
		
		
		GPIO_SetBits(GPIOB,GPIO_Pin_0);
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
		GPIO_SetBits(GPIOB,GPIO_Pin_3);
		GPIO_SetBits(GPIOB,GPIO_Pin_4);
		GPIO_SetBits(GPIOB,GPIO_Pin_7);
		GPIO_SetBits(GPIOB,GPIO_Pin_10);
		GPIO_SetBits(GPIOB,GPIO_Pin_11);
		
		
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
		GPIO_SetBits(GPIOC,GPIO_Pin_6);
		GPIO_SetBits(GPIOC,GPIO_Pin_7);
		GPIO_SetBits(GPIOC,GPIO_Pin_8);
		GPIO_SetBits(GPIOC,GPIO_Pin_9);
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		GPIO_SetBits(GPIOC,GPIO_Pin_14);
		GPIO_SetBits(GPIOC,GPIO_Pin_15);
		
		
		
		
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
		GPIO_ResetBits(GPIOA,GPIO_Pin_9);
		GPIO_ResetBits(GPIOA,GPIO_Pin_10);
		
		
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);
		GPIO_ResetBits(GPIOB,GPIO_Pin_1);
		GPIO_ResetBits(GPIOB,GPIO_Pin_3);
		GPIO_ResetBits(GPIOB,GPIO_Pin_4);
		GPIO_ResetBits(GPIOB,GPIO_Pin_7);
		GPIO_ResetBits(GPIOB,GPIO_Pin_10);
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);
		
		
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
		GPIO_ResetBits(GPIOC,GPIO_Pin_6);
		GPIO_ResetBits(GPIOC,GPIO_Pin_7);
		GPIO_ResetBits(GPIOC,GPIO_Pin_8);
		GPIO_ResetBits(GPIOC,GPIO_Pin_9);
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		GPIO_ResetBits(GPIOC,GPIO_Pin_14);
		GPIO_ResetBits(GPIOC,GPIO_Pin_15);
		
		
		

}


void IO_Init (void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
  /* !< At this stage the microcontroller clock setting is already configured,
   * this is done through SystemInit() function which is called from startup
   * file (startup_stm32fxxx_xx.s) before to branch to application main. To
   * reconfigure the default setting of SystemInit() function, refer to
   * system_stm32fxxx.c file */
	
	 	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO , ENABLE); 	
	
 
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	
	
  	/* Configure the USB connect IO */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	
		GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		GPIO_ResetBits(GPIOC,GPIO_Pin_5);

}

uint8_t Packet_Len = 0;

int main (void) {
  int32_t usb_rx_ch = -1;
  int32_t usb_tx_ch = -1;
	
	IO_Init();
	
//	Unused_GPIO_Init_Function();
	
  usbd_init();                          /* USB Device Initialization          */
  usbd_connect(__TRUE);                 /* USB Device Connect                 */
  while (!usbd_configured ());          /* Wait for device to configure       */
	
	APP_Initialize();

  while (1) {                           /* Loop forever                       */

			APP_Tasks();
                                        
		/* USB -> UART                        */
	usb_rx_ch = USBD_CDC_ACM_DataAvailable(); //USBD_CDC_ACM_GetChar ();
     
		if (usb_rx_ch != 0) {
			 USBD_CDC_ACM_DataRead((uint8_t*)&APP_Tx_Buffer[Packet_Len],usb_rx_ch);
			Packet_Len = Packet_Len+usb_rx_ch;
			if(Packet_Len>=72)
			{
				Packet_Len = 72;
				CAN_FD_TX_Process(Packet_Len);
				Packet_Len = 0;
			}
       usb_rx_ch = 0;
      
    }
		
		

                                        /* UART -> USB                        */
    if (usb_tx_ch == -1) {
      usb_tx_ch = APP_Rx_ptr_in;//UART_GetChar ();
			APP_Rx_ptr_in = -1;
    }
    if (usb_tx_ch != -1) {
			
		 if (USBD_CDC_ACM_DataSend ((uint8_t*)APP_Rx_Buffer,usb_tx_ch) == usb_tx_ch) 	
		 {
			  usb_tx_ch = -1;
		 }

    }
  }
}

