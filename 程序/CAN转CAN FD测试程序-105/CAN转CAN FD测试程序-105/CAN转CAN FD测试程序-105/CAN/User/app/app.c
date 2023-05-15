/*******************************************************************************
  Application:  Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    Implementation of application.

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


// Include files
#include "app.h"
#include "drv_canfdspi_api.h"
#include "drv_spi.h"
#include "stdio.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

APP_DATA appData;

CAN_CONFIG config;
CAN_OPERATION_MODE opMode;

// Transmit objects
CAN_TX_FIFO_CONFIG txConfig;
CAN_TX_FIFO_EVENT txFlags;
CAN_TX_MSGOBJ txObj;
uint8_t txd[MAX_DATA_BYTES];

// Receive objects
CAN_RX_FIFO_CONFIG rxConfig;
REG_CiFLTOBJ fObj;
REG_CiMASK mObj;
CAN_RX_FIFO_EVENT rxFlags;
//CAN_RX_MSGOBJ rxObj;
//uint8_t rxd[MAX_DATA_BYTES];

uint8_t CANFD_Send_Num = 0;
uint8_t CANFD_Recv_Num = 0;

CAN_RX_MSGOBJ rxObj[CAN_BUF_NUM];
uint8_t rxd[CAN_BUF_NUM][MAX_DATA_BYTES];


uint32_t delayCount = APP_LED_TIME;

REG_t reg;

APP_SwitchState lastSwitchState;

APP_Payload payload;

uint8_t ledCount = 0, ledState = 0;

uint8_t i;

CAN_BUS_DIAGNOSTIC busDiagnostics;
uint8_t tec;
uint8_t rec;
CAN_ERROR_STATE errorFlags;


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void APP_LED_Clear(uint8_t led)
{
    switch (led) {
        case 0: 
					LED_Off(LED0);
            break;
        default: break;
    }
}

void APP_LED_Set(uint8_t led)
{
    switch (led) {
        case 0: 
					LED_On(LED0);
            break;
        default: break;
    }
}

void APP_LED_Write(uint8_t led)
{
    uint8_t mask, pin;

    mask = 1;
    pin = 0;
    Nop();

    for (i = 0; i < APP_N_LED; i++, pin++) {
        if (led & mask) {
            // Set LED
            APP_LED_Set(pin);
        } else {
            // Clear LED
            APP_LED_Clear(pin);
        }

        mask = mask << 1;
    }
}

/*******************************************************************************
* Function Name  : EXTI_Configuration
* Description    : Configures the different EXTI lines.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void EXTI_Configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);

  EXTI_ClearITPendingBit(EXTI_Line0);


  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	



  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

void APP_Initialize(void)
{
    DRV_SPI_Initialize();

		GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_APB2PeriphClockCmd( RCC_INT_IN|RCC_INT_TX_IN|RCC_INT_RX_IN|RCC_LED0_PIN|RCC_S1_PIN , ENABLE); 	
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO , ENABLE); 	
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	/**
		*	INT-> PB6 PB8 PB9
		*/					 
		GPIO_InitStructure.GPIO_Pin = INT_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_INT_IN, &GPIO_InitStructure);	
	
		GPIO_InitStructure.GPIO_Pin = INT_TX_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_INT_TX_IN, &GPIO_InitStructure);	
	
		GPIO_InitStructure.GPIO_Pin = INT_RX_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_INT_RX_IN, &GPIO_InitStructure);	
		
		/**
		*	LED
		*/					 
		GPIO_InitStructure.GPIO_Pin =LED0_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_Init(PORT_LED0_PIN, &GPIO_InitStructure);
	

			/**
		*	S1
		*/	
		GPIO_InitStructure.GPIO_Pin = S1_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_S1_PIN, &GPIO_InitStructure);


		EXTI_Configuration();

    // Switch state
    lastSwitchState.S1 = APP_SWITCH_RELEASED;

#ifdef TEST_SPI
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);

    appData.state = APP_STATE_TEST_RAM_ACCESS;

#else
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
#endif

}


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void APP_CANFDSPI_Init(void)
{
    // Reset device
    DRV_CANFDSPI_Reset(DRV_CANFDSPI_INDEX_0);
	
	
    // Enable ECC and initialize RAM
    DRV_CANFDSPI_EccEnable(DRV_CANFDSPI_INDEX_0);

    DRV_CANFDSPI_RamInit(DRV_CANFDSPI_INDEX_0, 0xff);

    // Configure device
    DRV_CANFDSPI_ConfigureObjectReset(&config);
      config.IsoCrcEnable = 1;
      config.StoreInTEF = 0;
	
    DRV_CANFDSPI_Configure(DRV_CANFDSPI_INDEX_0, &config);

    // Setup TX FIFO
    DRV_CANFDSPI_TransmitChannelConfigureObjectReset(&txConfig);
    txConfig.FifoSize = 7;
    txConfig.PayLoadSize = CAN_PLSIZE_64;
    txConfig.TxPriority = 1;

    DRV_CANFDSPI_TransmitChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txConfig);

    // Setup RX FIFO
    DRV_CANFDSPI_ReceiveChannelConfigureObjectReset(&rxConfig);
    rxConfig.FifoSize = 15;
    rxConfig.PayLoadSize = CAN_PLSIZE_64;

    DRV_CANFDSPI_ReceiveChannelConfigure(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxConfig);

    // Setup RX Filter
    fObj.word = 0;
    fObj.bF.SID = 0xda;
    fObj.bF.EXIDE = 0;  //MIDE=1时 exide=1 扩展 exide=0标准
    fObj.bF.EID = 0x00;
		
    DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF);

    // Setup RX Mask
    mObj.word = 0;
    mObj.bF.MSID = 0x0;
    mObj.bF.MIDE = 0; //1是根据exide配置滤波 0是标准帧和扩展帧 Only allow standard IDs
    mObj.bF.MEID = 0x0;

    DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF);

    // Link FIFO and Filter
    DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, APP_RX_FIFO, true);
		
		
    // Setup Bit Time
		//		 DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);
	// DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_5M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
		 DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0,  CAN_1000K_8M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
		 
	
						
    // Setup Transmit and Receive Interrupts
    DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
	#ifdef APP_USE_TX_INT
    DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
	#endif
    DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);

    // Select Normal Mode
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);		//can fd模式
//	DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_CLASSIC_MODE);   //can 2.0模式
		
}



void APP_TransmitMessageQueue(void)
{
    APP_LED_Set(APP_TX_LED);

    uint8_t attempts = MAX_TXQUEUE_ATTEMPTS;

    // Check if FIFO is not full
    do {
        DRV_CANFDSPI_TransmitChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txFlags);
        if (attempts == 0) {
            Nop();
            Nop();
            DRV_CANFDSPI_ErrorCountStateGet(DRV_CANFDSPI_INDEX_0, &tec, &rec, &errorFlags);
            return;
        }
        attempts--;
    }
    while (!(txFlags & CAN_TX_FIFO_NOT_FULL_EVENT));

    // Load message and transmit
    uint8_t n = DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC);

    DRV_CANFDSPI_TransmitChannelLoad(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, &txObj, txd, n, true);

    APP_LED_Clear(APP_TX_LED);
}
/*******************************************************************************
* Function Name  : CAN_Frame_Get
* Description    : This function handles External lines 0 interrupt request.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/


		
void CAN_Frame_Get(void)
{

  if (APP_RX_INT()) 		
	{
		
		while(APP_RX_INT())
		{
								
						// Get message
			DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxObj[CANFD_Recv_Num], &rxd[CANFD_Recv_Num][0], MAX_DATA_BYTES);
				
			CANFD_Recv_Num++;
			
				if(	CANFD_Recv_Num == CAN_BUF_NUM)
				{
					CANFD_Recv_Num =0 ;
				}	
					
		}
	}
}

/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : 接收中断 未使用.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void EXTI0_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line0) != RESET ) {

			EXTI_ClearITPendingBit(EXTI_Line0);
		
			CAN_Frame_Get();
		}
			
}

