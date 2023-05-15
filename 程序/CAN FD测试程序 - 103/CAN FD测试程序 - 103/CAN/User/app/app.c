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
CAN_RX_MSGOBJ rxObj;
uint8_t rxd[MAX_DATA_BYTES];

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
		*	LED -> PB5
		*/					 
		GPIO_InitStructure.GPIO_Pin =LED0_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
		GPIO_Init(PORT_LED0_PIN, &GPIO_InitStructure);
	

			/**
		*	 -> PB4
		*/	
		GPIO_InitStructure.GPIO_Pin = S1_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
		GPIO_Init(PORT_S1_PIN, &GPIO_InitStructure);


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

void APP_Tasks(void)
{

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            Nop();
            APP_LED_Set(APP_INIT_LED);

            APP_CANFDSPI_Init();

            APP_LED_Clear(APP_INIT_LED);

            appData.state = APP_STATE_INIT_TXOBJ;

            break;
        }

            /* Initialize TX Object */
        case APP_STATE_INIT_TXOBJ:
        {
            // Configure transmit message
            txObj.word[0] = 0;
            txObj.word[1] = 0;

            txObj.bF.id.SID = TX_RESPONSE_ID;
            txObj.bF.id.EID = 0;

            txObj.bF.ctrl.BRS = 1;
            txObj.bF.ctrl.DLC = CAN_DLC_64;
            txObj.bF.ctrl.FDF = 1;
            txObj.bF.ctrl.IDE = 0;

            // Configure message data
            for (i = 0; i < MAX_DATA_BYTES; i++) txd[i] = txObj.bF.id.SID + i;

            appData.state = APP_STATE_FLASH_LEDS;
            break;
        }

            /* Flash all LEDs */
        case APP_STATE_FLASH_LEDS:
        {
            // Delay loop
            Nop();
            Nop();

            // Delay expired, update LEDs and reset delayCount
            if (delayCount == 0) {
                if (ledCount < APP_N_LED) {
                    ledState |= 1 << ledCount;
                } else {
                    ledState = 0;
                }
                Nop();
                APP_LED_Write(ledState);

                ledCount++;
                delayCount = APP_LED_TIME;
            } else {
                delayCount--;
            }

            if (ledCount > (APP_N_LED + 1)) {
                appData.state = APP_STATE_RECEIVE;
            } else {
                appData.state = APP_STATE_FLASH_LEDS;
            }

            break;
        }

            /* Receive a message */
        case APP_STATE_RECEIVE:
        {
            Nop();
            Nop();
            appData.state = APP_ReceiveMessage_Tasks();

            appData.state = APP_STATE_PAYLOAD;
            break;
        }

        case APP_STATE_PAYLOAD:
        {
            Nop();
            Nop();
            APP_PayLoad_Tasks();

            appData.state = APP_STATE_SWITCH_CHANGED;
            break;
        }

        case APP_STATE_SWITCH_CHANGED:
        {
            APP_TransmitSwitchState();
            appData.state = APP_STATE_RECEIVE;

            break;
        }

            /* RAM access test */
        case APP_STATE_TEST_RAM_ACCESS:
        {
            bool passed = APP_TestRamAccess();

            if (!passed) {
                Nop();
            }

            appData.state = APP_STATE_TEST_REGISTER_ACCESS;

            break;
        }

            /* Register access test */
        case APP_STATE_TEST_REGISTER_ACCESS:
        {
            bool passed = APP_TestRamAccess();

            if (!passed) {
                Nop();
            }

            appData.state = APP_STATE_TEST_RAM_ACCESS;

            break;
        }


            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            Nop();
            Nop();
            appData.state = APP_STATE_INIT;

            break;
        }
    }
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
    fObj.bF.EXIDE = 0;
    fObj.bF.EID = 0x00;
		
    DRV_CANFDSPI_FilterObjectConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &fObj.bF);

    // Setup RX Mask
    mObj.word = 0;
    mObj.bF.MSID = 0x0;
    mObj.bF.MIDE = 1; // Only allow standard IDs
    mObj.bF.MEID = 0x0;

    DRV_CANFDSPI_FilterMaskConfigure(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, &mObj.bF);

    // Link FIFO and Filter
    DRV_CANFDSPI_FilterToFifoLink(DRV_CANFDSPI_INDEX_0, CAN_FILTER0, APP_RX_FIFO, true);
		
		
    // Setup Bit Time
   // DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_500K_2M, CAN_SSP_MODE_AUTO, CAN_SYSCLK_40M);
		 DRV_CANFDSPI_BitTimeConfigure(DRV_CANFDSPI_INDEX_0, CAN_1000K_4M/*CAN_250K_500K*/, CAN_SSP_MODE_AUTO, CAN_SYSCLK_20M);\
		 
		 		 
    // Setup Transmit and Receive Interrupts
    DRV_CANFDSPI_GpioModeConfigure(DRV_CANFDSPI_INDEX_0, GPIO_MODE_INT, GPIO_MODE_INT);
	#ifdef APP_USE_TX_INT
    DRV_CANFDSPI_TransmitChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_TX_FIFO, CAN_TX_FIFO_NOT_FULL_EVENT);
	#endif
    DRV_CANFDSPI_ReceiveChannelEventEnable(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, CAN_RX_FIFO_NOT_EMPTY_EVENT);
    DRV_CANFDSPI_ModuleEventEnable(DRV_CANFDSPI_INDEX_0, CAN_TX_EVENT | CAN_RX_EVENT);

    // Select Normal Mode
    DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_NORMAL_MODE);
//	DRV_CANFDSPI_OperationModeSelect(DRV_CANFDSPI_INDEX_0, CAN_CLASSIC_MODE);
		
}




APP_STATES APP_ReceiveMessage_Tasks(void)
{
    APP_STATES nextState;

    // Normally we go to APP_STATE_PAYLOAD
    Nop();
    Nop();
    nextState = APP_STATE_PAYLOAD;

    // Check if FIFO is not empty
#ifdef APP_USE_RX_INT
    if (APP_RX_INT()) {

#else
    DRV_CANFDSPI_ReceiveChannelEventGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxFlags);

    if (rxFlags & CAN_RX_FIFO_NOT_EMPTY_EVENT) {
#endif
//                APP_LED_Set(APP_RX_LED);

        // Get message
        DRV_CANFDSPI_ReceiveMessageGet(DRV_CANFDSPI_INDEX_0, APP_RX_FIFO, &rxObj, rxd, MAX_DATA_BYTES);

        switch (rxObj.bF.id.SID) {
            case TX_REQUEST_ID:

                // Check for TX request command
                Nop();
                Nop();
                txObj.bF.id.SID = TX_RESPONSE_ID;

                txObj.bF.ctrl.DLC = rxObj.bF.ctrl.DLC;
                txObj.bF.ctrl.IDE = rxObj.bF.ctrl.IDE;
                txObj.bF.ctrl.BRS = rxObj.bF.ctrl.BRS;
                txObj.bF.ctrl.FDF = rxObj.bF.ctrl.FDF;

                for (i = 0; i < MAX_DATA_BYTES; i++) txd[i] = rxd[i];

                APP_TransmitMessageQueue();
                break;

            case LED_STATUS_ID:
                // Check for LED command
                APP_LED_Write(rxd[0]);
                break;

            case BUTTON_STATUS_ID:
                // Check for Button Status command
                // This can be used to test two EVBs without a CAN tool

                // S1 turns on D1
                if (rxd[0] & 0x01) {
                    APP_LED_Set(APP_LED_D1);
                } else {
                    APP_LED_Clear(APP_LED_D1);
                }

                break;

            case PAYLOAD_ID:
                // Check for Payload command
                Nop();
                Nop();
                payload.On = rxd[0];
                payload.Dlc = rxd[1];
                if (rxd[2] == 0) payload.Mode = true;
                else payload.Mode = false;
                payload.Counter = 0;
                payload.Delay = rxd[3];
                payload.BRS = rxd[4];

                break;

        }
    }

//        APP_LED_Clear(APP_RX_LED);

    return nextState;
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




void APP_TransmitSwitchState(void)
{
    APP_SwitchState newSwitchState;

    // Check if switch has changed
    Nop();
    newSwitchState.S1 = APP_S1_READ();

    bool switchChanged = newSwitchState.S1 != lastSwitchState.S1;

    if (switchChanged) {
        // Transmit new state
        txObj.bF.id.SID = BUTTON_STATUS_ID;

        txObj.bF.ctrl.DLC = CAN_DLC_1;
        txObj.bF.ctrl.IDE = 0;
        txObj.bF.ctrl.BRS = 1;
        txObj.bF.ctrl.FDF = 1;

        txd[0] = 0;
        if (newSwitchState.S1 == APP_SWITCH_PRESSED) txd[0] += 0x1;

        APP_TransmitMessageQueue();
    }

    lastSwitchState.S1 = newSwitchState.S1;
}



void APP_PayLoad_Tasks(void)
{
    uint8_t n;

    // Send payload?
    if (payload.On) {
        // Delay transmission
        if (delayCount == 0) {
            delayCount = payload.Delay;

            // Prepare data
            Nop();
            Nop();
            txObj.bF.id.SID = TX_RESPONSE_ID;

            txObj.bF.ctrl.DLC = payload.Dlc;
            txObj.bF.ctrl.IDE = 0;
            txObj.bF.ctrl.BRS = payload.BRS;
            txObj.bF.ctrl.FDF = 1;

            n = DRV_CANFDSPI_DlcToDataBytes((CAN_DLC) payload.Dlc);

            if (payload.Mode) {
                // Random data
                for (i = 0; i < n; i++) txd[i] = rand() & 0xff;
            } else {
                // Counter
                for (i = 0; i < n; i++) txd[i] = payload.Counter;
                payload.Counter++;
            }

            APP_TransmitMessageQueue();
        } else {
            delayCount--;
        }
    } else {
        delayCount = 0;
    }
}




bool APP_TestRegisterAccess(void)
{
    // Variables
    uint8_t length;
    bool good = false;

    Nop();

    // Verify read/write with different access length
    // Note: registers can be accessed in multiples of bytes
    for (length = 1; length <= MAX_DATA_BYTES; length++) {
        for (i = 0; i < length; i++) {
            txd[i] = rand() & 0x7f; // Bit 31 of Filter objects is not implemented
            rxd[i] = 0xff;
        }

        Nop();

        // Write data to registers
        DRV_CANFDSPI_WriteByteArray(DRV_CANFDSPI_INDEX_0, cREGADDR_CiFLTOBJ, txd, length);

        // Read data back from registers
        DRV_CANFDSPI_ReadByteArray(DRV_CANFDSPI_INDEX_0, cREGADDR_CiFLTOBJ, rxd, length);

        // Verify
        good = false;
        for (i = 0; i < length; i++) {
            good = txd[i] == rxd[i];

            if (!good) {
                Nop();
                Nop();

                // Data mismatch
                return false;
            }
        }
    }

    Nop();
    Nop();

    return true;
}

//! Test RAM access


bool APP_TestRamAccess(void)
{
    // Variables
    uint8_t length;
    bool good = false;

    Nop();

    // Verify read/write with different access length
    // Note: RAM can only be accessed in multiples of 4 bytes
    for (length = 4; length <= MAX_DATA_BYTES; length += 4) {
        for (i = 0; i < length; i++) {
            txd[i] = rand() & 0xff;
            rxd[i] = 0xff;
        }

        Nop();

        // Write data to RAM
        DRV_CANFDSPI_WriteByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, txd, length);

        // Read data back from RAM
        DRV_CANFDSPI_ReadByteArray(DRV_CANFDSPI_INDEX_0, cRAMADDR_START, rxd, length);

        // Verify
        good = false;
        for (i = 0; i < length; i++) {
            good = txd[i] == rxd[i];

            if (!good) {
                Nop();
                Nop();

                // Data mismatch
                return false;
            }
        }
    }

    return true;
}
