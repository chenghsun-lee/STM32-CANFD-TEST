/*******************************************************************************
  SPI Driver:  Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_spi.c

  Summary:
    Implementation of MCU specific SPI functions.

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
#include "drv_spi.h"
#include "stm32f10x.h"
#include "stdint.h"


#define RCC_APB2Periph_GPIO_SPI_FLASH_CS      RCC_APB2Periph_GPIOA
#define SPI_FALSH_CS_PORT                     GPIOA
#define SPI_FALSH_CS_PIN                      GPIO_Pin_4

#define SPI_FLASH_CS_LOW()       GPIO_ResetBits(SPI_FALSH_CS_PORT, SPI_FALSH_CS_PIN)
#define SPI_FLASH_CS_HIGH()      GPIO_SetBits(SPI_FALSH_CS_PORT, SPI_FALSH_CS_PIN)

/* Local function prototypes */
inline void spi_master_init(void);
inline int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize);

void DRV_SPI_Initialize(void)
{
	spi_master_init();
}

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
	return spi_master_transfer(SpiTxData, SpiRxData, spiTransferSize);
}

void spi_master_init(void)
{
	
		SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_SPI_FLASH_CS, ENABLE); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	

								  
  GPIO_InitStructure.GPIO_Pin = SPI_FALSH_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_FALSH_CS_PORT, &GPIO_InitStructure);
	
	
//	 GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	
  SPI_FLASH_CS_HIGH();
  /* SPI2 Config -------------------------------------------------------------*/ 								  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; 
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; 
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  /* Enable SPI2 */ 
  SPI_Cmd(SPI1, ENABLE); 
}

//#define USE_SPI_FUNCTIONS

int8_t spi_master_transfer(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
	uint16_t pos = 0;
	
	
		 SPI_FLASH_CS_LOW();
	
	while(pos < spiTransferSize)
	{
				/* Loop while DR register in not emplty */
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

			/* Send byte through the SPI2 peripheral */
			SPI_I2S_SendData(SPI1, SpiTxData[pos]);
						
			/* Wait to receive a byte */
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

			/* Return the byte read from the SPI bus */
				 
			SpiRxData[pos] =  SPI_I2S_ReceiveData(SPI1);
			
			pos++;
		
	}
	
	SPI_FLASH_CS_HIGH();
	
	
	return 0;
}

