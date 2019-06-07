/************************************************************************************//**
* \file         signaltower.h
* \brief        WS2812b driver for signal tower source file.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*  Copyright 2019 (c)  by HAN Automotive   http://www.han.nl        All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* Permission is hereby granted, free of charge, to any person obtaining a copy of this 
* software and associated documentation files (the "Software"), to deal in the Software
* without restriction, including without limitation the rights to use, copy, modify, merge,
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit
* persons to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
* \endinternal
****************************************************************************************/

/*
 *               +----------+
 *      Layer 3  |LED3  LED4|
 *               |          |
 *               |          |
 *               |          |
 *      Layer 2  |LED2  LED5|
 *               |          |
 *               |          |
 *               |          |
 *      Layer 1  |LED1  LED6|
 *               |          |
 *               |          |
 *               |          |
 *      Layer 0  |LED0  LED7|
 *               |          |
 *    ==========/            \
 *  Connection |              |
 *  wires      +--------------+
 */

/****************************************************************************************
* Include files
****************************************************************************************/
#include "signaltower.h"
#include "stdlib.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define BIT_0 (0xC0)
#define BIT_1 (0xFC)

// The number of bytes for each transfer is determined as follows:
// - 8 LEDs
// - 3 colors (RGB) per LED
// - 8 bits per color
// - 2*8 bits to force the line low for at least 9 us, before and after a
//   transfer
#define N_DATA_OUT ((8*3*8) + (2*8))

#define TIMEOUT_MAX (10000)

/****************************************************************************************
* Type definitions
****************************************************************************************/
typedef struct
{
	SPI_TypeDef *spi;
	uint32_t rcc_spi_clk;
	void (*rcc_apb_periph_clock_cmd_spi)(uint32_t, FunctionalState);
	// ------------------------------------------------------------------------
	GPIO_TypeDef *gpio_port;
	uint16_t gpio_mosi_pin;
	uint32_t rcc_gpio_port_clk;
	void (*rcc_apb_periph_clock_cmd_gpio)(uint32_t, FunctionalState);
	// ------------------------------------------------------------------------
	DMA_Channel_TypeDef *dma_channel;
	uint32_t dma_tc_flag;
  
} tSpiChannelMapping;

/****************************************************************************************
* External function prototypes
****************************************************************************************/

/****************************************************************************************
* Function prototypes
****************************************************************************************/
void signaltower_transfer_data(void);

/****************************************************************************************
* Constant data declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a signal tower module. */
const static tSpiChannelMapping channelMapping[] =
{
	{ // idx 0:
		SPI1, RCC_APB2Periph_SPI1, RCC_APB2PeriphClockCmd,
		GPIOA, GPIO_Pin_7, RCC_APB2Periph_GPIOA, RCC_APB2PeriphClockCmd,
		DMA1_Channel3, DMA1_FLAG_TC3
	},
	// ------------------------------------------------------------------------
	{ // idx 1:
		SPI2, RCC_APB1Periph_SPI2, RCC_APB1PeriphClockCmd,
		GPIOB, GPIO_Pin_15, RCC_APB2Periph_GPIOB, RCC_APB2PeriphClockCmd,
		DMA1_Channel5, DMA1_FLAG_TC5
	}
};

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Output buffer for signal tower module. */
uint8_t data_out[N_DATA_OUT] = {0};

/** \brief Pointer to the selected SPI channel. */
static SPI_TypeDef *mSPI = NULL;

/** \brief Pointer to the selected DMA channel. */
static DMA_Channel_TypeDef *mDMA_Channel = NULL;

/** \brief Global flag that indicates DMA transfer has completed. */
static uint32_t mDMA_TC_Flag = 0;

/************************************************************************************//**
** \brief     Initializes the signal tower driver module.
** \param     channel An SPI channel.
** \return    none.
**
****************************************************************************************/
void signaltower_init(uint8_t channel)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;	
	DMA_InitTypeDef  DMA_InitStructure;
	
    /* make sure the channel is valid before using it as an array indexer */
    if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
    {
      ErrCodesSetError(ER_CODE_SIGNAL_TOWER_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    
	mSPI = channelMapping[channel].spi;
	mDMA_Channel = channelMapping[channel].dma_channel;
	mDMA_TC_Flag = channelMapping[channel].dma_tc_flag;
	
	// Enable peripheral clocks
	channelMapping[channel].rcc_apb_periph_clock_cmd_spi(channelMapping[channel].rcc_spi_clk, ENABLE);
	channelMapping[channel].rcc_apb_periph_clock_cmd_gpio(channelMapping[channel].rcc_gpio_port_clk, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	// Configure SPI master pins: MOSI
	GPIO_InitStructure.GPIO_Pin = channelMapping[channel].gpio_mosi_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(channelMapping[channel].gpio_port, &GPIO_InitStructure);
	
	// SPI configuration
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_Init(mSPI, &SPI_InitStructure);

	// Enable SPI
	SPI_Cmd(mSPI, ENABLE);
	
	// Enable SPI Tx DMA trigger
	SPI_I2S_DMACmd(mSPI, SPI_I2S_DMAReq_Tx, ENABLE);
	
	// DMA configuration
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(mSPI->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data_out;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = N_DATA_OUT;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(mDMA_Channel, &DMA_InitStructure);

	signaltower_reset();
}

/************************************************************************************//**
** \brief     Turns off all LEDs of the signal tower.
** \return    none.
**
****************************************************************************************/
void signaltower_reset(void)
{	
	uint32_t cnt=0;

	// If not initialized, return
    if(mSPI == NULL)
	{
		return;
	}
	
	// Set first 8-bytes to zero
	for(uint32_t i=0; i < 8; ++i)
	{
		data_out[cnt++] = 0x00;
	}

	// Prepare data
	for(uint32_t i=0; i < (N_DATA_OUT - (2*8)); ++i)
	{
		data_out[cnt++] = BIT_0;
	}

	// Set last 8-bytes to zero
	for(uint32_t i=0; i < 8; ++i)
	{
		data_out[cnt++] = 0x00;
	}

    // Transfer data	
    signaltower_transfer_data();
}

// 
/************************************************************************************//**
** \brief     Sets the color of all the layers of the signal tower into the global 
**            array data_out.
**            RGB colors are coded into a 32-bit variable: 0x00GGRRBB
** \param     layer0_grb Color for layer 0 (bottom of the signal tower)
** \param     layer1_grb Color for layer 1
** \param     layer2_grb Color for layer 2
** \param     layer3_grb Color for layer 3 (top of the signal tower)
** \return    none.
**
****************************************************************************************/
void signaltower_write(uint32_t layer0_grb, uint32_t layer1_grb, uint32_t layer2_grb, uint32_t layer3_grb)
{
	uint32_t cnt=0;

	// If not initialized, return
    if(mSPI == NULL)
	{
		return;
	}
	
	// Prepare the data
	
	// 8 bytes 0x00
	for(uint32_t i=0; i<8; ++i)
	{
		data_out[cnt++] = 0x00;
	}

	// LED 0: Layer 0
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer0_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}

	// LED 1: Layer 1
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer1_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}

	// LED 2: Layer 2
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer2_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}

	// LED 3: Layer 3
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer3_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}
	
	// LED 4: Layer 3
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer3_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}

	// LED 5: Layer 2
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer2_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}

	// LED 6: Layer 1
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer1_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}

	// LED 7: Layer 0
	for(uint32_t i=0; i<24; ++i)
	{
		if((layer0_grb & (0x00800000 >> i)) == 0)
		{
			data_out[cnt++] = BIT_0;
		}
		else
		{
			data_out[cnt++] = BIT_1;			
		}	
	}
	
	// 8 bytes 0x00
	for(uint32_t i=0; i<8; ++i)
	{
		data_out[cnt++] = 0x00;
	}
		
	signaltower_transfer_data();
}

/************************************************************************************//**
** \brief     Transfers the data through the SPI MOSI pin by starting a DMA transfer.
**            Uses the data_out global array for transmission.
** \return    none.
**
****************************************************************************************/
inline void signaltower_transfer_data(void)
{
	uint32_t timeout;
	DMA_InitTypeDef DMA_InitStructure;

	// Setup DMA transfer
	DMA_Cmd(mDMA_Channel, DISABLE);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(mSPI->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)data_out;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = N_DATA_OUT;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(mDMA_Channel, &DMA_InitStructure);
	
	// Start DMA
	DMA_Cmd(mDMA_Channel, ENABLE);	

	// Make sure that a previous transfer is not ongoing
	timeout = 0;
	while((DMA_GetFlagStatus(mDMA_TC_Flag) == RESET) && (timeout < TIMEOUT_MAX))
	{
		timeout++;
	}
	DMA_ClearFlag(mDMA_TC_Flag);

	// Wait for last data transmitted
	timeout = 0;
	while((SPI_I2S_GetFlagStatus(mSPI, SPI_I2S_FLAG_BSY) == RESET) && (timeout < TIMEOUT_MAX))
	{
		timeout++;
	}
}

/************************************ end of signaltower.c *****************************/
