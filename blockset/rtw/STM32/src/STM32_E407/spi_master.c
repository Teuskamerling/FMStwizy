/************************************************************************************//**
* \file         spi_master.c
* \brief        SPI master driver source file.
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

/****************************************************************************************
* Include files
****************************************************************************************/
#include "os.h"                                       /* for operating system          */
#include "spi_master.h"                               /* SPI master driver header file */
#include "digout.h"                                   /* Digital output driver        */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Maximum amount of loops to wait for SPI hardware flags to become available. */
#define SPI_MASTER_MAX_LOOP_RETRY           (5000)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type with channel mapping information. */
typedef struct
{
  SPI_TypeDef* peripheral;
  uint32_t spi_clk;
  uint8_t spi_apb_number;
  uint32_t clk_gpio_port_clk;
  GPIO_TypeDef* clk_gpio_port;
  uint16_t clk_pin;
  uint16_t clk_pin_source;
  uint32_t miso_gpio_port_clk;
  GPIO_TypeDef* miso_gpio_port;
  uint16_t miso_pin;
  uint16_t miso_pin_source;
  uint32_t mosi_gpio_port_clk;
  GPIO_TypeDef* mosi_gpio_port;
  uint16_t mosi_pin;
  uint16_t mosi_pin_source;
  uint8_t gpio_af;
} tSpiMasterChannelMapping;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static uint16_t SpiMasterGetClockPrescaler(tSpiComSpeedCfg speed);


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
const static tSpiMasterChannelMapping channelMapping[] =
{
  /* idx 0: SPI_MASTER_CHANNEL1_PA5_PA6_PB5 */
  { SPI1, RCC_APB2Periph_SPI1, 2, RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_5, GPIO_PinSource5, RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_6, GPIO_PinSource6, RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_5, GPIO_PinSource5, GPIO_AF_SPI1 },
  /* idx 1: SPI_MASTER_CHANNEL2_PB10_PC2_PC3 */
  { SPI2, RCC_APB1Periph_SPI2, 1, RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_10, GPIO_PinSource10, RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_2, GPIO_PinSource2, RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_3, GPIO_PinSource3, GPIO_AF_SPI2 }
};


/************************************************************************************//**
** \brief     Initializes the SPI master driver.
** \param     speed The desired communication speed.
** \param     channel SPI channel identifier.
** \param     polarity Configuration for the clock polarity when idle.
** \param     phase Configuration of the clock edge for sampling a bit value.
** \return    none.
**
****************************************************************************************/
void SpiMasterInit(uint8_t channel, tSpiComSpeedCfg speed, tSpiClkPolarityCfg polarity, tSpiClkPhaseCfg phase)
{
  SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_SPI_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* enable SPI clock */
  if (channelMapping[channel].spi_apb_number == 1)
  {
    RCC_APB1PeriphClockCmd(channelMapping[channel].spi_clk, ENABLE);
  }
  else
  {
    RCC_APB2PeriphClockCmd(channelMapping[channel].spi_clk, ENABLE);
  }
  /* enable GPIO clock for SPI pins */
  RCC_AHB1PeriphClockCmd(channelMapping[channel].clk_gpio_port_clk, ENABLE);
  RCC_AHB1PeriphClockCmd(channelMapping[channel].miso_gpio_port_clk, ENABLE);
  RCC_AHB1PeriphClockCmd(channelMapping[channel].mosi_gpio_port_clk, ENABLE);
  /* connect the SPI master pins to the peripherals alternate function */
  GPIO_PinAFConfig(channelMapping[channel].clk_gpio_port, channelMapping[channel].clk_pin_source, channelMapping[channel].gpio_af);
  GPIO_PinAFConfig(channelMapping[channel].miso_gpio_port, channelMapping[channel].miso_pin_source, channelMapping[channel].gpio_af);
  GPIO_PinAFConfig(channelMapping[channel].mosi_gpio_port, channelMapping[channel].mosi_pin_source, channelMapping[channel].gpio_af);
  /* configure SPI master pins: SCK */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Pin = channelMapping[channel].clk_pin;
  GPIO_Init(channelMapping[channel].clk_gpio_port, &GPIO_InitStructure);
  /* configure SPI master pins: MOSI */
  GPIO_InitStructure.GPIO_Pin = channelMapping[channel].mosi_pin;
  GPIO_Init(channelMapping[channel].mosi_gpio_port, &GPIO_InitStructure);
  /* configure SPI master pins: MIS0 */
  GPIO_InitStructure.GPIO_Pin = channelMapping[channel].miso_pin;
  GPIO_Init(channelMapping[channel].miso_gpio_port, &GPIO_InitStructure);
  /* SPI master configuration */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  /* configure the clock polarity */
  if (polarity == POLARITY_LOW)
  {
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  }
  else
  {
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  }
  /* configure the phase, which determines when a bit is sampled */
  if (phase == PHASE_1ST_EDGE)
  {
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  }
  else
  {
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  }
  /* configure the communication speed */
  SPI_InitStructure.SPI_BaudRatePrescaler = SpiMasterGetClockPrescaler(speed);
  SPI_Init(channelMapping[channel].peripheral, &SPI_InitStructure);
  /* enable SPI master */
  SPI_Cmd(channelMapping[channel].peripheral, ENABLE);
} /*** end of SpiMasterInit ***/


/************************************************************************************//**
** \brief     Transmits a byte via the SPI interface.
** \param     channel SPI channel identifier.
** \param     data Byte value to transmit.
** \return    none.
**
****************************************************************************************/
void SpiMasterTransmit(uint8_t channel, uint8_t data)
{
  uint16_t retryCnt;

  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_SPI_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* wait for SPI master Tx buffer empty */
  retryCnt = 0;
  while (SPI_I2S_GetFlagStatus(channelMapping[channel].peripheral, SPI_I2S_FLAG_TXE) == RESET)
  {
    if (++retryCnt > SPI_MASTER_MAX_LOOP_RETRY)
    {
      ErrCodesSetError(ER_CODE_SPI_TRANSFER_TIMEOUT, ER_PARAM_SEVERITY_CRITICAL, TRUE);
      return;
    }
  }
  /* send SPI master data */
  SPI_I2S_SendData(channelMapping[channel].peripheral, data);
} /*** end of SpiMasterTransmit ***/


/************************************************************************************//**
** \brief     Receives a byte via the SPI interface.
** \param     channel SPI channel identifier.
** \return    Value of the received byte.
**
****************************************************************************************/
uint8_t SpiMasterReceive(uint8_t channel)
{
  uint16_t retryCnt;

  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < sizeof(channelMapping)/sizeof(channelMapping[0])))
  {
    ErrCodesSetError(ER_CODE_SPI_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* wait for SPI master data reception */
  retryCnt = 0;
  while (SPI_I2S_GetFlagStatus(channelMapping[channel].peripheral, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if (++retryCnt > SPI_MASTER_MAX_LOOP_RETRY)
    {
      ErrCodesSetError(ER_CODE_SPI_TRANSFER_TIMEOUT, ER_PARAM_SEVERITY_CRITICAL, TRUE);
      return 0;
    }
  }
  /* read and return the received data byte */
  return (uint8_t) SPI_I2S_ReceiveData(channelMapping[channel].peripheral);
} /*** end of SpiMasterReceive ***/


/************************************************************************************//**
** \brief     Sets the SS pin. If enable is TRUE the pin is set to logic low, selecting
**            the slave. If enable is FALSE the pin is set to logic high, deselecting the
**            slave.
** \param     digout_pinID Pin identifier of the digital output to use.
** \param     enable TRUE to make the pin logic low, FALSE to make the pin logic high.
** \return    none.
**
****************************************************************************************/
void SpiMasterSetSlaveSelect(uint8_t digout_pinID, uint8_t enable)
{
  /* configure the digital output. this is done every time because the digital output
   * for the slave select can change between transfers in case more SPI slaves are
   * connected to the same channel.
   */
  DigoutConfigure(digout_pinID, DIGOUT_CFG_PUSHPULL);
  /* process the request */
  if (enable == TRUE)
  {
    /* make the digital output low to select the SPI slave */
    DigoutSet(digout_pinID, DIGOUT_LOW);
  }
  else
  {
    /* make the digital output high to deselect the SPI slave */
    DigoutSet(digout_pinID, DIGOUT_HIGH);
  }
} /*** end of SpiMasterSlaveSelectHigh ***/


/************************************************************************************//**
** \brief     Utility function to implement a wait. It simply loops for the number of
**            cycles specified. Sometimes slaves require a specific delay time after
**            enabling the slave select pin before data can be sent to it. This function
**            can be used to realize such a delay.
** \param     cycles Number of loop iterations to wait. On the STM32F40x running at
**                   168 MHz and using the CodeSourcery GCC compiler, 240 cycles are
**                   approximately 10us.
** \return    none.
**
****************************************************************************************/
void SpiMasterWait(uint32_t cycles)
{
  /* store wait time as volatile to prevent the compiler from optimizing it */
  volatile uint32_t waitCycles = cycles;

  /* wait in a loop */
  while (waitCycles-- > 0)
  {
    /* do nothing.. */;
  }
} /*** end of SpiMasterDelay ***/


/************************************************************************************//**
** \brief     Utility function to convert the communication speed value to a baudrate
**            prescaler value supported by the STM32 peripheral driver library.
** \return    Baudrate prescaler value.
**
****************************************************************************************/
static uint16_t SpiMasterGetClockPrescaler(tSpiComSpeedCfg speed)
{
  uint16_t prescaler_value;

  switch (speed)
  {
    case COM_SPEED_21000_KHZ:
      prescaler_value = SPI_BaudRatePrescaler_2;
      break;

    case COM_SPEED_10500_KHZ:
      prescaler_value = SPI_BaudRatePrescaler_4;
      break;

    case COM_SPEED_5250_KHZ:
      prescaler_value = SPI_BaudRatePrescaler_8;
      break;

    case COM_SPEED_2625_KHZ:
      prescaler_value = SPI_BaudRatePrescaler_16;
      break;

    case COM_SPEED_1313_KHZ:
      prescaler_value = SPI_BaudRatePrescaler_32;
      break;

    case COM_SPEED_656_KHZ:
      prescaler_value = SPI_BaudRatePrescaler_64;
      break;

    case COM_SPEED_328_KHZ:
      prescaler_value = SPI_BaudRatePrescaler_128;
      break;

    case COM_SPEED_164_KHZ:
    default:
      prescaler_value = SPI_BaudRatePrescaler_256;
      break;
  }

  return prescaler_value;
} /*** end of SpiMasterGetClockPrescaler ***/


/************************************ end of spi_master.c ******************************/


