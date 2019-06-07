/************************************************************************************//**
* \file         spi_master.h
* \brief        SPI master driver header file.
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
#ifndef SPI_MASTER_H
#define SPI_MASTER_H

/****************************************************************************************
* Include files
****************************************************************************************/
#include <stdint.h>                                   /* ANSI C types                  */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* SPI peripheral identifiers. note that these are used as array indexers so make
 * sure the first one has value 0 and the others are increments.
 */
#define SPI_MASTER_CHANNEL1_PA5_PA6_PB5    (0)        /* SPI channel 1 - D13/D12/D11   */
#define SPI_MASTER_CHANNEL2_PB10_PC2_PC3   (1)        /* SPI channel 2 - UEXT 9/7/8    */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Configuration values for the communication speed. It's basically a prescaler
 *         value that is scaled of the APB1 (PCLK1) clock, which is 42Mhz for this system.
 */
typedef enum
{
  COM_SPEED_21000_KHZ,
  COM_SPEED_10500_KHZ,
  COM_SPEED_5250_KHZ,
  COM_SPEED_2625_KHZ,
  COM_SPEED_1313_KHZ,
  COM_SPEED_656_KHZ,
  COM_SPEED_328_KHZ,
  COM_SPEED_164_KHZ
} tSpiComSpeedCfg;


/** \brief Configuration values for the clock polarity. */
typedef enum
{
  POLARITY_LOW,                                 /**< CLK idles low                     */
  POLARITY_HIGH,                                /**< CLK idles high                    */
} tSpiClkPolarityCfg;

/** \brief Configuration values for the clock phase. This determines when a bit is
 *         sampled. A clock period has 2 edges, a rising and a falling one. The order
 *         of the edges depends on the configuration of the clock polatiry. With thi
 *         configurable it is defined on which edge a bit is sampled, either the first
 *         one or the second.
 */
typedef enum
{
  PHASE_1ST_EDGE,                               /**< Sample bit on first CLK edge      */
  PHASE_2ND_EDGE,                               /**< Sample bit on the second CLK edge */
} tSpiClkPhaseCfg;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void    SpiMasterInit(uint8_t channel, tSpiComSpeedCfg speed, tSpiClkPolarityCfg polarity, tSpiClkPhaseCfg phase);
void    SpiMasterTransmit(uint8_t channel, uint8_t data);
uint8_t SpiMasterReceive(uint8_t channel);
void    SpiMasterSetSlaveSelect(uint8_t digout_pinID, uint8_t enable);
void    SpiMasterWait(uint32_t cycles);


#endif /* SPI_MASTER_H */
/********************************* end of spi_master.h *********************************/


