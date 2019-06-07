/************************************************************************************//**
* \file         anin.c
* \brief        Analog inputs driver source file.
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
#include "anin.h"                                     /* Analog input driver           */
#include "os.h"                                       /* for operating system          */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f10x.h"                                /* STM32 registers               */
#include "stm32f10x_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Total number of analog input channels */
#define ANIN_MAX_CHANNELS         (16)

/** \brief Invalid index value for array aninChannelCfgInfo */
#define ANIN_INVALID_CHANNEL_IDX  (ANIN_MAX_CHANNELS)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type with pin mapping information. */
typedef struct
{
  uint8_t peripheral;
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t adcChannel;
} tAninPinMapping;

/** \brief Structure type to keep track of a channel configuration. */
typedef struct
{
  uint8_t pinIdx;
  uint8_t configuredFlg;
} tAninChannelCfgInfo;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void    AninInit(void);
static void    AninReinit(void);
static uint8_t AninGetNumConfiguredChannels(void);
static uint8_t AninGetChannelIndex(uint8_t id);


/****************************************************************************************
* Constant data declarations
****************************************************************************************/
const static tAninPinMapping pinMapping[] =
{
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_0, ADC_Channel_10  }, /* 0:  ANIN_PORTC_PIN0 */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_1, ADC_Channel_11  }, /* 1:  ANIN_PORTC_PIN1 */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_2, ADC_Channel_12  }, /* 2:  ANIN_PORTC_PIN2 */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_3, ADC_Channel_13  }, /* 3:  ANIN_PORTC_PIN3 */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_4, ADC_Channel_14  }, /* 4:  ANIN_PORTC_PIN4 */
  { RCC_APB2Periph_GPIOC, GPIOC, GPIO_Pin_5, ADC_Channel_15  }, /* 5:  ANIN_PORTC_PIN5 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_3, ADC_Channel_3   }, /* 6:  ANIN_PORTA_PIN3 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_2, ADC_Channel_2   }, /* 7:  ANIN_PORTA_PIN2 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_0, ADC_Channel_0   }, /* 8:  ANIN_PORTA_PIN0 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_1, ADC_Channel_1   }, /* 9:  ANIN_PORTA_PIN1 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_4, ADC_Channel_4   }, /* 10: ANIN_PORTA_PIN4 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_7, ADC_Channel_7   }, /* 11: ANIN_PORTA_PIN7 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_6, ADC_Channel_6   }, /* 12: ANIN_PORTA_PIN6 */
  { RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_5, ADC_Channel_5   }, /* 13: ANIN_PORTA_PIN5 */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_0, ADC_Channel_8   }, /* 14: ANIN_PORTB_PIN0 */
  { RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_1, ADC_Channel_9   }  /* 15: ANIN_PORTB_PIN1 */
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Array for storing the analog to digital conversion results. */
static uint16_t aninConversionResults[ANIN_MAX_CHANNELS];

/** \brief Flag to determine is the driver was already initialized. */
static uint8_t aninInitialized = FALSE;

/** \brief Array to keep track of a channel's configuration. */
static tAninChannelCfgInfo aninChannelCfgInfo[ANIN_MAX_CHANNELS];

/** \brief Flag to determine if the configuration was changed and requires updating. */
static uint8_t aninConfigurationChangedFlg = FALSE;

/** \brief Variable that holds the period time of ADCconversionTask in OS ticks. This allows
 **         a dynamic override through function ADCconversionTaskSetPeriod. */
static portTickType ADCconversionTaskPeriodTicks = (((portTickType)500*1000)/portTICK_PERIOD_US);


/************************************************************************************//**
** \brief     Initializes the analog inputs driver.
** \return    none.
**
****************************************************************************************/
static void AninInit(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  uint8_t idx;

  /* only do initialization once */
  if (aninInitialized == FALSE)
  {
    /* set flag */
    aninInitialized = TRUE;
    /* init arrays */
    for (idx=0; idx<ANIN_MAX_CHANNELS; idx++)
    {
      aninConversionResults[idx] = 0;
      aninChannelCfgInfo[idx].configuredFlg = FALSE;
    }
    /* PCLK2 is the APB2 clock, ADCCLK = PCLK2/6 = 72/6 = 12MHz */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    /* enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    /* enable ADC1 clock so that we can talk to it */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* do a first time initialization so we can calibrate */
    /* DMA1 channel1 configuration to store the ADC1 conversion results as a ring buffer */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)aninConversionResults;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = ANIN_MAX_CHANNELS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    /* enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    /* put everything back to power-on defaults */
    ADC_DeInit(ADC1);
    /* ADC1 and ADC2 operate independently */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    /* enable the scan conversion so we can convert multiple channels */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    /* don't do continuous conversions - do them on demand */
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    /* start conversion by software, not an external trigger */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    /* conversions are 12 bit - put them in the lower 12 bits of the result */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    /* say how many channels would be used by the sequencer */
    ADC_InitStructure.ADC_NbrOfChannel = ANIN_MAX_CHANNELS;
    /* now do the setup */
    ADC_Init(ADC1, &ADC_InitStructure);
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
    /* enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);
    /* check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));
    /* start ADC1 calibration */
    ADC_StartCalibration(ADC1);
    /* check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));
  }
} /*** end of AninInit ***/


/************************************************************************************//**
** \brief     Reinitializes the analog input driver for a different number of channels.
** \return    none.
**
****************************************************************************************/
static void AninReinit(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  uint8_t channelIdx;
  uint8_t numChannels;

  /* determine number of configured channels */
  numChannels = AninGetNumConfiguredChannels();
  /* DMA1 channel1 configuration to store the ADC1 conversion results as a ring buffer */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)aninConversionResults;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = numChannels;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  /* enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  /* put everything back to power-on defaults */
  ADC_DeInit(ADC1);
  /* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  /* enable the scan conversion so we can convert multiple channels */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  /* don't do continuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  /* start conversion by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  /* conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  /* say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = numChannels;
  /* now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);
  /* configure the channels */
  for (channelIdx=0; channelIdx<numChannels; channelIdx++)
  {
    ADC_RegularChannelConfig(ADC1, pinMapping[aninChannelCfgInfo[channelIdx].pinIdx].adcChannel,
                             channelIdx+1, ADC_SampleTime_41Cycles5);
  }
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  /* enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
} /*** end of AninReinit ***/


/************************************************************************************//**
** \brief     Finds the number of channels that are configured.
** \return    Number of configured channels.
**
****************************************************************************************/
static uint8_t AninGetNumConfiguredChannels(void)
{
  uint8_t count = 0;

  for (count=0; count<ANIN_MAX_CHANNELS; count++)
  {
    if (aninChannelCfgInfo[count].configuredFlg == FALSE)
    {
      break;
    }
  }

  return count;
} /*** end of AninGetNumConfiguredChannels ***/


/************************************************************************************//**
** \brief     Obtains the index into the aninChannelCfgInfo[] array of a specific pin.
** \param     id Pin identifier.
** \return    Index into the aninChannelCfgInfo[] array or ANIN_INVALID_CHANNEL_IDX if
**            not found.
**
****************************************************************************************/
static uint8_t AninGetChannelIndex(uint8_t id)
{
  uint8_t count;
  uint8_t channelIdx = ANIN_INVALID_CHANNEL_IDX;

  /* iterate through the array */
  for (count=0; count<ANIN_MAX_CHANNELS; count++)
  {
    /* is this the one we are looking for? */
    if ( (aninChannelCfgInfo[count].configuredFlg == TRUE) &&
         (aninChannelCfgInfo[count].pinIdx == id) )
    {
      /* store it and stop searching */
      channelIdx = count;
      break;
    }
  }
  /* return the results */
  return channelIdx;
} /*** end of AninGetChannelIndex ***/


/************************************************************************************//**
** \brief     Configures a analog input pin.
** \param     id Pin identifier.
** \return    none.
**
****************************************************************************************/
void AninConfigure(uint8_t id, uint8_t filtered)
{
  GPIO_InitTypeDef gpio_init;
  uint8_t nextFreeChannelIdx;

  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_ANIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* make sure the driver is initialized */
  AninInit();
  /* only continue if a configuration for this pin identifier wasn't already made */
  if (AninGetChannelIndex(id) == ANIN_INVALID_CHANNEL_IDX)
  {
    /* enable the port's peripheral clock */
    RCC_APB2PeriphClockCmd(pinMapping[id].peripheral, ENABLE);
    /* prepare pin configuration */
    gpio_init.GPIO_Pin = pinMapping[id].pin;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode = GPIO_Mode_AIN;
    /* initialize the pin */
    GPIO_Init(pinMapping[id].port, &gpio_init);
    /* determine next free channel index */
    nextFreeChannelIdx = AninGetNumConfiguredChannels();
    /* make sure there is still a free spot in the results array */
    if (!(nextFreeChannelIdx < ANIN_MAX_CHANNELS))
    {
      ErrCodesSetError(ER_CODE_ANIN_NO_FREE_CHANNELS, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* store what pin identifier belongs to the newly configured channel */
    aninChannelCfgInfo[nextFreeChannelIdx].pinIdx = id;
    aninChannelCfgInfo[nextFreeChannelIdx].configuredFlg = TRUE;
    /* request processing of the configuration change */
    aninConfigurationChangedFlg = TRUE;
  }
} /*** end of AninConfigure ***/


/************************************************************************************//**
** \brief     Obtains the conversion result of the analog input pin.
** \param     id Pin identifier.
** \return    12-bit analog to digital conversion result.
**
****************************************************************************************/
uint16_t AninGet(uint8_t id, uint8_t filtered)
{
  uint8_t channelIdx;
  uint16_t result = 0;

  /* make sure the id is valid before using it as an array indexer */
  if (!(id < sizeof(pinMapping)/sizeof(pinMapping[0])))
  {
    ErrCodesSetError(ER_CODE_ANIN_INVALID_PIN, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* if no analog inputs are configured, the driver is not initialized and nothing
   * needs to be done
   */
  if (aninInitialized == TRUE)
  {
    /* find the channel index that belongs to this pin identifier */
    channelIdx = AninGetChannelIndex(id);

    if (channelIdx != ANIN_INVALID_CHANNEL_IDX)
    {
      /* read from the results array where the conversion result was stored by the DMA */
      result = aninConversionResults[channelIdx];
    }
  }
  /* return the result */
  return result;
} /*** end of AninGet ***/


/************************************************************************************//**
** \brief     Starts an analog to digital conversion sequence for all configured
**            channels.
** \return    none.
**
****************************************************************************************/
void AninConvert(void)
{
  /* if no analog inputs are configured, the driver is not initialized and nothing
   * needs to be done
   */
  if (aninInitialized == TRUE)
  {
    /* is a configuration change request pending? */
    if (aninConfigurationChangedFlg == TRUE)
    {
      /* reset flag */
      aninConfigurationChangedFlg = FALSE;
      /* re-initialize the driver for the new configuration */
      AninReinit();
    }
    /* start the conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  }
} /*** end of AninConvert ***/

/************************************************************************************//**
** \brief     Configures the period time of ADCfilterTask.
** \param     period Task period time in microseconds.
** \return    none.
**
****************************************************************************************/
void ADCconversionTaskSetPeriod(uint32_t period)
{
  ADCconversionTaskPeriodTicks = ((portTickType)period/portTICK_PERIOD_US);
} /*** end of ADCconversionTaskSetPeriod ***/

/************************************************************************************//**
** \brief     Task to automatically start conversion and filter the ADC results
** \param     *pvParameters (required by FREERTOS)
** \return    none.
** Ticket ref #71
****************************************************************************************/
void ADCconversionTask(void *pvParameters)
{
  /* Declare local variables */
  portTickType lastWakeTime;

  /* initialize to current time */
  lastWakeTime = xTaskGetTickCount();

  /* enter task body */
  for( ;; )
  {
	/*Start conversion of ADC's (and filtering) */
	ADCconversionTaskFunction();
	/* activate this task periodically, run ten times faster than application */
    vTaskDelayUntil(&lastWakeTime, ADCconversionTaskPeriodTicks);
  } /* End of task body */
  
} /*** End of ADCconversionTask ***/

/************************************************************************************//**
** \brief     Task to filter the ADC results
** \param     none
** \return    none.
** Ticket ref #71
****************************************************************************************/
void ADCconversionTaskFunction(void)
{
	/* Start the analog conversions */
	AninConvert();	
} /*** end of ADCfilterTaskFunction ***/


/************************************ end of anin.c ************************************/


