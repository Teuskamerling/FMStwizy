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
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */

/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Flag to determine if the configuration was changed and requires updating. */
static uint8_t aninConfigurationChangedFlg = FALSE;

/** \brief Flag to determine is the driver was already initialized. */
static uint8_t aninInitialized = FALSE;

/** \brief Variable that holds the period time of ADCconversionTask in OS ticks. This allows
 **         a dynamic override through function ADCconversionTaskSetPeriod. */
static portTickType ADCconversionTaskPeriodTicks = (((portTickType)500*1000)/portTICK_PERIOD_US);

/** \brief Array for storing the analog to digital conversion results. */
static uint16_t aninConversionResults[ANIN_MAX_CHANNELS];

/** \brief Array to keep track of a channel's configuration. */
static tAninChannelCfgInfo aninChannelCfgInfo[ANIN_MAX_CHANNELS];

/** \brief Pointer to the beginning of the AninFilterStruct that holds the information
 * needed for filtering
 */
static AninFilterStruct *pointerToAninFilterData;

/****************************************************************************************
* Macro definitions
****************************************************************************************/


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void    AninInit(void);
static void    AninReinit(void);
static uint8_t AninGetChannelIndex(uint8_t id);
static void    AninConvert(void);
static uint8_t AninGetNumConfiguredChannels(void);
static void    MedianFilter(uint16_t *NewSample, AninFilterStruct *PointerToFilterStruct);
static AninFilterStruct * AninGetPointerToFilterStruct(uint8_t pinid);
static void    InitializeFilterData(void);

/****************************************************************************************
* Constant data declarations
****************************************************************************************/
const static tAninPinMapping pinMapping[] =
{
  { RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_1, ADC_Channel_1  }, /* 0:  ANIN_PORTA_PIN1  */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_0, ADC_Channel_8  }, /* 1:  ANIN_PORTB_PIN0  */
  { RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_1, ADC_Channel_9  }, /* 2:  ANIN_PORTB_PIN1  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_0, ADC_Channel_10 }, /* 3:  ANIN_PORTC_PIN0  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_1, ADC_Channel_11 }, /* 4:  ANIN_PORTC_PIN1  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_2, ADC_Channel_12 }, /* 5:  ANIN_PORTC_PIN2  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_3, ADC_Channel_13 }, /* 6:  ANIN_PORTC_PIN3  */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_4, ADC_Channel_14 }, /* 7:  ANIN_PORTC_PIN4 */
  { RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_5, ADC_Channel_15 }  /* 8:  ANIN_PORTC_PIN5  */
};

/************************************************************************************//**
** \brief     Initializes the analog inputs driver.
** \return    none.
**
****************************************************************************************/
static void AninInit(void)
{
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
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
	  aninChannelCfgInfo[idx].filterFlg = FALSE;
    }
    /* enable the DMA2 clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    /* enable the ADC2 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    /* ADC common init */
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
  }
} /*** end of AninInit ***/


/************************************************************************************//**
** \brief     Reinitializes the analog input driver for a different number of channels.
** \return    none.
**
****************************************************************************************/
static void AninReinit(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  uint8_t channelIdx;
  uint8_t numChannels;

  /* determine number of configured channels */
  numChannels = AninGetNumConfiguredChannels();
  /* DMA2 channel1 stream 2 configuration to store the ADC2 conversion results as a ring buffer */
  DMA_DeInit(DMA2_Stream2);
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC2->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)aninConversionResults;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = numChannels;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream2, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream2, ENABLE);
  /* put everything back to power-on defaults */
  ADC_DeInit();
  /* configure 12-bit resolution */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  /* enable the scan conversion so we can convert multiple channels */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  /* don't do continuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  /* start conversion by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  /* conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  /* say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfConversion = numChannels;
  /* now do the setup */
  ADC_Init(ADC2, &ADC_InitStructure);
  /* configure the channels */
  for (channelIdx=0; channelIdx<numChannels; channelIdx++)
  {
    ADC_RegularChannelConfig(ADC2, pinMapping[aninChannelCfgInfo[channelIdx].pinIdx].adcChannel,
                             channelIdx+1, ADC_SampleTime_56Cycles);
  }
  /* Turn off prefetch to reduce noise by accessing flash, see AN4073 */
  FLASH_PrefetchBufferCmd(DISABLE);
  /* Enable flash instruction & data cache, see AN4073 */
  FLASH_InstructionCacheCmd(ENABLE);
  FLASH_DataCacheCmd(ENABLE);
  /* Set ADCDC1 in the PWR_CR register, see option 1 in AN4073 */
  PWR->CR |= 0x2000; /*13th bit*/
  /* enable DMA request after last transfer */
   ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  /* enable ADC2 DMA */
  ADC_DMACmd(ADC2, ENABLE);
  /* enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
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
** \param     id Pin identifier, filtered indicate whether the result should be filtered
** \return    none.
**
****************************************************************************************/
void AninConfigure(uint8_t id, uint8_t filtered)
{
  GPIO_InitTypeDef GPIO_InitStructure;
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
    RCC_AHB1PeriphClockCmd(pinMapping[id].peripheral, ENABLE);
    /* prepare pin configuration */
    GPIO_InitStructure.GPIO_Pin = pinMapping[id].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    /* initialize the pin */
    GPIO_Init(pinMapping[id].port, &GPIO_InitStructure);
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
	if(filtered > 0){
		aninChannelCfgInfo[nextFreeChannelIdx].filterFlg = TRUE;
	}
	
    /* request processing of the configuration change */
    aninConfigurationChangedFlg = TRUE;
  }
} /*** end of AninConfigure ***/


/************************************************************************************//**
** \brief     Obtains the conversion result of the analog input pin.
** \param     id Pin identifier, filtered indicate whether to return the filtered value
** \return    12-bit analog to digital conversion result.
**
****************************************************************************************/
uint16_t AninGet(uint8_t id, uint8_t filtered)
{
  uint8_t channelIdx;
  uint16_t result = 0;
  AninFilterStruct *pointerToFilterStruct = NULL;
  AninFilterStruct FilterStruct;

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
	if(!filtered)
	{
		/* find the channel index that belongs to this pin identifier */
		channelIdx = AninGetChannelIndex(id);

		if (channelIdx != ANIN_INVALID_CHANNEL_IDX)
			{
			/* read from the results array where the conversion result was stored by the DMA */
			result = aninConversionResults[channelIdx];
			}
	} else {
		/* Check if the struct with filtered data exists by checking if the pointer exists */
		if(pointerToAninFilterData != NULL)
		{
			/* Search through memory area with ADC filter data for pinid == id */
			pointerToFilterStruct = AninGetPointerToFilterStruct(id); 
			if (pointerToFilterStruct != NULL)
			{
				/* read from the struct where the filter results were stored by the median filter */
				FilterStruct = *pointerToFilterStruct;
				/* Calculate the average and write to result */
				result = (uint16_t) (FilterStruct.FilterResult >> LOWPASSFILTERSHIFT);
			}
		} else {
			/* Set error because the pointer wasn't found */
			ErrCodesSetError(ER_CODE_ANIN_POINTER_NOT_FOUND, ER_PARAM_SEVERITY_CRITICAL, TRUE);
		}
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
static void AninConvert(void)
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
    ADC_SoftwareStartConv(ADC2);
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
** \brief     Function to start conversion and filter the ADC results
** \param     none
** \return    none.
** Ticket ref #71
****************************************************************************************/
void ADCconversionTaskFunction(void)
{
	/* Declare local variables */
	uint8_t i, j;
	AninFilterStruct tempFilterStruct;
	uint8_t channelIdx;
	
	/* Start the analog conversions */
	AninConvert();
	/* Only filter result if channels need to be filtered */
	if(nrFilteredADCchannels > 0)
	{
		/* Allocate and initialize the data in the memory if this has not been done yet*/
		if(pointerToAninFilterData == NULL)
		{
			InitializeFilterData(); 
		}
		/* Loop through all channels */
		for(i=0;i<ANIN_MAX_CHANNELS;i++)
		{
			/* Look for channels that need filtering */
			if(aninChannelCfgInfo[i].filterFlg && aninChannelCfgInfo[i].configuredFlg)
			{
				for(j=0;j<nrFilteredADCchannels;j++)
				{
					tempFilterStruct = *(pointerToAninFilterData + j);
					if(tempFilterStruct.pinId == aninChannelCfgInfo[i].pinIdx)
					{
						/*Find the index in the conversion result array*/
						channelIdx = AninGetChannelIndex(aninChannelCfgInfo[i].pinIdx);
						/*Call the MedianFilter */
						MedianFilter(&aninConversionResults[channelIdx],(pointerToAninFilterData + j));
						//break;? no need to continue in the for loop //TODO Jason
					}
				}
			}
		} /* end of for loop */
	}
} /*** end of ADCfilterTaskFunction ***/

/************************************************************************************//**
** \brief     Initializes the data for analog filtering
** \param     none.
** \return    none.
**
****************************************************************************************/
static void InitializeFilterData(void)
{
	AninFilterStruct tempFilterStruct;
	AninFilterStruct *pointerToStructToFill;
	uint8_t i;
	uint8_t InitIndex;
		
	/* Allocate memory for the structs */
	pointerToAninFilterData = pvPortMalloc(sizeof(AninFilterStruct)*(uint16_t)nrFilteredADCchannels);
	/* Set error if the memory couldn't be allocated */
	if(pointerToAninFilterData == NULL)
	{
		ErrCodesSetError(ER_CODE_ANIN_ALLOC_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
	} else {
		/* Put pin id's in the struct */
		/* Point to the first struct in the memory area */
		pointerToStructToFill = pointerToAninFilterData;
		/* Loop through all configured analog inputs and find the ones with filtering */
		for(i=0;i<ANIN_MAX_CHANNELS;i++)
		{
			if(aninChannelCfgInfo[i].filterFlg > 0)
			{
				tempFilterStruct.pinId = aninChannelCfgInfo[i].pinIdx;
				/* Set all sample data to zero and add the sample age */
				for( InitIndex = 0; InitIndex < FILTERSIZE; InitIndex++ )
				{
					tempFilterStruct.Samples[InitIndex]  = InitIndex << 27;
				}
				/* Set the filter result to zero */
				tempFilterStruct.FilterResult = 33;
				/* Write data to the memory */
				*pointerToStructToFill = tempFilterStruct;
				/* Move pointer to the next struct */
				pointerToStructToFill++;
				/* Check if the pointer isn't exceeding the memory allocated for analog filtering */
				if ((uint32_t)pointerToStructToFill > (uint32_t)(pointerToAninFilterData + nrFilteredADCchannels))
				{
					ErrCodesSetError(ER_CODE_ANIN_POINTER_OUT_RANGE, ER_PARAM_SEVERITY_CRITICAL, TRUE);
					vPortFree(pointerToAninFilterData);
					break; /* Stop the for loop to make sure no memory is overwritten */
				}
			}
		} /* end of for loop */
	}
} /*** end of InitializeFilterData ***/

/************************************************************************************//**
** \brief     Returns a pointer to the struct matching the pin id
** \param     pinid, pin identifier to be found.
** \return    pointer to struct.
**
****************************************************************************************/
static AninFilterStruct * AninGetPointerToFilterStruct(uint8_t pinId)
{
	/* Declare variables */
	uint8_t j;
	AninFilterStruct *pointerToFilterStruct;
	AninFilterStruct tempFilterStruct;

	/* Check if global pointer is NULL (set error if it is) */
	if(pointerToAninFilterData != NULL)
	{
		/* Loop through structs and stop when pin id is found */
		for(j=0;j<nrFilteredADCchannels;j++)
		{
			tempFilterStruct = *(pointerToAninFilterData+j);
			if(pinId == tempFilterStruct.pinId)
			{
				pointerToFilterStruct = (pointerToAninFilterData+j);
				return pointerToFilterStruct;
				//break; also possible, to avoid 2 return paths (MISRA-C compliance)
			}
		}
	} else {
		ErrCodesSetError(ER_CODE_ANIN_INVALID_POINTER, ER_PARAM_SEVERITY_CRITICAL, TRUE);
	}
	return pointerToFilterStruct;
} /*** end of AninGetPointerToFilterStruct() ***/



/************************************************************************************//**
** \brief     Using a special median filter to filter ADC samples. Originally designed
			  by Ton Fleuren.
** \param     pointer to NewSample, pointer to FilterOuput (this could be made a return
			  values aswell.
** \return    none.
** Ticket ref #71
****************************************************************************************/
static void MedianFilter(uint16_t *NewSample, AninFilterStruct *PointerToFilterStruct)
{
	/* Declare local variables */
	AninFilterStruct FilterData;
	uint32_t IntNewSample;
	uint8_t Index = 0;
	uint8_t NewSampleIndex = 0;
	
  /* Copy the data from the memory to the local variable */
  FilterData = *PointerToFilterStruct;
  
   //find the element to be removed; top 5 bits equal SampleAgeIndex
   //if the array is properly initialised and maintained, a sample should always be found without overrunning the array!
  while( ( FilterData.Samples[Index] & 0xF8000000 ) != ( FilterData.SampleAgeIndex << 27 ) )
    Index++;

   //if the sample to be removed lays before the end of the FIR filter window
  if( Index < ( ( FILTERSIZE - LOWPASSFILTERSIZE ) / 2 + LOWPASSFILTERSIZE ) )
  { 
    // add the value that is going to be shifted into the window
    FilterData.FilterResult += FilterData.Samples[( FILTERSIZE - LOWPASSFILTERSIZE ) / 2 + LOWPASSFILTERSIZE] & 0x07FFFFFF;
    
    if( Index >= ( FILTERSIZE - LOWPASSFILTERSIZE ) / 2 ) // sample is within FIR filter window
 	  FilterData.FilterResult -= FilterData.Samples[Index] & 0x07FFFFFF; // subtract removed sample
    else // sample is below the FIR filter window
      // subtract sample that is going to be shifted out of window  
      FilterData.FilterResult -= FilterData.Samples[( FILTERSIZE - LOWPASSFILTERSIZE ) / 2] & 0x07FFFFFF;
  }
  
  // remove the old sample by shifting samples after it down
  while( Index++ < FILTERSIZE - 1 )
      FilterData.Samples[Index - 1] = FilterData.Samples[ Index ];
  
  // insert sample with maximum value to prevent buffer overrun if the
  // new sample is larger than all existing samples
  FilterData.Samples[FILTERSIZE - 1] = 0x07FFFFFF;
  
  // limit NewSample to 12 bits
  IntNewSample = *NewSample & 0x0FFF;

  // find place to insert new sample. this could be improved by using a binary search algorithm
  while( ( FilterData.Samples[NewSampleIndex] & 0x07FFFFFF ) < IntNewSample )
    NewSampleIndex++;
  
  if( NewSampleIndex < ( FILTERSIZE - LOWPASSFILTERSIZE ) / 2 + LOWPASSFILTERSIZE )
  { // new sample will be inserted within or before the FIR filter window
    // subtract sample that is going to be shifted out of the window
    FilterData.FilterResult -= FilterData.Samples[( FILTERSIZE - LOWPASSFILTERSIZE ) / 2 + LOWPASSFILTERSIZE - 1] & 0x07FFFFFF;
    
    if( NewSampleIndex >= ( FILTERSIZE - LOWPASSFILTERSIZE ) / 2 ) // sample is within FIR filter window
      FilterData.FilterResult += IntNewSample; // add inserted sample
    else
      // add sample that is going to be shifted into window
      FilterData.FilterResult += FilterData.Samples[( FILTERSIZE - LOWPASSFILTERSIZE ) / 2 - 1] & 0x07FFFFFF;
  }
  
  // shift samples from insertion index up (backwards is easier)
  Index = FILTERSIZE - 1;
  while( Index-- > NewSampleIndex )
    FilterData.Samples[Index + 1] = FilterData.Samples[Index];
    
  // insert new sample
  FilterData.Samples[NewSampleIndex] = IntNewSample | ( FilterData.SampleAgeIndex << 27 );
    
  if( ++FilterData.SampleAgeIndex >= FILTERSIZE )
    FilterData.SampleAgeIndex = 0;
    
  /* Copy data back to the memory block */
  *PointerToFilterStruct = FilterData;
} /*** End of MedianFilter ***/
/************************************ end of anin.c ************************************/


