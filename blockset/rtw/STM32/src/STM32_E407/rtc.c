/************************************************************************************//**
* \file         rtc.c
* \brief        RTC driver source file.
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
#include "rtc.h"

/************************************************************************************//**
** \brief     Initializes the RTC clock.
** \return    none.
**
****************************************************************************************/
void InitRtcClock(void){
	RTC_InitTypeDef RTC_InitStructure;
	/*** Clock Enable ***/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // PWR clock must be enabled to access RTC and RTC backup registers

	/*** Unlock RTC Registers ***/
	PWR_BackupAccessCmd(ENABLE);     // Enable access to backup domain (RTC registers, RTC backup data registers, and backup SRAM)
	RTC_WriteProtectionCmd(DISABLE); // Disable RTC register write protection

	/*** RTC Enable ***/
	RCC_LSEConfig(RCC_LSE_ON);              // Enable LSE (32.768 kHz low speed external) crystal
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // Select LSE as RTC source
	RCC_RTCCLKCmd(ENABLE);                  // Enable RTC

	/*** Enter RTC Initialization Mode ***/
	if(RTC_EnterInitMode() == ERROR)
		ErrCodesSetError(ER_CODE_RTC_INIT_FAILED, ER_PARAM_SEVERITY_MINOR, TRUE);

	/*** RTC Configuration ***/
	/* Internal Clock Frequency                       */
	/* (F_RTCCLK) / ((PREDIV_A + 1) * (PREDIV_S + 1)) */
	/* Example: 32768 Hz / (128 * 256) = 1 Hz         */
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_InitStructure.RTC_AsynchPrediv = 127;
	RTC_InitStructure.RTC_SynchPrediv = 255;
	if(RTC_Init(&RTC_InitStructure) == ERROR)
		ErrCodesSetError(ER_CODE_RTC_INIT_FAILED, ER_PARAM_SEVERITY_MINOR, TRUE);
} /*** End of InitRtcClock ***/


/************************************************************************************//**
** \brief     Sets the date and time on the RTC.
** \param	  pointer to RTC_DateTypeDef struct with date
** \param	  pointer to RTC_TimeTypeDef struct with time
** \return    none.
**
****************************************************************************************/
void SetRtcClock(RTC_DateTypeDef* RTC_DateStruct, RTC_TimeTypeDef* RTC_TimeStruct){
	/* Set date first and check for error */
if(RTC_SetDate(RTC_Format_BIN, RTC_DateStruct) == ERROR)
   ErrCodesSetError(ER_CODE_RTC_SET_CLOCK_FAILED, ER_PARAM_SEVERITY_MINOR, TRUE);
	/* Set time and check for error */
if(RTC_SetTime(RTC_Format_BIN, RTC_TimeStruct) == ERROR)
	ErrCodesSetError(ER_CODE_RTC_SET_CLOCK_FAILED, ER_PARAM_SEVERITY_MINOR, TRUE);
} /**** End of SetRtcClock ****/

/************************************ end of rtc.c ************************************/
