/************************************************************************************//**
* \file         app.c
* \brief        Application source file.
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
* DEALINGS IN THE SOFTWARE.* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "model.h"                                    /* Simulink model interface      */
#include "os.h"                                       /* for operating system          */
#include "app.h"                                      /* for application               */
#include "errorcodes.h"                               /* error codes module            */
#include "eeprom.h"                                   /* EEPROM driver                 */
#include "eeprom_sim.h"                               /* simulated EEPROM  driver      */
#include "anin.h"                                     /* Analog input driver           */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define APP_CTRL_TASK_PRIO             		 (tskIDLE_PRIORITY + 6)
/** \brief Priority and stacksize for the task that does the analog conversion */
#define ADC_CONVERSION_TASK_PRIO             (tskIDLE_PRIORITY + 5)
#define ADC_CONVERSION_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Variable that holds the period time of AppCtrlTask in OS ticks. This allows
 *         a dynamic override through function AppCtrlTaskSetPeriod.
 */
static portTickType appCtrlTaskPeriodTicks = (((portTickType)500*1000)/portTICK_PERIOD_US);

/** \brief Variable that holds the stack size of AppCtrlTask in word elements, not bytes.
 *         this allows a dynamic override through function AppCtrlTaskSetStackSize. It
 *         needs a bit of extra for FATFS because this one is configured to support LFN
 *         with a dynamic working buffer on the STACK.
 */
static uint16_t appCtrlTaskStackSize = configMINIMAL_STACK_SIZE + 150;


/************************************************************************************//**
** \brief     Initializes the program's application.
** \return    none.
**
****************************************************************************************/
void AppInit(void)
{
  /* initialize the eeprom driver */
  EepromInit();
  /* initialize the error handler */
  ErrCodesInit();
  /* initialize the Simulink model */
  ModelInit();
  /* Run the ADC conversion (and filter) once to ensure it is finished when AppCtrlTask starts */
  ADCconversionTaskFunction();
  /* create the ADC conversion task */
  xTaskCreate(ADCconversionTask, "ADCconversionTask", ADC_CONVERSION_TASK_STACK_SIZE,
              NULL, ADC_CONVERSION_TASK_PRIO, NULL);
  /* create the application task */
  xTaskCreate(AppCtrlTask, "AppCtrlTask", appCtrlTaskStackSize,
              NULL, APP_CTRL_TASK_PRIO, NULL);
} /*** end of AppInit ***/


/************************************************************************************//**
** \brief     Configures the period time of AppCtrlTask.
** \param     period Task period time in microseconds.
** \return    none.
**
****************************************************************************************/
void AppCtrlTaskSetPeriod(uint32_t period)
{
  appCtrlTaskPeriodTicks = ((portTickType)period/portTICK_PERIOD_US);
} /*** end of AppCtrlTaskSetPeriod ***/


/************************************************************************************//**
** \brief     Configures the stack size of AppCtrlTask.
** \attention This function only works if it is called before the task creation in
**            AppInit().
** \param     stackSize Stack size in bytes.
** \return    none.
**
****************************************************************************************/
void AppCtrlTaskSetStackSize(uint16_t stackSize)
{
  /* set the desired stack size */
  appCtrlTaskStackSize = stackSize / sizeof(portBASE_TYPE);
  /* make sure it is not too small */
  if (appCtrlTaskStackSize < configMINIMAL_STACK_SIZE)
  {
    appCtrlTaskStackSize = configMINIMAL_STACK_SIZE;
  }
} /*** end of AppCtrlTaskSetStackSize ***/


/************************************************************************************//**
** \brief     Periodic application control task.
** \param     pvParamters Pointer to task parameters data structure.
** \return    none.
**
****************************************************************************************/
void AppCtrlTask(void *pvParameters)
{
  portTickType lastWakeTime;

  /* initialize to current time */
  lastWakeTime = xTaskGetTickCount();

  /* enter task body */
  for( ;; )
  {
    /* update the Simulink model at the fixed step time */
    ModelStep();
    /* activate this task periodically */
    vTaskDelayUntil(&lastWakeTime, appCtrlTaskPeriodTicks);
  }
} /*** end of AppCtrlTask ***/


/*********************************** end of app.c **************************************/
