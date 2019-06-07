/************************************************************************************//**
* \file         co_node.c
* \brief        CANopen node source file.
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
#include "co_node.h"                                  /* for CANopen node              */
#include "errorHandling.h"
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Priority of the CANopen node task. */
#define CO_NODE_TASK_PRIO             (tskIDLE_PRIORITY + 8)

/** \brief Stack size of the CANopen node task. */
#define CO_NODE_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)

/** \brief Period time (5ms) of the CANopen node task. */
#define CO_NODE_TASK_PERIOD_TICKS     (((portTickType)5000)/portTICK_PERIOD_US)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void CoNodeTask(void *pvParameters);
static void CoNodePdoProcessTimerCallback(xTimerHandle pxTimer);


/****************************************************************************************
* Local declarations
****************************************************************************************/
/** \brief CANopen reset state. */
static CO_NMT_reset_cmd_t coNodeReset = CO_RESET_COMM;

/** \brief CANopen EEPROM object. */
static CO_EE_t CO_EEO;

/** \brief CANopen EEPROM initialization status. */
static CO_ReturnError_t coEeStatus;

/** \brief Holds the CAN channel on which the CANopen node should run. */
static uint8_t coNodeCanChannel;

/** \brief Timer for the PDO processing. This needs to be done every millisecond. */
static xTimerHandle coPdoProcessTimer;


/************************************************************************************//**
** \brief     Initializes the CANopen node module. Note that it is assumed that the CAN
**            driver has been initialized prior to calling this function.
** \param     canChannel Channel on which to run the CANopen node.
** \return    none.
**
****************************************************************************************/
void CoNodeInit(uint8_t canChannel)
{
  portBASE_TYPE result;

  /* store the CAN channel to use */
  coNodeCanChannel = canChannel;

  /* initialize EEPROM - part 1 */
  coEeStatus = CO_EE_init_1(&CO_EEO, (uint8_t*) &CO_OD_EEPROM, sizeof(CO_OD_EEPROM),
                            (uint8_t*) &CO_OD_ROM, sizeof(CO_OD_ROM));

  /* create the application task */
  xTaskCreate(CoNodeTask, "CoNodeTask", CO_NODE_TASK_STACK_SIZE,
              NULL, CO_NODE_TASK_PRIO, NULL);

  /* create the PDO processing 1ms timer */
  coPdoProcessTimer = xTimerCreate("coPdoTmr",
                                   (((portTickType)1000)/portTICK_PERIOD_US),
                                   pdTRUE, NULL, CoNodePdoProcessTimerCallback);
  if (!(coPdoProcessTimer != NULL))
  {
    ErrCodesSetError(ER_CODE_CONODE_PDO_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* start the PDO timer. note that block time must be 0 since it could be that the
   * scheduler hasn't been started yet.
   */
  result = xTimerStart(coPdoProcessTimer, 0);
  if (!(result == pdPASS))
  {
    ErrCodesSetError(ER_CODE_CONODE_PDO_TIMER_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
} /*** end of CoNodeInit ***/


/************************************************************************************//**
** \brief     Obtains the zero based CAN channel index that this module is initialized
**            to operate on.
** \return    CAN channel index.
**
****************************************************************************************/
uint8_t CoNodeGetChannel(void)
{
  return coNodeCanChannel;
} /*** end of CoNodeGetChannel ***/



/************************************************************************************//**
** \brief     Read an 8-bit unsigned value from an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \return    The read data value.
**
****************************************************************************************/
uint8_t CoNodeODReadUINT8(uint16_t index, uint8_t subindex)
{
  uint8_t result = 0;
  uint16_t entryNo;
  uint8_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (uint8_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        result = *dataPtr;
      }
    }
  }

  return result;
} /*** end of CoNodeODReadUINT8 ***/


/************************************************************************************//**
** \brief     Read an 8-bit signed value from an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \return    The read data value.
**
****************************************************************************************/
int8_t CoNodeODReadSINT8(uint16_t index, uint8_t subindex)
{
  int8_t result = 0;
  uint16_t entryNo;
  int8_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (int8_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        result = *dataPtr;
      }
    }
  }

  return result;
} /*** end of CoNodeODReadSINT8 ***/


/************************************************************************************//**
** \brief     Read an 16-bit unsigned value from an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \return    The read data value.
**
****************************************************************************************/
uint16_t CoNodeODReadUINT16(uint16_t index, uint8_t subindex)
{
  uint16_t result = 0;
  uint16_t entryNo;
  uint16_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (uint16_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        result = *dataPtr;
      }
    }
  }

  return result;
} /*** end of CoNodeODReadUINT16 ***/


/************************************************************************************//**
** \brief     Read an 16-bit signed value from an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \return    The read data value.
**
****************************************************************************************/
int16_t CoNodeODReadSINT16(uint16_t index, uint8_t subindex)
{
  int16_t result = 0;
  uint16_t entryNo;
  int16_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (int16_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        result = *dataPtr;
      }
    }
  }

  return result;
} /*** end of CoNodeODReadSINT16 ***/


/************************************************************************************//**
** \brief     Read an 32-bit unsigned value from an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \return    The read data value.
**
****************************************************************************************/
uint32_t CoNodeODReadUINT32(uint16_t index, uint8_t subindex)
{
  uint32_t result = 0;
  uint16_t entryNo;
  uint32_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (uint32_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        result = *dataPtr;
      }
    }
  }

  return result;
} /*** end of CoNodeODReadUINT32 ***/


/************************************************************************************//**
** \brief     Read an 32-bit signed value from an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \return    The read data value.
**
****************************************************************************************/
int32_t CoNodeODReadSINT32(uint16_t index, uint8_t subindex)
{
  int32_t result = 0;
  uint16_t entryNo;
  int32_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (int32_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        result = *dataPtr;
      }
    }
  }

  return result;
} /*** end of CoNodeODReadSINT32 ***/


/************************************************************************************//**
** \brief     Write an 8-bit unsigned value to an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \param     data Value to write.
** \return    none.
**
****************************************************************************************/
void CoNodeODWriteUINT8(uint16_t index, uint8_t subindex, uint8_t data)
{
  uint16_t entryNo;
  uint8_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (uint8_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        *dataPtr = data;
      }
    }
  }
} /*** end of CoNodeODWriteUINT8 ***/


/************************************************************************************//**
** \brief     Write an 8-bit signed value to an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \param     data Value to write.
** \return    none.
**
****************************************************************************************/
void CoNodeODWriteSINT8(uint16_t index, uint8_t subindex, int8_t data)
{
  uint16_t entryNo;
  int8_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (int8_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        *dataPtr = data;
      }
    }
  }
} /*** end of CoNodeODWriteSINT8 ***/


/************************************************************************************//**
** \brief     Write an 16-bit unsigned value to an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \param     data Value to write.
** \return    none.
**
****************************************************************************************/
void CoNodeODWriteUINT16(uint16_t index, uint8_t subindex, uint16_t data)
{
  uint16_t entryNo;
  uint16_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (uint16_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        *dataPtr = data;
      }
    }
  }
} /*** end of CoNodeODWriteUINT16 ***/


/************************************************************************************//**
** \brief     Write an 16-bit signed value to an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \param     data Value to write.
** \return    none.
**
****************************************************************************************/
void CoNodeODWriteSINT16(uint16_t index, uint8_t subindex, int16_t data)
{
  uint16_t entryNo;
  int16_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (int16_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        *dataPtr = data;
      }
    }
  }
} /*** end of CoNodeODWriteSINT16 ***/


/************************************************************************************//**
** \brief     Write an 32-bit unsigned value to an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \param     data Value to write.
** \return    none.
**
****************************************************************************************/
void CoNodeODWriteUINT32(uint16_t index, uint8_t subindex, uint32_t data)
{
  uint16_t entryNo;
  uint32_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (uint32_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        *dataPtr = data;
      }
    }
  }
} /*** end of CoNodeODWriteUINT32 ***/


/************************************************************************************//**
** \brief     Write an 32-bit signed value to an object dictionary entry.
** \param     index Object dictionary index.
** \param     subindex Object dictionary subindex.
** \param     data Value to write.
** \return    none.
**
****************************************************************************************/
void CoNodeODWriteSINT32(uint16_t index, uint8_t subindex, int32_t data)
{
  uint16_t entryNo;
  int32_t *dataPtr;

  /* only continue if actually initialized */
  if (coNodeReset == CO_RESET_NOT)
  {
    /* find object in Object Dictionary */
    entryNo = CO_OD_find(CO->SDO, index);

    /* does object exist in OD? */
    if( (entryNo != 0xFFFF) && (subindex <= CO->SDO->OD[entryNo].maxSubIndex))
    {
      /* access its data pointer */
      dataPtr = (int32_t*) CO_OD_getDataPointer(CO->SDO, entryNo, subindex);
      /* write the value */
      if (dataPtr != NULL)
      {
        *dataPtr = data;
      }
    }
  }
} /*** end of CoNodeODWriteSINT32 ***/


/************************************************************************************//**
** \brief     Periodic CANopen node task. This task drives all the CANopen related
**            I/O and communication.
** \param     pvParamters Pointer to task parameters data structure.
** \return    none.
**
****************************************************************************************/
static void CoNodeTask(void *pvParameters)
{
  portTickType lastWakeTime;
  uint32_t timer1msPrevious;
  uint32_t timer1msCurrent;

  /* initialize to current time */
  lastWakeTime = xTaskGetTickCount();

  /* enter task body */
  for( ;; )
  {
    /* --------------- communication reset requested ----------------------------------- */
    if (coNodeReset == CO_RESET_COMM)
    {
      /* initialize the CANopen stack. this also initializes the CAN driver and leaves it
       * in configuration mode.
       */
      CO_init();
      /* initialize eeprom - part 2 */
      CO_EE_init_2(&CO_EEO, coEeStatus, CO->SDO, CO->em);
      /* initialize timer counter */
      timer1msPrevious = OsGetSystemTime();
      /* synchronize to CAN bus */
      CO_CANsetNormalMode(ADDR_CAN1);
      /* communication reset complete, switch to normal operation mode */
      coNodeReset = CO_RESET_NOT;
    }
    /* --------------- normal operation mode ------------------------------------------- */
    else if (coNodeReset == CO_RESET_NOT)
    {
      /* get current time */
      timer1msCurrent = OsGetSystemTime();
      /* run CANopen process while specifying the passed amount of milliseconds since the
       * last call and updating the reset state variable.
       */
      coNodeReset = CO_process(CO, timer1msCurrent - timer1msPrevious);
      /*  run CANopen EEPROM process */
      CO_EE_process(&CO_EEO);
      /* store current time for delta time calculation the next time around */
      timer1msPrevious = timer1msCurrent;
    }
    /* --------------- complete device reset ------------------------------------------- */
    else if (coNodeReset == CO_RESET_APP)
    {
      /* perform system reset */
      OsSystemReset();
    }
    /* activate this task periodically */
    vTaskDelayUntil(&lastWakeTime, CO_NODE_TASK_PERIOD_TICKS);
  }
} /*** end of CoNodeTask ***/


/************************************************************************************//**
** \brief     Callback function that gets called by the OS each time the period for
**            the PDO processing timer passed.
** \param     pxTimer Handle of the timer that expired.
** \return    none.
**
****************************************************************************************/
static void CoNodePdoProcessTimerCallback(xTimerHandle pxTimer)
{
  /* doublecheck timer handle */
  if (pxTimer == coPdoProcessTimer)
  {
    /* only process PDO's if the CANopen stack is in normal operation */
    if (coNodeReset == CO_RESET_NOT)
    {
      /* process the RPDOs */
      CO_process_RPDO(CO);
      /* process the TPDOs */
      CO_process_TPDO(CO);
    }
  }
} /*** end of CoNodePdoProcessTimerCallback ***/


/*********************************** end of co_node.c **********************************/
