/************************************************************************************//**
* \file         os.c
* \brief        Operating system source file.
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
#include <string.h>                                   /* ANSI C string lib             */
#include <ctype.h>                                    /* ANSI C C-type defs            */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "os.h"                                       /* for operating system          */
#include "app.h"                                      /* for application               */
#include "anin.h"                                     /* Analog input driver           */
#include "quadencoder.h"                              /* quadrature encoder driver     */
#include "reset.h"                                    /* microcontroller reset         */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Configures to measurement time over which the CPU load is calculated. */
#define OS_CPU_LOAD_TIMER_PERIOD_MS  (1000)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type for the task id to name mapping queue entry. */
typedef struct t_task_id_mapping_q_entry
{
  struct t_task_id_mapping_q_entry *prevQEntry;      /**< pointer to prev. queue entry */
  void *handle;                                      /**< task handle                  */
  char *name;                                        /**< task name                    */
  unsigned portBASE_TYPE prio;                       /**< task priority                */
  struct t_task_id_mapping_q_entry *nextQEntry;      /**< pointer to next queue entry  */
} tOsTaskIdMappingQEntry;

/** \brief Structure type with run-time measurement information. */
typedef struct
{
  uint8_t  taskRunningFlag;                          /**< task running flag.           */
  uint32_t taskRuntimeCount;                         /**< task runtime counter.        */
  uint8_t  measurementPaused;                        /**< paused information.          */
} tOsRunTimeMeasurementCtrl;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void OsInit(void);
static void OsCpuLoadTimerCallback(xTimerHandle pxTimer);
static tOsTaskIdMappingQEntry* OsTaskIdMappingQueueFindLast(void);
static tOsTaskIdMappingQEntry* OsTaskIdMappingQueueFindByHandle(void *handle);
static void *OsGetTaskHandleByName(char *name);
static void OsTaskIdMappingQueueAdd(void *handle, char *name, unsigned portBASE_TYPE prio);
static void OsTaskIdMappingQueueDelete(void *handle);
static uint8_t OsCompareTaskNames(char *name1, char *name2);
static void OsIdleRunTimeMeasurementInit(void);
static void OsIdleRunTimeMeasurementPauseFromISR(void);
static void OsIdleRunTimeMeasurementResumeFromISR(void);
static void OsIdleRunTimeMeasurementReset(void);
static uint32_t OsIdleRunTimeMeasurementGet(void);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Linked list with an overview of all created tasks TCB, priority and name. */
static tOsTaskIdMappingQEntry *osTaskIdMappingQueue;

/** \brief Variable to keep track of the idle task run time measurement. */
static tOsRunTimeMeasurementCtrl osIdleRunTimeMeasurementCtrl;

/** \brief Periodic timer to calculate the CPU load based on the idle run time
 *  measurement.*/
static xTimerHandle osCpuLoadTimer;

/** \brief Holds the current value of the CPU load. */
static uint8_t cpuLoadCurrent;

/** \brief Holds the maximum value of the CPU load. */
static uint8_t cpuLoadMax;

/** \brief Holds the number of milliseconds that the system has been running. */
static uint32_t osSystemTimeMs = 0;


/************************************************************************************//**
** \brief     This is the entry point for the software program and is called by the
**            reset ISR after the microcontroller is initialized.
** \return    program return code.
**
****************************************************************************************/
int main(void)
{
  /* initialize the operating system */
  OsInit();
  /* initialize the application */
  AppInit();
  /* start the scheduler */
  vTaskStartScheduler();
  /* code should never get here */
  for (;;)
  {
    ;
  }
  return 0;
} /*** end of main ***/


/****************************************************************************************
** \brief     Informs the OS that an interrupt is being processed.
** \return    none.
**
****************************************************************************************/
void OsIrqEnter(void)
{
  /* pause the idle task measurement */
  OsIdleRunTimeMeasurementPauseFromISR();
} /*** end of OsIrqEnter ***/


/****************************************************************************************
** \brief     Informs the OS that an interrupt processing just finished.
** \return    none.
**
****************************************************************************************/
void OsIrqExit(void)
{
  /* resume the idle task measurement */
  OsIdleRunTimeMeasurementResumeFromISR();
} /*** end of OsIrqEnter ***/


/************************************************************************************//**
** \brief     Enters a critical section and returns the interrupt state. This function
**            must always be used in a pair together with OsLeaveCriticalSection in the
**            following way:
**              uint32_t saved_cs_state;
**
**              saved_cs_state = OsEnterCriticalSection();
**              ...do critical section stuff...
**              OsLeaveCriticalSection(saved_cs_state);
**
**            Using these functions instead of taskENTER_CRITICAL()/taskEXIT_CRITICAL()
**            is safer because they do not automatically enabled the interrupts during
**            the last exit of the critical section. This means that the OsEnter/Exit
**            functions can be used during an ISR as well.
** \return    The state of the interrupts that can be disabled by the operating in a
**            critical section.
**
****************************************************************************************/
uint32_t OsEnterCriticalSection(void)
{
  return portSET_INTERRUPT_MASK_FROM_ISR();
} /*** end of OsEnterCriticalSection ***/


/************************************************************************************//**
** \brief     Leave a critical section and restores the interrupt state. This function
**            is the counter part to OsEnterCriticalSection and must always be used as
**            a pair.
** \param     old_cs_state. The value of the interrupt state.
** \return    none.
**
****************************************************************************************/
void OsLeaveCriticalSection(uint32_t old_cs_state)
{
  portCLEAR_INTERRUPT_MASK_FROM_ISR(old_cs_state);
} /*** end of OsLeaveCriticalSection ***/


/************************************************************************************//**
** \brief     Resets the microcontroller.
** \return    none.
**
****************************************************************************************/
void OsSystemReset(void)
{
  /* perform software reset */
  ResetMicrocontroller();
} /*** end of OsSystemReset ***/


/************************************************************************************//**
** \brief     Obtains the result of the most recent CPU load measurement.
** \return    CPU load as a percentage.
**
****************************************************************************************/
uint8_t OsGetCpuLoad(void)
{
  return cpuLoadCurrent;
} /*** end of OsGetCpuLoad ***/


/************************************************************************************//**
** \brief     Obtains the result of the maximum CPU load that was measured since the
**            start of the OS.
** \return    Max CPU load as a percentage.
**
****************************************************************************************/
uint8_t OsGetCpuLoadMax(void)
{
  return cpuLoadMax;
} /*** end of OsGetCpuLoadMax ***/


/************************************************************************************//**
** \brief     Obtains the number of bytes that are still available on the heap.
** \return    Unused heap space in bytes.
**
****************************************************************************************/
uint32_t OsGetFreeHeapSize(void)
{
  return xPortGetFreeHeapSize();
} /*** end of OsGetFreeHeapSize ***/


/************************************************************************************//**
** \brief     Obtains the number of bytes that are still available on the task's stack.
** \param     name Name of the task as used during task creation.
** \return    Unused stack space in bytes.
**
****************************************************************************************/
uint32_t OsGetTaskFreeStackSize(char *name)
{
  xTaskHandle taskHandle;
  uint32_t freeStackSize = 0;

  taskHandle = OsGetTaskHandleByName(name);

  if (taskHandle != NULL)
  {
    freeStackSize = uxTaskGetStackHighWaterMark(taskHandle) * sizeof(portBASE_TYPE);
  }
  return freeStackSize;
} /*** end of OsGetTaskFreeStackSize ***/


/************************************************************************************//**
** \brief     Callback that gets called each time a task was created.
** \param     handle Handle of the task that was created. Typically the same as its TCB.
** \param     name Name of the task.
** \param     prio Task priority.
** \return    none.
**
****************************************************************************************/
void OsTaskCreatedCallback(xTaskHandle taskHandle, char *name, unsigned portBASE_TYPE prio)
{
  /* add this task to the queue */
  OsTaskIdMappingQueueAdd(taskHandle, name, prio);
} /*** end of OsTaskCreatedCallback ***/


/************************************************************************************//**
** \brief     Callback that gets called each time a task was deleted.
** \param     handle Handle of the task that was deleted. Typically the same as its TCB.
** \return    none.
**
****************************************************************************************/
void OsTaskDeletedCallback(xTaskHandle taskHandle)
{
  /* remove this task from the queue */
  OsTaskIdMappingQueueDelete(taskHandle);
} /*** end of OsTaskDeletedCallback ***/


/************************************************************************************//**
** \brief     Obtains the number of milliseconds that the operating system is running.
** \return    System time in milliseconds.
**
****************************************************************************************/
uint32_t OsGetSystemTime(void)
{
  return osSystemTimeMs;
} /*** end of OsGetSystemTime ***/


/************************************************************************************//**
** \brief     Callback that gets called each time the OS tick time elapsed.
** \return    none.
**
****************************************************************************************/
void vApplicationTickHook(void)
{
  static uint16_t ticksCounter = 0;

  /* update the system time for a situation where the tick time is >= 1 ms */
  if (portTICK_PERIOD_US >= 1000)
  {
    osSystemTimeMs += portTICK_PERIOD_US / 1000;
  }
  /* update the system time for a situation where the tick time is < 1 ms */
  else
  {
    /* increment the ticks counter */
    ticksCounter++;
    /* did one millisecond pass? */
    if (ticksCounter >= (1000/portTICK_PERIOD_US))
    {
      osSystemTimeMs++;
      /* reset the ticks counter for measuring the next millisecond interval */
      ticksCounter = 0;
    }
  }
  /* start a new analog to digital conversion sequence */
  //AninConvert();
  /* update the quadrature encoder counters */
  QuadEncUpdate();
} /*** end of vApplicationTickHook ***/


#ifdef configASSERT
/************************************************************************************//**
** \brief     Called when a runtime assertion failed. It stores information about where
**            the assertion occurred and halts the software program.
** \param     file   Name of the source file where the assertion occurred.
** \param     line   Linenumber in the source file where the assertion occurred.
** \return    none
**
****************************************************************************************/
void AssertFailure(char *file, uint32_t line)
{
  /* store it as a generic system error */
  ErrCodesSetError(ER_CODE_GENERIC_SYSTEM_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
} /*** end of AssertFailure ***/
#endif /* configASSERT */


/************************************************************************************//**
** \brief     Initializes the mapping queue.
** \return    none.
**
****************************************************************************************/
static void OsInit(void)
{
  portBASE_TYPE result;

  /* init queue to empty */
  osTaskIdMappingQueue = NULL;
  /* initialize the run time measurement part that is used to determine CPU load */
  OsIdleRunTimeMeasurementInit();

  /* create the CPU load timer */
  osCpuLoadTimer = xTimerCreate("TmrCpuLoad",
                                (((portTickType)OS_CPU_LOAD_TIMER_PERIOD_MS*1000)/portTICK_PERIOD_US),
                                pdTRUE, (void *) 0,
                                OsCpuLoadTimerCallback);
  configASSERT(osCpuLoadTimer != NULL);

  /* reset CPU load  */
  cpuLoadCurrent = 0;
  cpuLoadMax = 0;
  /* start the timer. note that block time must be 0 since scheduler hasn't been started
   * yet.
   */
  result = xTimerStart(osCpuLoadTimer, 0 );
  configASSERT(result == pdPASS);

  /* reset the system timer */
  osSystemTimeMs = 0;
} /*** end of OsInit ***/


/************************************************************************************//**
** \brief     Callback function that gets called by the OS each time the period for
**            CPU load timer expired.
** \param     pxTimer Handler of the timer that expired.
** \return    none.
**
****************************************************************************************/
static void OsCpuLoadTimerCallback(xTimerHandle pxTimer)
{
  uint32_t totalTicks;
  uint32_t idleTicks;

  /* calculate how many ticks were possible during the last period */
  totalTicks = (OS_RUNTIME_COUNTER_FREQUENCY * OS_CPU_LOAD_TIMER_PERIOD_MS) / 1000;
  /* read how many ticks of this the idle task was running */
  idleTicks = OsIdleRunTimeMeasurementGet();
  /* calculate the CPU load during the last period */
  cpuLoadCurrent = ((totalTicks - idleTicks) * 100) / totalTicks;
  /* keep track of max CPU load */
  if (cpuLoadCurrent > cpuLoadMax)
  {
    cpuLoadMax = cpuLoadCurrent;
  }
  /* reset timer for the next measurement */
  OsIdleRunTimeMeasurementReset();
} /*** end of OsCpuLoadTimerCallback ***/


/************************************************************************************//**
** \brief     Finds the last entry in the queue and returns its pointer.
** \return    Pointer to the last entry in the queue or NULL if the queue is empty.
**
****************************************************************************************/
static tOsTaskIdMappingQEntry* OsTaskIdMappingQueueFindLast(void)
{
  tOsTaskIdMappingQEntry *lastQEntry;
  uint32_t saved_cs_state;


  /* can only search the queue if it is not empty */
  saved_cs_state = OsEnterCriticalSection();
  if (osTaskIdMappingQueue == NULL)
  {
    OsLeaveCriticalSection(saved_cs_state);
    return NULL;
  }

  /* find the last element in the list */
  lastQEntry = osTaskIdMappingQueue;
  while (lastQEntry->nextQEntry != NULL)
  {
    lastQEntry = lastQEntry->nextQEntry;
  }
  OsLeaveCriticalSection(saved_cs_state);
  return lastQEntry;
} /*** end of OsTaskIdMappingQueueFindLast ***/


/************************************************************************************//**
** \brief     Finds the queue entry for a task with a specific handle.
** \param     handle Handle of the task to find. Typically the same as its TCB.
** \return    Pointer to its queue entry or NULL is not present.
**
****************************************************************************************/
static tOsTaskIdMappingQEntry* OsTaskIdMappingQueueFindByHandle(void *handle)
{
  tOsTaskIdMappingQEntry *pQEntry;
  tOsTaskIdMappingQEntry *foundQEntry = NULL;
  uint32_t saved_cs_state;

  /* can only search the queue if it is not empty */
  saved_cs_state = OsEnterCriticalSection();
  if (osTaskIdMappingQueue != NULL)
  {
    pQEntry = osTaskIdMappingQueue;
    /* is this the one we are looking for? */
    if (pQEntry->handle == handle)
    {
      /* store the entry */
      foundQEntry = pQEntry;
    }
    /* search through the entire list */
    else
    {
      while (pQEntry->nextQEntry != NULL)
      {
        /* move on to the next entry in the linked list */
        pQEntry = pQEntry->nextQEntry;
        /* is this the one we are looking for? */
        if (pQEntry->handle == handle)
        {
          /* store the entry */
          foundQEntry = pQEntry;
          /* element found so all done */
          break;
        }
      }
    }
  }
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return foundQEntry;
} /*** end of OsTaskIdMappingQueueFindByHandle ***/


/************************************************************************************//**
** \brief     Obtains the task handle that belongs to the task with the specified name.
**            Note that the task handle is the same as its TCB.
** \param     name ASCII string with the name of the task.
** \return    Task handle.
**
****************************************************************************************/
static void *OsGetTaskHandleByName(char *name)
{
  tOsTaskIdMappingQEntry *pQEntry;
  tOsTaskIdMappingQEntry *foundQEntry = NULL;
  void *result = NULL;
  uint32_t saved_cs_state;

  /* can only search the queue if it is not empty */
  saved_cs_state = OsEnterCriticalSection();
  if (osTaskIdMappingQueue != NULL)
  {
    pQEntry = osTaskIdMappingQueue;
    /* is this the one we are looking for? */
    if (OsCompareTaskNames(pQEntry->name, name) == TRUE)
    {
      /* store the entry */
      foundQEntry = pQEntry;
    }
    /* search through the entire list */
    else
    {
      while (pQEntry->nextQEntry != NULL)
      {
        /* move on to the next entry in the linked list */
        pQEntry = pQEntry->nextQEntry;
        /* is this the one we are looking for? */
        if (OsCompareTaskNames(pQEntry->name, name) == TRUE)
        {
          /* store the entry */
          foundQEntry = pQEntry;
          /* element found so all done */
          break;
        }
      }
    }
  }
  /* return the results */
  if (foundQEntry != NULL)
  {
    result = foundQEntry->handle;
  }
  OsLeaveCriticalSection(saved_cs_state);
  return result;
} /*** end oif OsGetTaskHandleByName ***/


/************************************************************************************//**
** \brief     Adds an entry to the queue.
** \param     handle Handle of the task to add. Typically the same as its TCB.
** \param     name ASCII string with the name of the task.
** \param     prio Task priority.
** \return    none.
**
****************************************************************************************/
static void OsTaskIdMappingQueueAdd(void *handle, char *name, unsigned portBASE_TYPE prio)
{
  tOsTaskIdMappingQEntry *newQEntry;
  tOsTaskIdMappingQEntry *lastQEntry;
  uint32_t saved_cs_state;

  /* allocate memory for the queue entry on the heap */
  newQEntry = pvPortMalloc(sizeof(tOsTaskIdMappingQEntry));
  /* check if the heap wasn't full */
  configASSERT(newQEntry != NULL);
  /* configure the entry */
  newQEntry->handle = handle;
  newQEntry->name = name;
  newQEntry->prio = prio;
  /* entry is always added at the end so there is no next entry */
  newQEntry->nextQEntry = NULL;

  saved_cs_state = OsEnterCriticalSection();
  /* find the last entry into the queue */
  lastQEntry = OsTaskIdMappingQueueFindLast();
  /* add the new entry to the queue */
  if (lastQEntry == NULL)
  {
    /* queue empty so we are the first element */
    newQEntry->prevQEntry = NULL;
    osTaskIdMappingQueue = newQEntry;
  }
  else
  {
    /* add to the end of the list */
    lastQEntry->nextQEntry = newQEntry;
    newQEntry->prevQEntry = lastQEntry;
  }
  OsLeaveCriticalSection(saved_cs_state);
} /*** end of OsTaskIdMappingQueueAdd ***/


/************************************************************************************//**
** \brief     Removes an entry from the queue.
** \param     handle Handle of the task to remove. Typically the same as its TCB.
** \return    none.
**
****************************************************************************************/
static void OsTaskIdMappingQueueDelete(void *handle)
{
  tOsTaskIdMappingQEntry *delQEntry;
  uint32_t saved_cs_state;

  /* try to find the element to delete */
  delQEntry = OsTaskIdMappingQueueFindByHandle(handle);
  /* check if it's actually there */
  if (delQEntry != NULL)
  {
    saved_cs_state = OsEnterCriticalSection();
    /* is this the first one in the list? */
    if (delQEntry->prevQEntry == NULL)
    {
      /* remove it by making the list start at the next entry */
      osTaskIdMappingQueue = delQEntry->nextQEntry;
      if (delQEntry->nextQEntry != NULL)
      {
        /* can only make it the start of the list if it is an actual element */
        delQEntry->nextQEntry->prevQEntry = NULL;
      }
    }
    /* is this the last one in the list */
    else if (delQEntry->nextQEntry == NULL)
    {
      /* remove it by making the previous one the last one */
      delQEntry->prevQEntry->nextQEntry = NULL;
    }
    /* it's a list element somewhere in the middle */
    else
    {
      /* remove the link */
      delQEntry->prevQEntry->nextQEntry = delQEntry->nextQEntry;
      delQEntry->nextQEntry->prevQEntry = delQEntry->prevQEntry;
    }
    OsLeaveCriticalSection(saved_cs_state);
    /* now deallocate the memory */
    vPortFree(delQEntry);
  }
} /*** end of OsTaskIdMappingQueueDelete ***/


/************************************************************************************//**
** \brief     Performs a case sensitive comparison of 2 name strings.
** \param     name1 The first string to compare.
** \param     name2 The second string to compare.
** \return    BTRUE if the names are equal, FALSE otherwise.
**
****************************************************************************************/
static uint8_t OsCompareTaskNames(char *name1, char *name2)
{
  uint8_t idx;

  /* first make sure they have the same length */
  if (strlen(name1) != strlen(name2))
  {
    return FALSE;
  }
  /* start character by character comparison */
  for (idx=0; idx<strlen(name1); idx++)
  {
    /* compare them */
    if (name1[idx] != name2[idx])
    {
      return FALSE;
    }
  }
  /* still here so all characters match */
  return TRUE;
} /*** end of OsCompareTaskNames ***/


/************************************************************************************//**
** \brief     Callback that gets called each time a tick event occurred. Note that this
**            function will be called by a timer interrupt service routine so no critical
**            sections need to be used.
** \return    none
**
****************************************************************************************/
void OsRunTimeMeasurementTickEvent(void)
{
  if ( (osIdleRunTimeMeasurementCtrl.taskRunningFlag == TRUE) &&
       (osIdleRunTimeMeasurementCtrl.measurementPaused == FALSE) )
  {
    osIdleRunTimeMeasurementCtrl.taskRuntimeCount++;
  }
} /*** end of OsRunTimeMeasurementTickEvent ***/


/************************************************************************************//**
** \brief     Initializes the runtime measurement driver.
** \return    none
**
****************************************************************************************/
static void OsIdleRunTimeMeasurementInit(void)
{
  /* check that the run timer is configured for a higher frequency than the os tick */
  configASSERT(OS_RUNTIME_COUNTER_FREQUENCY > configTICK_RATE_HZ);
  osIdleRunTimeMeasurementCtrl.taskRunningFlag = FALSE;
  osIdleRunTimeMeasurementCtrl.taskRuntimeCount = 0;
  osIdleRunTimeMeasurementCtrl.measurementPaused = FALSE;
} /*** end of OsIdleRunTimeMeasurementInit ***/


/************************************************************************************//**
** \brief     Start the run time measurement.
** \return    none
**
****************************************************************************************/
void OsIdleRunTimeMeasurementStart(void)
{
  uint32_t saved_cs_state;

  saved_cs_state = OsEnterCriticalSection();
  osIdleRunTimeMeasurementCtrl.taskRunningFlag = TRUE;
  osIdleRunTimeMeasurementCtrl.measurementPaused = FALSE;
  OsLeaveCriticalSection(saved_cs_state);
} /*** end of OsIdleRunTimeMeasurementStart ***/


/************************************************************************************//**
** \brief     Stops the run time measurement.
** \return    none
**
****************************************************************************************/
void OsIdleRunTimeMeasurementStop(void)
{
  osIdleRunTimeMeasurementCtrl.taskRunningFlag = FALSE;
} /*** end of OsIdleRunTimeMeasurementStop ***/


/************************************************************************************//**
** \brief     Start the run time measurement.
** \return    none
**
****************************************************************************************/
static void OsIdleRunTimeMeasurementPauseFromISR(void)
{
} /*** end of OsIdleRunTimeMeasurementPauseFromISR ***/


/************************************************************************************//**
** \brief     Stops the run time measurement.
** \return    none
**
****************************************************************************************/
static void OsIdleRunTimeMeasurementResumeFromISR(void)
{
} /*** end of OsIdleRunTimeMeasurementResumeFromISR ***/


/************************************************************************************//**
** \brief     Resets the run time measurement timer.
** \return    none
**
****************************************************************************************/
static void OsIdleRunTimeMeasurementReset(void)
{
  osIdleRunTimeMeasurementCtrl.taskRuntimeCount = 0;
} /*** end of OsIdleRunTimeMeasurementReset ***/


/************************************************************************************//**
** \brief     Reads the run time measurement counter value.
** \return    counter value.
**
****************************************************************************************/
static uint32_t OsIdleRunTimeMeasurementGet(void)
{
  return osIdleRunTimeMeasurementCtrl.taskRuntimeCount;
} /*** end of OsIdleRunTimeMeasurementGet ***/


/*********************************** end of os.c ***************************************/
