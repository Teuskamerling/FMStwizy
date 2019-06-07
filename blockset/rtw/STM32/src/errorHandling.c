/************************************************************************************//**
* \file         errorHandling.c
* \brief        Error handling source file.
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
#include "errorHandling.h"
#include "errorcodes.h"                               /* Error codes module            */
#include "os.h"                                       /* for operating system          */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* total number of errors, excluding the first row */
#define ERROR_LIST_NR_ERRORS                  (20)


/****************************************************************************************
* Local data declaratons
****************************************************************************************/
/* "ErrorData_ts" is a  struct defined by the RC30 API:
 *   Altough the struct is build out of the variables noted as 'Next rows',
 *   the first row uses these variables as following;
 *
 *  Byte        0,1,      2            3              4,5,6,7
 *  First row*  reserved  controlFlag  semaphore      size 
 *  Next rows*  Code_u16  Param_u8     Occurrence_u8  Timestamp_u32
 */
static tErrCodeData errorListBuffer[ERROR_LIST_NR_ERRORS + 1];


/****************************************************************************************
** NAME:           getErrorListAddress
** PARAMETER:      none
** RETURN VALUE:   pointer to error list buffer
** DESCRIPTION:    Obtains a pointer to the error list buffer
**
****************************************************************************************/
uint8_t* getErrorListAddress(void)
{ 
  return (uint8_t*)&errorListBuffer[0];
} /*** end of getErrorListAddress ***/


/****************************************************************************************
** NAME:           bufferSync
** PARAMETER:      action info about how to update the buffer:
**                 - 0: reads the number of active errors
**                 - 1: reads the number of saved errors
**                 - 2: deletes the active errors
**                 - 3: deletes the saved errors
** RETURN VALUE:   none
** DESCRIPTION:    Updates the first entry in the error list buffer based on the given
**                 function parameter and performs the requested action.
**
****************************************************************************************/
void bufferSync(uint8_t action)
{ 
  uint8_t overflow = 0;
    
	/* set semaphore 0 & controlFlag 1 */
  errorListBuffer[0].occurrence = 0;
  errorListBuffer[0].param = 1;
  
  /* copy all errors to buffer, skip first row in errorList, record number of copied 
   * errors.
   */
  switch (action)
  {
    case 0:
      /* check for buffer overflow, if so, store max -1 (leave 1 row for buffer overflow
       * error) errors 
       */
      if (ErrCodesGetNumActiveErrors() < ERROR_LIST_NR_ERRORS)
      { 
        errorListBuffer[0].timestamp = ErrCodesGetActiveErrors(&errorListBuffer[1], 0, \
                                                                ERROR_LIST_NR_ERRORS); 
      }
      else
      {
        ErrCodesGetActiveErrors(&errorListBuffer[1], 0, ERROR_LIST_NR_ERRORS - 1);
        overflow = 1;
      }
      break;
    case 1:
      if (ErrCodesGetNumSavedErrors() < ERROR_LIST_NR_ERRORS)
      { 
        errorListBuffer[0].timestamp = ErrCodesGetSavedErrors(&errorListBuffer[1], 0, \
                                                              ERROR_LIST_NR_ERRORS); 
      }
      else
      {
        ErrCodesGetSavedErrors(&errorListBuffer[1], 0, ERROR_LIST_NR_ERRORS - 1);
        overflow = 1;
      }
      break;
    case 2:
      ErrCodesDelActiveErrors();
      errorListBuffer[0].timestamp = 0;
      break;
    case 3:
      ErrCodesDelSavedErrors();
      errorListBuffer[0].timestamp = 0;
      break;
  }
    
  if (overflow == 1)
  {
    /* set buffer overflow error */
    errorListBuffer[ERROR_LIST_NR_ERRORS].code = 0xFFFE;
    errorListBuffer[ERROR_LIST_NR_ERRORS].timestamp = xTaskGetTickCount();
    /* set max size */
    errorListBuffer[0].timestamp = ERROR_LIST_NR_ERRORS;
  }
} /*** end of bufferSync ***/


/****************************************************************************************
** NAME:           releaseBuffer
** PARAMETER:      none
** RETURN VALUE:   1 if the buffer was released, 0 otherewise
** DESCRIPTION:    Releases the error list buffer.
**
****************************************************************************************/
uint8_t releaseBuffer(void)
{
    /* set semaphore 1 */
    errorListBuffer[0].occurrence = 1;

    if (errorListBuffer[0].param == 1)
    {
      /* set controlFlag 0 */
      errorListBuffer[0].param = 0;
      return 1;
    }
    return 0;
} /*** end of releaseBuffer ***/


/*********************************** end of errorHandling.c ****************************/
