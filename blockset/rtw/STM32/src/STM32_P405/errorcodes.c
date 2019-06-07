/************************************************************************************//**
* \file         errorcodes.c
* \brief        Error codes module source file.
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
#include "errorcodes.h"                               /* Error codes module            */
#include "os.h"                                       /* for operating system          */
#include "errorList.h"                                /* for error list                */
#include "eeprom_sim.h"                               /* simulated EEPROM  driver      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Constant signature value that is added to each error code when stored in
 *         EEPROM to flag it as valid.
 */
#define ERRCODES_VALIDITY_FLAG        (0x11EE55AAul)

/** \brief Return code in case no error was found. */
#define ERRCODES_CODE_NOT_FOUND       (-1)

/** \brief Mask to identify and entire group of error that have the same code but
 *         possibly a different param.
 */
#define ERRCODES_GROUP_MASK           (0xFFFF)

/** \brief For storing the error codes, 768 bytes of simulated EEPROM is reserved at the
 *         end of the 2kb simulated EEPROM module. This defines the start address.
 */
#define ERRCODES_EEPROM_START_ADDRESS (0x500)

/** \brief The number of bytes reserved for error codes in the simulated EEPROM. */
#define ERRCODES_EEPROM_SIZE          (768)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static int16_t ErrCodesFindActiveError(uint16_t errCode, uint8_t errParam, uint8_t useErrParam);
static uint8_t ErrCodesIsValidActiveError(uint8_t index);
static uint8_t ErrCodesDelSingleActiveError(uint16_t errCode, uint8_t errParam, uint8_t useErrParam);
static void    ErrCodesProgramStoredError(uint8_t index, uint16_t errCode, uint8_t errParam);
static uint8_t ErrCodesReadStoredError(uint8_t index, tErrCodeData *errData);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Array with the active errors. */
static tErrCodeData errCodeActiveErrors[ERRCODES_NUM_OF_MAX_ERRORS];

/** \brief Index into the active errors array where the next error can be stored. */
static uint8_t errCodeNextFreeActiveErrIdx;

/** \brief Index into the saved errors allocation where the next error can be stored. */
static uint8_t errCodeNextFreeStoredErrIdx;


/************************************************************************************//**
** \brief     Initializes the error codes module. It checks the validity of the flash
**            EEPROM sector that was reserved for error codes storages and erases the
**            entire sector in invalid.
** \return    none.
**
****************************************************************************************/
void ErrCodesInit(void)
{
  /* delete all active errors */
  ErrCodesDelActiveErrors();
  /* determine how many error codes are stored in */
  errCodeNextFreeStoredErrIdx = 0;
  while(ErrCodesReadStoredError(errCodeNextFreeStoredErrIdx, NULL) == TRUE)
  {
    /* move on to the next one */
    errCodeNextFreeStoredErrIdx++;
  }
} /*** end of ErrCodesInit ***/


/************************************************************************************//**
** \brief     Set an error. The error is stored in RAM (active error) and optionally in
**            EEPROM (saved error). An error is well-defined by its 16-bit error code
**            and 8-bit error parameter.
** \details   It is always stored in RAM. If the error is already stored in RAM, then its
**            occurrence counter will be increased and the timestamp will be updated.
**            The timestamp is the number of ticks the OS is running since
**            startup. Additionally, the error is saved in EEPROM if (1) parameter
**            errSaveEeprom is > 0 and (2) the error is not yet stored in RAM.
**            So if an error was not yet set in RAM (active error) and this function is
**            called with the errSaveEerpom param to > 0, it will be saved in both EEPROM
**            and RAM. Note that an error with the same code but different param is
**            treated as a seperate error code.
** \param     errCode Error code value, which is application defined.
** \param     errParam Error parameter, which is application defined. Could be severity
**            level for example.
** \param     errSaveEeprom. Set to 0 to make it just an active error or > 0 to also
**            save it to EEPROM as a stored error. The latter option only works if it
**            is not yet an active error.
** \return    none.
**
****************************************************************************************/
void ErrCodesSetError(uint16_t errCode, uint8_t errParam, uint8_t errSaveEeprom)
{
  uint8_t errIdx;
  int16_t errSearchResult;
  tErrCodeData errData;
  uint8_t errStoredAlreadyPresent = FALSE;

  /* is this error already present in RAM as an active error? */
  errSearchResult = ErrCodesFindActiveError(errCode, errParam, TRUE);
  if (errSearchResult != ERRCODES_CODE_NOT_FOUND)
  {
    /* this error is already present so simply update it */
    errIdx = (uint8_t)errSearchResult;
    /* validate the index before using it  */
    errCodeActiveErrors[errIdx].occurrence++;
    errCodeActiveErrors[errIdx].timestamp = xTaskGetTickCount();
    /* all done */
    return;
  }
  /* still here so the error was not yet present in RAM */
  if (errCodeNextFreeActiveErrIdx < ERRCODES_NUM_OF_MAX_ERRORS)
  {
    /* get the index of where to store this error */
    errIdx = errCodeNextFreeActiveErrIdx;
    /* increment indexer to the next free active error for the next time */
    errCodeNextFreeActiveErrIdx++;
    /* store it as an active erorr in RAM */
    errCodeActiveErrors[errIdx].code = errCode;
    errCodeActiveErrors[errIdx].param = errParam;
    errCodeActiveErrors[errIdx].occurrence = 1;
    errCodeActiveErrors[errIdx].timestamp = xTaskGetTickCount();
    /* should it be stored in EEPROM as well? */
    if (errSaveEeprom != 0)
    {
      /* make sure the same error isn't already stored */
      for (errIdx=0; errIdx<errCodeNextFreeStoredErrIdx; errIdx++)
      {
        /* read the error */
        if (ErrCodesReadStoredError(errIdx, &errData) == TRUE)
        {
          /* check similar code/param */
          if ( (errData.code == errCode) && (errData.param == errParam) )
          {
        	/* program the error at the same location as the already existing
        	 * error. this will simply update its occurence counter.
        	 */
            ErrCodesProgramStoredError(errIdx, errCode, errParam);
            errStoredAlreadyPresent = TRUE;
            break;
          }
        }
      }
      /* only store the error if not already stored */
      if (errStoredAlreadyPresent == FALSE)
      {
        if (errCodeNextFreeStoredErrIdx < ERRCODES_NUM_OF_MAX_ERRORS)
        {
          ErrCodesProgramStoredError(errCodeNextFreeStoredErrIdx, errCode, errParam);
          errCodeNextFreeStoredErrIdx++;
        }
      }
    }
  }
} /*** end of ErrCodesSetError ***/


/************************************************************************************//**
** \brief     Deletes all saved errors from EEPROM by erasing the flash pages that are
**            reserved for the stored error codes. Note that this operates on simulated
**            EEPROM so function EepromSimSynchronize should be used to actually process
**            the changes in non-volatile flash.
** \return    none.
**
****************************************************************************************/
void ErrCodesDelSavedErrors(void)
{
  /* erase the block of simulated EEPROM reserved by this module.  */
  EepromSimErase(ERRCODES_EEPROM_START_ADDRESS, ERRCODES_EEPROM_SIZE);
  /* reset the index to the next free error */
  errCodeNextFreeStoredErrIdx = 0;
} /*** end of ErrCodesDelSavedErrors ***/


/************************************************************************************//**
** \brief     Deletes all active errors from RAM.
** \return    none.
**
****************************************************************************************/
void ErrCodesDelActiveErrors(void)
{
  uint8_t idx;

  /* empty the active errors array */
  for (idx=0; idx < (sizeof(errCodeActiveErrors)/sizeof(errCodeActiveErrors[0])); idx++)
  {
    errCodeActiveErrors[idx].code = 0;
    errCodeActiveErrors[idx].param = 0;
    errCodeActiveErrors[idx].occurrence = 0;
    errCodeActiveErrors[idx].timestamp = 0;
  }
  /* init the index into the active errors array */
  errCodeNextFreeActiveErrIdx = 0;
} /*** end of ErrCodesDelActiveErrors ***/


/************************************************************************************//**
** \brief     Delete an active error / group of active errors in RAM. An error is well-
**            defined by error code and error parameter. If error parameter is 0xFFFF
**            a group of active errors specified by error code is deleted.
** \param     errCode Error code, specifies the origin and type of the error.
** \param     errParam Error parameter. It can be used freely by the application as
**            parameters:  0x0000..0x00FF : Identifies a single error.
**                         0xFFFF         : Identifies an error group, i.e. all errors
**                                          with the specified error code.
**                                          (ERRCODES_GROUP_MASK)
** \return    none.
**
****************************************************************************************/
void ErrCodesDelActiveError(uint16_t errCode, uint16_t errParam)
{
  if (errParam == ERRCODES_GROUP_MASK)
  {
    /* delete all active errors with this code */
    while (ErrCodesDelSingleActiveError(errCode, 0, FALSE) == TRUE)
    {
      ;
    }
  }
  else
  {
    /* delete the active error that matches both code and param */
    ErrCodesDelSingleActiveError(errCode, errParam, TRUE);
  }
} /*** end of ErrCodesDelActiveError ***/


/************************************************************************************//**
** \brief     Get the number of saved errors stored in EEPROM.
** \return    Number of errors stored in EEPROM.
**
****************************************************************************************/
uint8_t ErrCodesGetNumSavedErrors(void)
{
  uint8_t numErrors = ERRCODES_NUM_OF_MAX_ERRORS;

  if (errCodeNextFreeStoredErrIdx <= ERRCODES_NUM_OF_MAX_ERRORS)
  {
    numErrors = errCodeNextFreeStoredErrIdx;
  }

  return numErrors;
} /*** end of ErrCodesGetNumSavedErrors ***/


/************************************************************************************//**
** \brief     Get the number of active errors stored in RAM.
** \return    Number of errors stored in RAM.
**
****************************************************************************************/
uint8_t ErrCodesGetNumActiveErrors(void)
{
  uint8_t numErrors = ERRCODES_NUM_OF_MAX_ERRORS;

  if (errCodeNextFreeActiveErrIdx <= ERRCODES_NUM_OF_MAX_ERRORS)
  {
    numErrors = errCodeNextFreeActiveErrIdx;
  }

  return numErrors;
} /*** end of ErrCodesGetNumActiveErrors ***/


/************************************************************************************//**
** \brief     Copy saved errors stored in EEPROM to the user provided buffer.
** \param     startIndexErrors Start index to get errors from EEPROM. Index 0 points to
**            the first (oldest) error. At first it has to started with 0. But if there
**            are more errors than specified in parameter maxNumErrors then call this
**            function again with a certain value.
** \param     Maximum number of errors which can be copied. The needed buffer size has
**            to be large enough.
** \return    Number of errors that were copied to the buffer. The last stored error is
**            read if the return value is smaller than parameter maxNumErrors.
**
****************************************************************************************/
uint8_t ErrCodesGetSavedErrors(tErrCodeData *errData, uint8_t startIndexErrors, uint8_t maxNumErrors)
{
  uint8_t numErrors = 0;
  uint8_t errIdx;
  tErrCodeData storedErrData;

  /* validate params */
  if (!((startIndexErrors < ERRCODES_NUM_OF_MAX_ERRORS) && (errData != NULL)))
  {
    ErrCodesSetError(ER_CODE_ERRCODES_INVALID_PARAMS, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* copy loop */
  for (errIdx=startIndexErrors; errIdx<(startIndexErrors+maxNumErrors); errIdx++)
  {
    /* stop loop when all stored errors are copied or end of array is reached */
    if ( (ErrCodesReadStoredError(errIdx, &storedErrData) == FALSE) || (errIdx >= ERRCODES_NUM_OF_MAX_ERRORS) )
    {
      break;
    }
    /* still here so copy the error */
    errData->code = storedErrData.code;
    errData->param = storedErrData.param;
    errData->occurrence = storedErrData.occurrence;
    errData->timestamp = storedErrData.timestamp;
    /* point to the next buffer entry */
    errData++;
    /* increment number of read errors */
    numErrors++;
  }
  /* return the number of errors copied to the buffer */
  return numErrors;
} /*** end of ErrCodesGetSavedErrors ***/


/************************************************************************************//**
** \brief     Copy saved errors stored in RAM to the user provided buffer.
** \param     startIndexErrors Start index to get errors from RAM. Index 0 points to
**            the first (oldest) error. At first it has to started with 0. But if there
**            are more errors than specified in parameter maxNumErrors then call this
**            function again with a certain value.
** \param     Maximum number of errors which can be copied. The needed buffer size has
**            to be large enough.
** \return    Number of errors that were copied to the buffer. The last stored error is
**            read if the return value is smaller than parameter maxNumErrors.
**
****************************************************************************************/
uint8_t ErrCodesGetActiveErrors(tErrCodeData *errData, uint8_t startIndexErrors, uint8_t maxNumErrors)
{
  uint8_t numErrors = 0;
  uint8_t errIdx;

  /* validate params */
  if (!((startIndexErrors < ERRCODES_NUM_OF_MAX_ERRORS) && (errData != NULL)))
  {
    ErrCodesSetError(ER_CODE_ERRCODES_INVALID_PARAMS, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* copy loop */
  for (errIdx=startIndexErrors; errIdx<(startIndexErrors+maxNumErrors); errIdx++)
  {
    /* stop loop when all active errors are copied or end of array is reached */
    if ( (ErrCodesIsValidActiveError(errIdx) == FALSE) || (errIdx >= ERRCODES_NUM_OF_MAX_ERRORS) )
    {
      break;
    }
    /* still here so copy the error */
    errData->code = errCodeActiveErrors[errIdx].code;
    errData->param = errCodeActiveErrors[errIdx].param;
    errData->occurrence = errCodeActiveErrors[errIdx].occurrence;
    errData->timestamp = errCodeActiveErrors[errIdx].timestamp;
    /* point to the next buffer entry */
    errData++;
    /* increment number of read errors */
    numErrors++;
  }
  /* return the number of errors copied to the buffer */
  return numErrors;
} /*** end of ErrCodesGetActiveErrors ***/


/************************************************************************************//**
** \brief     Test if an error is active (that means stored in RAM).
** \param     errCode Error code value, which is application defined.
** \param     errParam Error parameter, which is application defined.
** \return    TRUE if this error is set in RAM, FALSE otherwise
**
****************************************************************************************/
uint8_t ErrCodesTestError(uint16_t errCode, uint16_t errParam)
{
  if (ErrCodesFindActiveError(errCode, errParam, TRUE) == ERRCODES_CODE_NOT_FOUND)
  {
    return FALSE;
  }
  else
  {
    return TRUE;
  }
} /*** end of ErrCodesTestError ***/


/************************************************************************************//**
** \brief     Finds the saved error with the same code and param as specified by the
**            parameters and stores its data if found.
** \param     errCode Error code value to find, which is application defined.
** \param     errParam Error parameter to find, which is application defined.
** \return    TRUE if this error was found, FALSE otherwise
**
****************************************************************************************/
uint8_t ErrCodesSearchSavedError(uint16_t errCode, uint8_t errParam, tErrCodeData *errData)
{
  uint8_t idx;
  tErrCodeData errReadData;

  /* search through the stored errors */
  for (idx=0; idx < errCodeNextFreeStoredErrIdx; idx++)
  {
    if (ErrCodesReadStoredError(idx, &errReadData) == TRUE)
    {
      if ( (errReadData.code == errCode) && (errReadData.param == errParam) )
      {
        /* store its data */
        errData->code = errReadData.code;
        errData->param = errReadData.param;
        errData->occurrence = errReadData.occurrence;
        errData->timestamp = errReadData.timestamp;
        /* no need to continue searching */
        return TRUE;
      }
    }
  }
  /* still here so error not found */
  return FALSE;
} /*** end of ErrCodesSearchSavedError ***/


/************************************************************************************//**
** \brief     Finds the active error with the same code and param as specified by the
**            parameters and stores its data if found.
** \param     errCode Error code value to find, which is application defined.
** \param     errParam Error parameter to find, which is application defined.
** \return    TRUE if this error was found, FALSE otherwise
**
****************************************************************************************/
uint8_t ErrCodesSearchActiveError(uint16_t errCode, uint8_t errParam, tErrCodeData *errData)
{
  uint8_t idx;

  /* search through the active errors array */
  for (idx=0; idx < errCodeNextFreeActiveErrIdx; idx++)
  {
    if ( (errCodeActiveErrors[idx].code == errCode) && (errCodeActiveErrors[idx].param == errParam) )
    {
      /* store its data */
      errData->code = errCodeActiveErrors[idx].code;
      errData->param = errCodeActiveErrors[idx].param;
      errData->occurrence = errCodeActiveErrors[idx].occurrence;
      errData->timestamp = errCodeActiveErrors[idx].timestamp;
      /* no need to continue searching */
      return TRUE;
    }
  }
  /* still here so error not found */
  return FALSE;
} /*** end of ErrCodesSearchActiveError ***/



/************************************************************************************//**
** \brief     Searches the active error RAM array to find the error with the specified
**            code and param and returns its index into the array.
** \param     errCode Error code value, which is application defined.
** \param     errParam Error parameter, which is application defined.
** \param     useErrParam if TRUE a match is searched for both the code and param. If
**            FALSE then the first error with a matching code is found and param is
**            treated as don't care.
** \return    Index in the array of where this error is located or
**            ERRCODES_CODE_NOT_FOUND in case the error was not found.
**
****************************************************************************************/
static int16_t ErrCodesFindActiveError(uint16_t errCode, uint8_t errParam, uint8_t useErrParam)
{
  uint8_t idx;
  int16_t result = ERRCODES_CODE_NOT_FOUND;

  /* search through the active errors array */
  for (idx=0; idx < (sizeof(errCodeActiveErrors)/sizeof(errCodeActiveErrors[0])); idx++)
  {
    if (errCodeActiveErrors[idx].code == errCode)
    {
      if (useErrParam == FALSE)
      {
        /* error code found so store its index */
        result = idx;
        /* no need to continue searching */
        break;
      }
      else
      {
        /* need to also math param */
        if (errCodeActiveErrors[idx].param == errParam)
        {
          /* error code found so store its index */
          result = idx;
          /* no need to continue searching */
          break;
        }
      }
    }
  }
  /* return the result */
  return result;
} /*** end of ErrCodesFindActiveError ***/


/************************************************************************************//**
** \brief     Determines if an active error at the specific index in the array is valid.
** \return    TRUE if valid, FALSE otherwise.
**
****************************************************************************************/
static uint8_t ErrCodesIsValidActiveError(uint8_t index)
{
  /* a valid active error is expected to have a non zero occurence */
  if (errCodeActiveErrors[index].occurrence != 0)
  {
    return TRUE;
  }
  return FALSE;
} /*** end of ErrCodesIsValidActiveError ***/


/************************************************************************************//**
** \brief     Delete an active error in RAM. An error is well-defined by error code and
**            error parameter.
** \param     errCode Error code, specifies the origin and type of the error.
** \param     errParam Error parameter. It can be used freely by the application as
**            parameters:  0x0000..0x00FF : Identifies a single error.
** \param     useErrParam if TRUE a match is searched for both the code and param. If
**            FALSE then the first error with a matching code is deleted and param is
**            treated as don't care.
** \return    TRUE is this error was found and deleted, FALSE if this error is not
**            present.
**
****************************************************************************************/
static uint8_t ErrCodesDelSingleActiveError(uint16_t errCode, uint8_t errParam, uint8_t useErrParam)
{
  uint8_t errIdxDel;
  uint8_t errIdxLast;
  uint8_t errIdx;
  int16_t errSearchResult;

  /* first see if this error actually exists */
  errSearchResult = ErrCodesFindActiveError(errCode, errParam, useErrParam);
  if (errSearchResult == ERRCODES_CODE_NOT_FOUND)
  {
    /* noting to do because this error is not present */
    return FALSE;
  }
  /* still here so this error is present */
  errIdxDel = (uint8_t)errSearchResult;

  /* is it the last entry in the array */
  if (errIdxDel == (errCodeNextFreeActiveErrIdx - 1))
  {
    /* remove the last entry from the array */
    errCodeActiveErrors[errIdxDel].code = 0;
    errCodeActiveErrors[errIdxDel].param = 0;
    errCodeActiveErrors[errIdxDel].occurrence = 0;
    errCodeActiveErrors[errIdxDel].timestamp = 0;
    errCodeNextFreeActiveErrIdx--;
    return TRUE;
  }
  /* still here so there are more errors after this one in the array then need to be
   * copied down.
   */
  errIdxLast = errCodeNextFreeActiveErrIdx - 1;
  /* perform copy down */
  for (errIdx=errIdxDel; errIdx<(errIdxDel + (errIdxLast-errIdxDel)); errIdx++)
  {
    errCodeActiveErrors[errIdx].code = errCodeActiveErrors[errIdx+1].code;
    errCodeActiveErrors[errIdx].param = errCodeActiveErrors[errIdx+1].param;
    errCodeActiveErrors[errIdx].occurrence = errCodeActiveErrors[errIdx+1].occurrence;
    errCodeActiveErrors[errIdx].timestamp = errCodeActiveErrors[errIdx+1].timestamp;
  }
  /* remove the last entry from the array */
  errCodeActiveErrors[errIdxLast].code = 0;
  errCodeActiveErrors[errIdxLast].param = 0;
  errCodeActiveErrors[errIdxLast].occurrence = 0;
  errCodeActiveErrors[errIdxLast].timestamp = 0;
  errCodeNextFreeActiveErrIdx--;
  return TRUE;
} /*** end of ErrCodesDelSingleActiveError ***/


/************************************************************************************//**
** \brief     Programs the error code to flash EEPROM at the correct index. Note that the
**            flash memory must be in an erased state. It is not possible to overwrite
**            an exsiting one due to the properties of flash memory.
** \param     index Zero based index of the error code. Can be 0 to
**            (ERRCODES_NUM_OF_MAX_ERRORS-1)
** \param     errCode Error code, specifies the origin and type of the error.
** \param     errParam Error parameter. It can be used freely by the application as
**            parameters:  0x0000..0x00FF : Identifies a single error.
** \return    None.
**
****************************************************************************************/
static void ErrCodesProgramStoredError(uint8_t index, uint16_t errCode, uint8_t errParam)
{
  uint32_t programBaseAddr;
  uint32_t programData[4];
  uint8_t  wordCnt;
  tErrCodeData readErrCode;
  uint8_t occurrenceCnt = 1;

  /* check if the error is already stored, in which case the occurrence counter needs
   * to be updated.
   */
  if (ErrCodesReadStoredError(index, &readErrCode) == TRUE)
  {
	  occurrenceCnt = readErrCode.occurrence + 1;
  }
  /* prepare the data to be programmed in chunks of 32-bit */
  /* array element 0: code, param and occurrence */
  programData[0] = (errCode << 16) | (errParam << 8) | occurrenceCnt;
  /* array element 1: the timestamp */
  programData[1] = xTaskGetTickCount();
  /* array element 2: signature validity flag */
  programData[2] = ERRCODES_VALIDITY_FLAG;
  /* checksum */
  programData[3] = programData[0] + programData[1] + programData[2];

  /* determine the base address */
  programBaseAddr = ERRCODES_EEPROM_START_ADDRESS + (index * sizeof(programData));

  /* program the data word-by-word */
  for (wordCnt=0; wordCnt< (sizeof(programData)/sizeof(programData[0])); wordCnt++)
  {
    EepromSimUINT32Set(programBaseAddr, programData[wordCnt]);
    programBaseAddr += sizeof(programData[0]);
  }
} /*** end of ErrCodesProgramStoredError ***/


/************************************************************************************//**
** \brief     Reads the error code from flash EEPROM at the specified index.
** \param     index Zero based index of the error code. Can be 0 to
**            (ERRCODES_NUM_OF_MAX_ERRORS-1)
** \param     errData Storage buffer for the error data. If NULL then no copying of data
**            is performed.
** \return    TRUE is a valid error was stored at the index, FALSE otherwise.
**
****************************************************************************************/
static uint8_t ErrCodesReadStoredError(uint8_t index, tErrCodeData *errData)
{
  uint32_t errorBaseAddr;
  uint32_t errorRawData[4];
  uint8_t  wordCnt;
  uint32_t checksum;

  /* determine the base address */
  errorBaseAddr = ERRCODES_EEPROM_START_ADDRESS + (index * sizeof(errorRawData));

  /* read the data word-by-word */
  for (wordCnt=0; wordCnt< (sizeof(errorRawData)/sizeof(errorRawData[0])); wordCnt++)
  {
    errorRawData[wordCnt] = EepromSimUINT32Get(errorBaseAddr);
    errorBaseAddr += sizeof(errorRawData[0]);
  }

  /* verify that the validity flag is present */
  if (errorRawData[2] != ERRCODES_VALIDITY_FLAG)
  {
    return FALSE;
  }

  /* calculate the checksum */
  checksum = errorRawData[0] + errorRawData[1] + errorRawData[2];
  /* verify the checksum */
  if (checksum != errorRawData[3])
  {
    return FALSE;
  }

  /* still here so a valid error code was read. now parse it */
  if (errData != NULL)
  {
    errData->code = (uint16_t)(errorRawData[0] >> 16);
    errData->param = (uint8_t)(errorRawData[0] >> 8);
    errData->occurrence = (uint8_t)errorRawData[0];
    errData->timestamp = errorRawData[1];
  }
  return TRUE;
} /*** end of ErrCodesReadStoredError ***/


/************************************ end of errorcodes.c ******************************/


