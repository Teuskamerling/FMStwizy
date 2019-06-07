/************************************************************************************//**
* \file         filelogger.c
* \brief        SD-card file logger source file.
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
#include <ctype.h>                                    /* STD character class library   */
#include <string.h>                                   /* ANSI C string lib             */
#include <stdlib.h>                                   /* STD utility functions library */
#include "filelogger.h"                               /* filelogger module             */
#include "os.h"                                       /* for operating system          */
#include "ff.h"                                       /* FATFS file system library     */
#include "ftoa.h"                                     /* float to ASCII utility        */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define FILELOGGER_TASK_PRIO              (tskIDLE_PRIORITY + 4)
#define FILELOGGER_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE + 512)
#define FILELOGGER_MAX_LINE_LENGTH        (256)
#define FILELOGGER_ELEMENT_SEPARATOR      (';')
#define FILELOGGER_MAX_FILENAME_LEN       (24u)
#define FILELOGGER_MAX_FILENAME_EXT_LEN   (6u)
/** \brief Maximum number of decimal digits for the float to string conversion. */
#define FILELOGGER_MAX_FTOA_PRECISION     (3u)
/** \brief Maximum number of decimal digits for the double to string conversion. */
#define FILELOGGER_MAX_DTOA_PRECISION     (6u)
/** \brief Code indicating that the sequence number in the log file is invalid. */
#define FILELOGGER_INVALID_FILE_SEQUENCE  (-1)



/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Element table entry type type. */
typedef struct
{
	void *data;                                         /**< pointer to data value       */
	tFileLoggerElementType type;                        /**< data type                   */
} tFileLoggerTableEntry;


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Variable that holds the period time of FileLoggerTask in OS ticks. This allows
*         a dynamic override.
*/
static portTickType fileLoggerTaskPeriodTicks = (((portTickType)500*1000)/portTICK_PERIOD_US);

/** \brief Handle to the file logger task. */
static xTaskHandle fileLoggerTaskHandle = NULL;

/** \brief Pointer to the table with element entries. */
static tFileLoggerTableEntry *fileLoggerTable = NULL;

/** \brief Variable that holds the maximum allowed of data value elements that can be
*         logged.
*/
static uint8_t fileLoggerMaxElements = 0;

/** \brief String buffer for constructing a line for the file logger. */
static uint8_t fileLoggerLine[FILELOGGER_MAX_LINE_LENGTH];

/** \brief Character to be used as decimal separator. */
static uint8_t fileLoggerDotDecimalSeparator = '.';

/** \brief Boolean flag to determine if logging should actually be performed or not. */
static uint8_t fileLoggerIsLogging;

/** \brief File system object for mounting. */
static FATFS fileLoggerFS;

/** \brief FatFS file handle to the log-file. */
static FIL   fileLoggerFileHandle;

/** \brief Buffer for storing the name of the logfile with full path information. */
static char fileLoggerFileStr[FILELOGGER_MAX_FILENAME_LEN];

/** \brief Logger task parameter. */
static uint8_t fileLoggerStopRequest;

/** \brief Holds flag to determine if automatic start of logging should be used. */
static uint8_t fileLoggerAutoStart;

/** \brief A pointer to the string that contains the name of the file. */
//static const char *pointerToFileNameString;

/** \brief A pointer to the string which holds the first line of a file. */
//static const char *pointerToFirstLineString;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void     FileLoggerClearTable(void);
static void     FileLoggerTask(void *pvParameters);
static uint16_t FileLoggerIntToStr(int32_t value, uint8_t dest[]);
static uint16_t FileLoggerUIntToStr(uint32_t value, uint8_t dest[]);
static uint8_t  FileLoggerCreateFile(const char *filenameStr, const char *tableHeaderStr);
static uint8_t  FileLoggerSetFilename(const char *filenameStr);
static uint8_t  FileLoggerOpenFile(void);
static uint8_t  FileLoggerAddLineToFile(const char *line);
static int16_t  FileLoggerFilenameExtractSequence(const char *fileToCheck,
const char *baseFilename);
static char    *FileLoggerSeqNumToStr(uint16_t value, char *dest);


/************************************************************************************//**
** \brief     Initializes the file logger module.
** \param     filenameStr Desired name of the log-file on the SD-card, including file
**                        extension.
** \param     tableHeaderStr First line in the logger file that contains the column
**                           headers. A NULL-pointer skips this.
** \param     maxElements Maximum number of elements of which the data values should
**                        be periodically logged.
** \param     loggingIntervalTimeMs Interval time in milliseconds for the sampling of
**                                  the element data values and writing these as a line
**                                  to the log-file on the SD-card.
** \param     dotDecimalSeparator   If TRUE uses a '.' character as decimal seperator,
**                                  otherwise a ',' is used.
** \param     autoStart             If TRUE the logging is started automatically the
**                                  moment at least one element was added to the
**                                  logger module with function FileLoggerInitElement.
**                                  If FALSE, then the start of logging is triggered by
**                                  a call to FileLoggerStart.
** \return    none.
**
****************************************************************************************/
void FileLoggerInit(uint8_t maxElements, uint16_t loggingIntervalTimeMs, 
					uint8_t dotDecimalSeparator, uint8_t autoStart)
{
	portBASE_TYPE result;
	

	/* default to not active logging */
	fileLoggerIsLogging = FALSE;
	
	/* store autostart flag */
	fileLoggerAutoStart = autoStart;

	/* store decimal separator character */
	if (dotDecimalSeparator == FALSE)
	{
		fileLoggerDotDecimalSeparator = ',';
	}
	else
	{
		fileLoggerDotDecimalSeparator = '.';
	}

	/* Allocate table for element on the heap */
	fileLoggerTable = (tFileLoggerTableEntry *) pvPortMalloc(sizeof(tFileLoggerTableEntry) * maxElements);
	/* Verify that there was space on the heap for the table allocation */
	if (fileLoggerTable == NULL)
	{
		ErrCodesSetError(ER_CODE_LOGGER_TABLE_ALLOCATION_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
	}
	/* Store the number of elements the table can hold */
	fileLoggerMaxElements = maxElements;
	/* Invalidate all the entries in the table */
	FileLoggerClearTable();
	/* Set the stop request to FALSE */
	fileLoggerStopRequest = FALSE;
	/* Create the periodic file logger task */
	fileLoggerTaskPeriodTicks = (((portTickType)loggingIntervalTimeMs*1000)/portTICK_PERIOD_US);
	result = xTaskCreate(FileLoggerTask, "FileLoggerTask", FILELOGGER_TASK_STACK_SIZE,
	NULL, FILELOGGER_TASK_PRIO, &fileLoggerTaskHandle);
	if (!(result == pdPASS))
	{
		ErrCodesSetError(ER_CODE_LOGGER_TASK_INVALID, ER_PARAM_SEVERITY_CRITICAL, TRUE);
	}
	else
	{
		/* Suspend the task until at least one element is configured */
		vTaskSuspend(fileLoggerTaskHandle);
	}
	/* Start by mounting the file system, using logical disk 0 */
	if (f_mount(0, &fileLoggerFS) != FR_OK)
	{
		ErrCodesSetError(ER_CODE_LOGGER_FILE_CREATION_ERROR, ER_PARAM_SEVERITY_MINOR, TRUE);
	}
} /*** end of FileLoggerInit ***/


/************************************************************************************//**
** \brief     Initializes and element for the file logger.
** \param     elementIdx Zero based index to the element in the table.
** \param     elementPtr Pointer to the element's data value.
** \param     elementType Elements data type.
** \return    none.
**
****************************************************************************************/
void FileLoggerInitElement(uint8_t elementIdx, void *elementPtr, 
tFileLoggerElementType elementType)
{
	/* make sure the table is valid before accessing it */
	if (fileLoggerTable != NULL)
	{
		/* verify that the element index is not out of bounds */
		if (elementIdx >= fileLoggerMaxElements)
		{
			ErrCodesSetError(ER_CODE_LOGGER_TABLE_INVALID_ELEMENT, ER_PARAM_SEVERITY_CRITICAL, TRUE);
		}
		else
		{
			/* add the element to the table */
			fileLoggerTable[elementIdx].data = elementPtr;
			fileLoggerTable[elementIdx].type = elementType;
			/* resume the task function now that at least one element is in the table and logging
			* is active */
			if ( fileLoggerAutoStart == TRUE )
			{
				FileLoggerStart();
			}
		}
	}
} /*** end of FileLoggerInitElement ***/


/************************************************************************************//**
** \brief     Utility function for clearing all the entries in the elements table.
** \return    none.
**
****************************************************************************************/
static void FileLoggerClearTable(void)
{
	uint8_t elementIdx;

	/* make sure the table is valid before accessing it */
	if (fileLoggerTable != NULL)
	{
		/* set all entries to NULL pointer and a default data type of uint8_t */
		for (elementIdx=0; elementIdx<fileLoggerMaxElements; elementIdx++)
		{
			fileLoggerTable[elementIdx].data = NULL;
			fileLoggerTable[elementIdx].type = FILELOGGER_8BIT_UNSIGNED;
		}
	}
} /*** end of FileLoggerClearTable ***/


/************************************************************************************//**
** \brief     Periodic file logger task.
** \param     pvParamters Pointer to task parameters data structure.
** \return    none.
**
****************************************************************************************/
static void FileLoggerTask(void *pvParameters)
{
	portTickType lastWakeTime;
	uint8_t *linePtr;
	uint32_t currentTime;
	uint32_t startTime = 0;
	uint8_t elementIdx;
	int32_t elementValSigned;
	uint32_t elementValUnsigned;
	float elementValFloat;
	double elementValDouble;
	char floatStr[12 + FILELOGGER_MAX_DTOA_PRECISION + 1]; /* +1 for '\0' */

	/* initialize to current time */
	lastWakeTime = xTaskGetTickCount();

	/* Enter task body */
	for( ;; )
	{
		/* Create new file when logging is started */
		if (fileLoggerIsLogging == FALSE)
		{
			fileLoggerIsLogging = TRUE;
			/* Attempt to create and prepare the log-file on the SD-card, this typically only fails if
			* no SD-card is present, in this case there is no need to log at all. */
			if (FileLoggerCreateFile(LoggerFileName, LoggerFirstLine) == FALSE)
			{
				ErrCodesSetError(ER_CODE_LOGGER_FILE_CREATION_ERROR, ER_PARAM_SEVERITY_MINOR, TRUE);
				fileLoggerIsLogging = FALSE;
				vTaskSuspend(fileLoggerTaskHandle); /* Suspend task on an error */
			}
			/* Store the start time of logging so that the logged time resets at the start of logging.
			* this is not done when logging is restarted because the max file size was reached */
			if(startTime == 0)	startTime = OsGetSystemTime();
		}
		/* Reset the pointer to the start of the line */
		linePtr = &fileLoggerLine[0];
		/* First add the time. start with the seconds */
		currentTime = OsGetSystemTime() - startTime;
		linePtr += FileLoggerUIntToStr(currentTime/1000, linePtr);
		/* Add decimal separator character */
		*linePtr = fileLoggerDotDecimalSeparator;
		linePtr++;
		/* Now add the milliseconds */
		if(currentTime%1000 < 100){
			*linePtr = '0';
		}else{
		FileLoggerUIntToStr(currentTime%1000, linePtr);
		}
		linePtr++;
		if(currentTime%100 < 10){
			*linePtr = '0';
		}else{
		FileLoggerUIntToStr(currentTime%100, linePtr);
		}
		linePtr++;
		FileLoggerUIntToStr(currentTime%10, linePtr);
		linePtr++;
		/* Add the column separator */
		*linePtr = FILELOGGER_ELEMENT_SEPARATOR;
		linePtr++;
		/* Now loop through all elements and add them to the line */
		for (elementIdx=0; elementIdx<fileLoggerMaxElements; elementIdx++)
		{
			/* Make sure there is enough space on the line for the next element value. the
			* worst case length is 12 characters (for the smallest int32), but this is
			* overruled by the largest supported float by the ftoa() function, which is
			* determined by its integer part of the largest float (i.e. (int)FLT_MAX).
			* this value is 2147483647 and contains 11 digits. However, the ftoa() function
			* will still add a decimal character and FILELOGGER_MAX_DTOA_PRECISION digits,
			* so the number of characters would be 2147483647.0000, assuming
			* FILELOGGER_MAX_DTOA_PRECISION = 4. this means there should be space for
			* 12 + FILELOGGER_MAX_DTOA_PRECISION on the line.*/
			if ((linePtr + (12 + FILELOGGER_MAX_DTOA_PRECISION)) > &fileLoggerLine[FILELOGGER_MAX_LINE_LENGTH-1])
			{
				/* TODO Jason implement error handling here */
				break;
			}

			/* add a 0 value for invalid/not yet configured elements */
			if (fileLoggerTable[elementIdx].data == NULL)
			{
				*linePtr = '0';
				linePtr++;
			}
			else
			{
				/* convert the element value to a string, which differs based on its type */
				switch (fileLoggerTable[elementIdx].type)
				{
				case FILELOGGER_8BIT_SIGNED:
					elementValSigned = *((int8_t *)fileLoggerTable[elementIdx].data);
					linePtr += FileLoggerIntToStr(elementValSigned, linePtr);
					break;

				case FILELOGGER_8BIT_UNSIGNED:
					elementValUnsigned = *((uint8_t *)fileLoggerTable[elementIdx].data);
					linePtr += FileLoggerUIntToStr(elementValUnsigned, linePtr);
					break;

				case FILELOGGER_16BIT_SIGNED:
					elementValSigned = *((int16_t *)fileLoggerTable[elementIdx].data);
					linePtr += FileLoggerIntToStr(elementValSigned, linePtr);
					break;

				case FILELOGGER_16BIT_UNSIGNED:
					elementValUnsigned = *((uint16_t *)fileLoggerTable[elementIdx].data);
					linePtr += FileLoggerUIntToStr(elementValUnsigned, linePtr);
					break;

				case FILELOGGER_32BIT_SIGNED:
					elementValSigned = *((int32_t *)fileLoggerTable[elementIdx].data);
					linePtr += FileLoggerIntToStr(elementValSigned, linePtr);
					break;

				case FILELOGGER_32BIT_UNSIGNED:
					elementValUnsigned = *((uint32_t *)fileLoggerTable[elementIdx].data);
					linePtr += FileLoggerUIntToStr(elementValUnsigned, linePtr);
					break;

				case FILELOGGER_FLOAT_SINGLE:
					elementValFloat = *((float *)fileLoggerTable[elementIdx].data);
					/* convert to string */
					ftoa(floatStr, elementValFloat, FILELOGGER_MAX_FTOA_PRECISION, fileLoggerDotDecimalSeparator);
					/* add it to the line */
					strcpy((char *)linePtr, floatStr);
					/* update the line pointer */
					linePtr += strlen(floatStr);
					break;

				case FILELOGGER_FLOAT_DOUBLE:
					elementValDouble = *((double *)fileLoggerTable[elementIdx].data);
					/* convert to string */
					ftoa(floatStr, elementValDouble, FILELOGGER_MAX_DTOA_PRECISION, fileLoggerDotDecimalSeparator);
					/* add it to the line */
					strcpy((char *)linePtr, floatStr);
					/* update the line pointer */
					linePtr += strlen(floatStr);
					break;

				default:
					break;
				}
			}
			/* add the column separator */
			*linePtr = FILELOGGER_ELEMENT_SEPARATOR;
			linePtr++;
		} /* End of For loop with elements */
		/* complete the line by adding the string termination */
		*linePtr = '\0';
		linePtr++;

		/* Write the line to the file */
		FileLoggerAddLineToFile((const char *)fileLoggerLine);
		
		/* Check to see if maximum size has been reached */
		if (f_size(&fileLoggerFileHandle) > fileLoggerMaxFileLength)
		{
			/* close the file */
			f_close(&fileLoggerFileHandle);
			/* Set the file logging flag to false, this will make sure a new file will be created */
			fileLoggerIsLogging = FALSE;
		}
		/* Check for stop request */
		if (fileLoggerStopRequest == TRUE)
		{
			/* close the file */
			f_close(&fileLoggerFileHandle);
			/* Set the file logging flag to false */
			fileLoggerIsLogging = FALSE;
			/* suspend ourselves indefinitely */
			vTaskSuspend(fileLoggerTaskHandle);
			/* Reinitialize to current time after restart of task */
			lastWakeTime = xTaskGetTickCount();
			/* Reset startTime */
			startTime = 0;
		}
		else
		{
			/* activate this task periodically */
			vTaskDelayUntil(&lastWakeTime, fileLoggerTaskPeriodTicks);
		}
	} /* End of task body */
} /*** end of FileLoggerTask ***/


/************************************************************************************//**
** \brief     Triggers the start of logging. Note that this function has no effect if
**            the module was initialized with the autoStart parameter set to true.
** \return    none.
**
****************************************************************************************/
void FileLoggerStart(void)
{
	/* Start logging if not auto started by resuming the task function. */
	if ( (fileLoggerTaskHandle != NULL) && (fileLoggerIsLogging == FALSE) )
	{
		/* Only start if the task is suspended to make sure no problems occur if this
		* function is called multiple times.*/
		if (eTaskGetState(fileLoggerTaskHandle) == eSuspended)
		{
			/* Reset the request to stop the logger task */
			fileLoggerStopRequest = FALSE;
			vTaskResume(fileLoggerTaskHandle);
		}
	}
} /*** end of FileLoggerStart ***/


/************************************************************************************//**
** \brief     Stops logging by suspending the logger task and closing the log-file. Note
**            that this function must be called once upon software program shutdown or
**            other external trigger to stop logging, otherwise no valid log-file will
**            be present on the SD-card.
** \return    none.
**
****************************************************************************************/
void FileLoggerStop(void)
{
	/* set the request to stop the logger task and close the log-file */
	fileLoggerStopRequest = TRUE;
} /*** end of FileLoggerStop ***/


/************************************************************************************//**
** \brief     Converts a 32-bit signed integer to a string with ASCII characters.
** \param     value the value to convert
** \param     dest pointer to character array for the destination string.
** \return    the length of the string excluding the string termination.
**
****************************************************************************************/
static uint16_t FileLoggerIntToStr(int32_t value, uint8_t dest[])
{
	int8_t sign = 1;
	uint8_t stringBuf[12];
	uint8_t *charPtr;
	uint8_t charIdx;

	/* remove sign from value but keep track if it was a negative number or not */
	if (value < 0)
	{
		sign = -1;
		/* note that this doesn't work for the value -2147483648, because 2147483648 does not
	* fit into 32-bit signed int. in this exceptional case, do the conversion manually.
	*/
		if (value < -2147483647)
		{
			dest  = (uint8_t*) "-2147483648";
			return 11;
		}
		value = -value;
	}

	/* range is -2147483648 to 2147483647. 12 characters are needed worst case to store
* the smallest number including the '-' sign. move the pointer to the end and
* place the string termination.
*/
	charPtr = &stringBuf[0];
	charPtr += 11;
	*charPtr = '\0';

	/* convert each digit one-by-one going from least significant digit to most
* significant.
*/
	do
	{
		charPtr--;
		*charPtr = value % 10 + '0';
		value /= 10;
	}
	while (value > 0);

	/* if it was a negative number, add the '-' sign */
	if (sign < 0)
	{
		charPtr--;
		*charPtr = '-';
	}

	/* charPtr now points to the start of the resulting string. note that this is in
* most cases not the same as stringBuf[0]. for convenience copy the resulting
* string to the dest[]-array so that the string is actually at the start of
* this array.
*/
	charIdx = 0;
	do
	{
		dest[charIdx] = *charPtr;
		charIdx++;
		charPtr++;
	}
	while (*charPtr != '\0');
	/* to complete the string copy, add the string termination character */
	dest[charIdx] = '\0';
	return charIdx;
} /*** end of FileLoggerIntToStr ***/


/************************************************************************************//**
** \brief     Converts a 32-bit unsigned integer to a string with ASCII characters.
** \param     value the value to convert
** \param     dest pointer to character array for the destination string.
** \return    the length of the string excluding the string termination.
**
****************************************************************************************/
static uint16_t FileLoggerUIntToStr(uint32_t value, uint8_t dest[])
{
	uint8_t stringBuf[12];
	uint8_t *charPtr;
	uint8_t charIdx;

	/* range is 0 to 4294967295. 11 characters are needed worst case to store
* the largest number including the string termination.
*/
	charPtr = &stringBuf[0];
	charPtr += 10;
	*charPtr = '\0';

	/* convert each digit one-by-one going from least significant digit to most
* significant.
*/
	do
	{
		charPtr--;
		*charPtr = value % 10 + '0';
		value /= 10;
	}
	while (value > 0);

	/* charPtr now points to the start of the resulting string. note that this is in
* most cases not the same as stringBuf[0]. for convenience copy the resulting
* string to the dest[]-array so that the string is actually at the start of
* this array.
*/
	charIdx = 0;
	do
	{
		dest[charIdx] = *charPtr;
		charIdx++;
		charPtr++;
	}
	while (*charPtr != '\0');
	/* to complete the string copy, add the string termination character */
	dest[charIdx] = '\0';
	return charIdx;
} /*** end of FileLoggerUIntToStr ***/


/************************************************************************************//**
** \brief     Creates the file on the SD-card, stores the filename for successive file
**            operations and adds the header string to the file name.
**
** \param     filenameStr Desired name of the log-file on the SD-card, including file
**                        extension.
** \param     tableHeaderStr First line in the logger file that contains the column
**                           headers. A NULL-pointer skips this.
** \return    TRUE is successful, FALSE otherwise.
**
****************************************************************************************/
static uint8_t FileLoggerCreateFile(const char *filenameStr, const char *tableHeaderStr)
{
	/* determine file name of the log-file */
	if (FileLoggerSetFilename(filenameStr) == FALSE)
	{
		return FALSE;
	}
	/* create the log-file on the SD-card and open it for writing */
	if (FileLoggerOpenFile() == FALSE)
	{
		return FALSE;
	}
	/* add the table header string */
	FileLoggerAddLineToFile(tableHeaderStr);
	return TRUE;
} /*** end of FileLoggerCreateFile ***/


/************************************************************************************//**
** \brief     Determines and sets the filename that should be used in future file
**            operations.
** \param     filenameStr Desired name of the log-file on the SD-card, including file
**                        extension.
** \return    TRUE is successful, FALSE otherwise.
**
****************************************************************************************/
static uint8_t FileLoggerSetFilename(const char *filenameStr)
{
	FRESULT res;
	FILINFO fno;
	DIR dir;
#if _USE_LFN
	char lfn[_MAX_LFN + 1];
#endif
	char *fn;
	uint16_t nextSeqNum = 0;
	uint16_t currentSeqNum;
	char seqStr[4];
#if _USE_LFN
	fno.lfname = lfn;
	fno.lfsize = sizeof(lfn);
#endif
	char baseFileWithoutExt[FILELOGGER_MAX_FILENAME_LEN];
	char baseFileExt[FILELOGGER_MAX_FILENAME_EXT_LEN];
	char *charPtr;

	/* clear the filename string buffer */
	memset(fileLoggerFileStr, '\0', FILELOGGER_MAX_FILENAME_LEN);

	/* Need to add a sequence number to the filename. for this we need
	* to search through all files in the root directory on the SD-card to find the
	* log file with the highest sequence number to find the sequence number to use
	* for the new log file.
	*/
	res = f_opendir(&dir, "/");
	/* clear the log file extension buffer  */
	memset(baseFileExt, '\0', FILELOGGER_MAX_FILENAME_EXT_LEN);
	/* copy the filename with extension */
	strcpy(baseFileWithoutExt, filenameStr);
	/* determine the location of the extension */
	if ((charPtr = strchr(baseFileWithoutExt, '.'))  != NULL)
	{
		/* copy extension to extension buffer, including the '.' */
		strcpy(baseFileExt, charPtr);
		/* replace the '.' with a string termination character to remove the extension */
		*charPtr = '\0';
	}

	if (res == FR_OK)
	{
		/* go through all the files in the directory */
		for (;;)
		{
			/* read an item from the directory */
			res = f_readdir(&dir, &fno);
			/* break on error or end of directory */
			if (res != FR_OK || fno.fname[0] == 0)
			{
				break;
			}
			/* ignore dot entry */
			if (fno.fname[0] == '.')
			{
				continue;
			}
			#if _USE_LFN
			fn = *fno.lfname ? fno.lfname : fno.fname;
			#else
			fn = fno.fname;
			#endif
			/* ignore directories, we are just interested in files */
			if ((fno.fattrib & AM_DIR) != AM_DIR)
			{
				/* check if this is a log file and if so try to extract its sequence
		* number.
		*/
				currentSeqNum = FileLoggerFilenameExtractSequence(fn, baseFileWithoutExt);
				/* is its sequence number larger than we have found so far? */
				if ( (currentSeqNum != FILELOGGER_INVALID_FILE_SEQUENCE) &&
						(currentSeqNum > nextSeqNum) )
				{
					/* store this number as the highest found sequency number */
					nextSeqNum = currentSeqNum;
				}
			}
		}
	}
	/* determine sequence number for the new log file. note that if no existing
* log files were found, the sequence number is still 0.
*/
	nextSeqNum++;
	/* create the log filename with the correct sequence number */
	strcat(fileLoggerFileStr, baseFileWithoutExt);
	strcat(fileLoggerFileStr, FileLoggerSeqNumToStr(nextSeqNum, seqStr));
	strcat(fileLoggerFileStr, baseFileExt);
	/* all done */
	return TRUE;
} /*** end of FileLoggerSetFilename ***/


/************************************************************************************//**
** \brief     Open the log file for writing and automatically sets the write pointer
**            to the end of the file so data can be appended.
** \return    TRUE if successful, FALSE otherwise.
**
****************************************************************************************/
static uint8_t FileLoggerOpenFile(void)
{
	/* open the file and create a new one if it does not yet exist */
	if (f_open(&fileLoggerFileHandle, fileLoggerFileStr, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
	{
		return FALSE;
	}
	/* move file pointer to the end of the file */
	if (f_lseek(&fileLoggerFileHandle, f_size(&fileLoggerFileHandle)) != FR_OK)
	{
		return FALSE;
	}
	return TRUE;
} /*** end of FileLoggerOpenFile ***/



/************************************************************************************//**
** \brief     Writes a line to the file at the current file pointer.
** \param     line The line to add to the file.
** \return    TRUE if successful, FALSE otherwise.
**
****************************************************************************************/
static uint8_t FileLoggerAddLineToFile(const char *line)
{
	/* write the line to the file */
	if (f_puts(line, &fileLoggerFileHandle) == EOF)
	{
		return FALSE;
	}
	/* add the new line character */
	if (f_putc('\n', &fileLoggerFileHandle) == EOF)
	{
		return FALSE;
	}
	return TRUE;
} /*** end of FileLoggerAddLineToFile ***/


/************************************************************************************//**
** \brief     Checks if a filename is a log file and if so, it extracts and
**            returns the sequence number of the log file.
** \details   log files can have a sequence number name which is "filename<xxx>.ext"
**            where <xxx> is a 3 digit decimal number that automatically increments
**            with each logging session. This function validates if a filename is
**            actually a log file and if so returns the sequence number. For
**            example: FileLogerFilenameExtractSequence("mylog005.txt") returns 5.
** \param     fileToCheck Filename of the file to check.
** \param     baseFilename Base file name for log-files without file extension. "mylog"
**                         in the above example.
** \return    Log file sequence number if successful, FILELOGGER_INVALID_FILE_SEQUENCE
**            otherwise.
**
****************************************************************************************/
static int16_t FileLoggerFilenameExtractSequence(const char *fileToCheck, const char *baseFilename)
{
	uint8_t idx;
	char    sequenceStr[] = "xxx";

	/* check if the fileToCheck is actually a log file */
	if (strstr(fileToCheck, baseFilename) == NULL)
	{
		return FILELOGGER_INVALID_FILE_SEQUENCE;
	}

	/* seems to be a log-file. now find the location of the first sequence number digit */
	for (idx=0; idx<strlen(fileToCheck); idx++)
	{
		if (isdigit((int)fileToCheck[idx]) != 0)
		{
			/* found the first digit so copy the 3 digits to the sequence string */
			strncpy(sequenceStr, &fileToCheck[idx], 3);
			break;
		}
	}
	/* double check that all characters are digits */
	for (idx=0; idx<strlen(sequenceStr); idx++)
	{
		/* double check that the character is in fact a digit */
		if (isdigit((int)sequenceStr[idx]) == 0)
		{
			return FILELOGGER_INVALID_FILE_SEQUENCE;
		}
	}
	/* now convert it to a sequence number */
	return atoi(sequenceStr);
} /*** end of FileLoggerFilenameExtractSequence ***/


/************************************************************************************//**
** \brief     Converts a sequence number to a character string with leading zeroes.
** \param     value The value to convert
** \param     dest  Pointer to the destination string. Must be able to hold at least 4
**                  characters.
** \return    Pointer to the start of the resulting string.
**
****************************************************************************************/
static char *FileLoggerSeqNumToStr(uint16_t value, char *dest)
{
	/* make sure value is 3 digits */
	value = (value > 999) ? 999 : value;
	/* convert all 3 digits to ASCII characters */
	dest[0] = (value / 100) + '0';
	value = value % 100;
	dest[1] = (value / 10) + '0';
	value = value % 10;
	dest[2] = value + '0';
	/* add nul termination */
	dest[3] = '\0';
	/* return the start of the resulting string */
	return dest;
} /*** end of FileLoggerSeqNumToStr ***/


/********************************* end of filelogger.c *********************************/


