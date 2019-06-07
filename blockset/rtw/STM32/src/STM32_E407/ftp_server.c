/************************************************************************************//**
* \file         ftp_server.c
* \brief        FTP server.
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
#include "FreeRTOS.h"								/* FreeRTOS includes */
#include "task.h"									/* Tasks of RTOS includes */
#include "api.h"									/* API for LwIp stack */
#include "tcp.h"									/* For TCP settings (disable Nagle) */
#include "ff.h"										/* API for FAT FS */
#include <stdlib.h>									/* Standard library */
#include <string.h>									/* Library for string handling */
#include "ftp_server.h"								/* FTP server include */
#include "errorcodes.h"                             /* for error codes               */
#include "errorList.h"                              /* for error list                */

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define FTPSERVER_TASK_PRIO              (tskIDLE_PRIORITY + 1)
#define FTPSERVER_TASK_STACK_SIZE        (configMINIMAL_STACK_SIZE + 1024)


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Variable that holds the period time of FileLoggerTask in OS ticks. This allows
*         a dynamic override. */
static portTickType FtpServerTaskPeriodTicks = (((portTickType)500*1000)/portTICK_PERIOD_US);

/** \brief Handle to the FTP server task. */
static xTaskHandle FtpServerTaskHandle = NULL;

/** \brief Pointer to the buffer to write or read to SD card. */ 
static uint8_t *SDbuffer;

/** \brief File system object for mounting. */
static FATFS SD_FTP_Handle;

/** \brief Pointer to request data. */
static uint8_t *request;

/** \brief Connection descriptors for FTP data port */
struct netconn *controlConn;

/** \brief Connection descriptors for FTP data port */
struct netconn *dataConn;

/** \brief String to hold the username. */
char *FTP_UserName;
/** \brief String to hold the password. */
char *FTP_Password;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void FtpGetMDTM(FILINFO *fileInfo, char *tempString);
static uint8_t FtpMainControl(struct netconn *connfd, uint8_t *request);
static int8_t FtpStartDataConnection(uint16_t port);
static uint8_t FtpQuitOrWrongRequest(struct netconn *connfd, uint8_t *request);
static uint8_t FtpSendListOfFiles(char *tempString, struct netconn *newDataConn);
static void int2str(uint32_t integer, char* string);
static uint16_t netconn_rcv_req(struct netconn *conn, uint8_t *request, void **nbuffer, uint8_t noCopyFlag);

/***************************************************************************************
** \brief     Initializes the FTP server
** \param     None
** \return    None
****************************************************************************************/
void FtpServerInit(void){
	portBASE_TYPE result;

	if( ((SDbuffer= pvPortMalloc( BLOCK_SIZE )) == NULL) || /* Allocate mem for SD buffer */
			((request= pvPortMalloc( FTP_REQUEST_SPACE )) == NULL) ) /* Allocate mem for request message data */
	{
		/* Free mem if something went wrong */
		vPortFree(SDbuffer); 
		vPortFree(request);
	}
	/* SD Card Initialization */
	if( f_mount(0, &SD_FTP_Handle) != FR_OK )
	{
		/* Free mem if something went wrong */
		vPortFree(SDbuffer);
		vPortFree(request);
		return;
	}
	
	result = xTaskCreate(FtpServerTask, "FtpServerTask", FTPSERVER_TASK_STACK_SIZE,
	NULL, FTPSERVER_TASK_PRIO, &FtpServerTaskHandle);
	if (!(result == pdPASS))
	{
		ErrCodesSetError(ER_CODE_FTP_TASK_INVALID, ER_PARAM_SEVERITY_CRITICAL, TRUE);
	}
	else
	{
		/* For MISRA-C compliance */
	}
}

/***************************************************************************************
** \brief     Sets the credentials of the FTP server
** \param     username pointer to '\0' terminated username string
** \param     password pointer to '\0' terminated password string
** \return    None
****************************************************************************************/
void FtpServerSetCredentials(char *username, char *password)
{
    FTP_UserName = pvPortMalloc(strlen(username));
    FTP_Password = pvPortMalloc(strlen(password));
    
    if((FTP_UserName == NULL) || (FTP_Password == NULL))
    {
        /* Free mem if something went wrong */
		vPortFree(FTP_UserName);
		vPortFree(FTP_Password);
        
        FTP_UserName = NULL;
        FTP_Password = NULL;
        
        return;
    }
    
    /* Copy the username and the password */
    strcpy(FTP_UserName, username);
    strcpy(FTP_Password, password);
}

/***************************************************************************************
** \brief     FTP server Task: 1 client and 1 file per transfer
** \param     Parameter pointer to use with task
** \return    None
****************************************************************************************/
void FtpServerTask( void *pvParameters )
{
	portTickType lastWakeTime;
	uint8_t sessionInfo = FTP_CLOSED;
	struct netconn *conn; /* Connection descriptor for FTP control port */
	
	/* initialise to current time */
	lastWakeTime = xTaskGetTickCount();

	/* Create a new TCP connection handle */
	conn = netconn_new(NETCONN_TCP);

	/* Bind the connection to FTP port on any local IP address */
	netconn_bind(conn, IP_ADDR_ANY, FTP_TCP_CONTROL_PORT);

	/* Put the connection into LISTEN state. */
	netconn_listen(conn);
	
	/* Start task body */
	for(;;)
	{
		if( sessionInfo == FTP_CLOSED )
		{
			if( netconn_accept(conn,&controlConn) == ERR_OK )
			{
				sessionInfo = FTP_OPEN;
			}
		}
		else /* FTP_OPEN */
		{
			/* Service connection */
			if( FtpMainControl( controlConn, request ) )
			{
				sessionInfo = FTP_CLOSED;
			}
		}
		/* Activate this task periodically */
		vTaskDelayUntil(&lastWakeTime, FtpServerTaskPeriodTicks);
	}
	/* Program should never get here */    
	return;
}

/***************************************************************************************
* FTP Server Main Control Socket Parser: requests and responses
*  Available requests: USER, PASS, PORT, QUIT
*
* @param connection descriptor
* @param buffer to hold FTP requests
* @return none
****************************************************************************************/
static uint8_t FtpMainControl(struct netconn *connClient, uint8_t *request)
{
	struct netbuf *buf = NULL;
	uint8_t userOK = FALSE;
	uint8_t correct_login = FALSE;
	uint8_t passwordAttempts = 0;
	static uint8_t dataConnCreated = 0;
	uint16_t len = 0;
	UINT bytesRead; /* This is an unsigned int 32bit, uint32_t gives a warning */
	int8_t fat_error, lwip_error;
	char* tempString = (char*) SDbuffer;
	struct netconn *newDataConn;
	FIL file; /* File object handle */
	char FTP_dataRepresentationType = 0;
	
	/* Send FTP server first RESPONSE */
	netconn_write(connClient,FTP_WELCOME_RESPONSE,strlen(FTP_WELCOME_RESPONSE),NETCONN_NOCOPY);

	while (1) /* Stay in this loop until 'break' is called */
	{
		/* Check for REQUEST from client */
		len = netconn_rcv_req(connClient, request, NULL,0);
		/* Stop when fatal error occurred */
		if(ERR_IS_FATAL(connClient->last_err))
		{
			break; 
		} 
		if(len == 0)
		{
			/* Nothing received yet */
			continue;
		}
		if(connClient->last_err == ERR_OK) /* No errors */
		{
			/* Read data from buffer */
			netbuf_data( buf, (void **) &request, &len);
			
			/* First the USER request must be received */
			if (!userOK && strstr((const char *)request,(const char *)FTP_USER_REQUEST) == NULL)
			{
				/* Expected USER request */
				netconn_write(connClient,FTP_BAD_SEQUENCE_RESPONSE,strlen(FTP_BAD_SEQUENCE_RESPONSE),NETCONN_NOCOPY);
				continue; /* Go back to beginning */
			}
			/* Secondly the PASS request must be received */
			if (userOK && !correct_login && strstr((const char *)request,(const char *)FTP_PASS_REQUEST) == NULL)
			{
				/* Expected PASS request */
				netconn_write(connClient,FTP_BAD_SEQUENCE_RESPONSE,strlen(FTP_BAD_SEQUENCE_RESPONSE),NETCONN_NOCOPY);
				continue; /* Go back to beginning */
			}
			
			/******** USER request ********/
			if( strstr((const char *)request,(const char *)FTP_USER_REQUEST) != NULL) 
			{          
				/* Check valid user name */
				if(!userOK && !strncmp((const char *)&request[sizeof(FTP_USER_REQUEST)],FTP_UserName,strlen(FTP_UserName)))
				{
					/* Response: user OK */
					netconn_write(connClient,FTP_USER_RESPONSE,strlen(FTP_USER_RESPONSE),NETCONN_NOCOPY);
					userOK = TRUE;
					continue;
				} else {
					/* Response: user Failed */
					netconn_write(connClient,FTP_USER_FAIL_RESPONSE,strlen(FTP_USER_FAIL_RESPONSE),NETCONN_NOCOPY);
				}
			/******** PASS request ********/
			}else if(userOK && strstr((const char *)request,(const char *)FTP_PASS_REQUEST) != NULL ) /* User password request */
			{
				/* Authentication process: password must match exactly */
				if( !strncmp((const char *)&request[sizeof(FTP_PASS_REQUEST)],FTP_Password,strlen(FTP_Password)) )
				{
					/* Response: Password OK */
					netconn_write(connClient,FTP_PASS_OK_RESPONSE,strlen(FTP_PASS_OK_RESPONSE),NETCONN_NOCOPY);
					correct_login = TRUE;
				} else
				{
					/* Response: Password FAILED */
					netconn_write(connClient,FTP_USER_FAIL_RESPONSE,strlen(FTP_USER_FAIL_RESPONSE),NETCONN_NOCOPY);
					passwordAttempts++;
					if(passwordAttempts > 5)
					{
						break; /* Stop session and break connection */
					}
				}
			}
			/******** PWD request ********/
			else if(strstr((const char *)request,(const char *)FTP_PWD_REQUEST) != NULL ) /* Search for FPT_PWD_REQUEST in alloq_rq and return pointer to it*/
			{
				tempString[0] = 0; /* Empty tempString string */
				strcat(tempString,"257 \"/"); /* Glue 257 "\ in the front of the string */
				if(f_getcwd(&tempString[6], 50) == FR_OK){ /* Get current working dir */
					strcpy(&tempString[6], &tempString[9]); /* Delete the volume number '0:' */
					strcat(tempString,"\"\r\n"); /* Glue <CR><LF> behind the string */
					netconn_write(connClient,tempString,strlen(tempString),NETCONN_NOCOPY);
				} else {
					netconn_write(connClient,FTP_WRITE_FAIL_RESPONSE,strlen(FTP_WRITE_FAIL_RESPONSE),NETCONN_NOCOPY); /* 550 File unavailable */
				}
			}
			/******** TYPE request ********/
			else if(strstr((const char *)request,(const char *)FTP_TYPE_REQUEST) != NULL )
			{
				if(request[5] == 'I' || request[5] == 'A'){ /* Check if client requests binary mode or ASCI mode */
					netconn_write(connClient,FTP_COMMAND_OK,strlen(FTP_COMMAND_OK),NETCONN_NOCOPY);
					FTP_dataRepresentationType = request[5]; /* Save the TYPE for later use */
				}else{
					netconn_write(connClient,FTP_CMD_NOT_IMP_RESPONSE,strlen(FTP_CMD_NOT_IMP_RESPONSE),NETCONN_NOCOPY);
				}
			}
			/******** PORT request ********/
			else if(strstr((const char *)request,(const char *)FTP_PORT_REQUEST) != NULL ) /* Search for FPT_PORT_REQUEST in alloq_rq and return pointer to it*/
			{
				//TODO Jason: implement
			}
			/******** DELETE request ********/
			else if( strstr((const char *)request,(const char *)FTP_DELE_REQUEST) != NULL )
			{
				tempString[0] = 0; /* Clear tempString */
				/* Get file name from message:DELE file\r\n */
				strcat(tempString, (const char*) &request[5]);/* remove 'DELE ' and glue on tempString */
				tempString[strlen(tempString)-2] = 0; /* remove last 3 bytes (\r\n & 0) and replace by 0 */
				if (FR_OK == f_unlink(tempString)) /* Delete file */
				{
					/* Response: OK */
					netconn_write(connClient,FTP_FILE_ACT_OK_RESPONSE,strlen(FTP_FILE_ACT_OK_RESPONSE),NETCONN_NOCOPY);
				}
				else
				{
					/* Response: FAILED */
					netconn_write(connClient,FTP_WRITE_FAIL_RESPONSE,strlen(FTP_WRITE_FAIL_RESPONSE),NETCONN_NOCOPY);
				}
			} 
			/******** SYST request ********/
			else if ( strstr((const char *)request,(const char *)FTP_SYST_REQUEST) != NULL )
			{
				/* Response: Windows NT */
				netconn_write(connClient,FTP_SYST_RESPONSE,strlen(FTP_SYST_RESPONSE),NETCONN_NOCOPY);
			}
			/******** FEAT request ********/
			else if ( strstr((const char *)request,(const char *)FTP_FEAT_REQUEST) != NULL )
			{
				/* Response: 211 extended features: SIZE MDTM */
				netconn_write(connClient,FTP_FEAT_RESPONSE,strlen(FTP_FEAT_RESPONSE),NETCONN_NOCOPY);
				netconn_write(connClient,FTP_FEAT_RESPONSE2,strlen(FTP_FEAT_RESPONSE2),NETCONN_NOCOPY);
			}
			/******** PASV request ********/
			else if ( strstr((const char *)request,(const char *)FTP_PASV_REQUEST) != NULL )
			{
				/* Prepare the connection for data on port 28339 */
				if(dataConnCreated == 0){
					if(FtpStartDataConnection(28339) == ERR_OK){dataConnCreated = 1;}
				}
				tempString[0] = 0; /* Empty the temporary string */
				strcat(tempString,FTP_PASV_RESPONSE); /* Write the first part of the response to the string */
				strcat(tempString," (169,254,19,63,110,179).\r\n"); /* Add the IP address and port to the string, port 28339 */
				/* Response: PASV positive response */
				netconn_write(connClient,tempString,31,NETCONN_NOCOPY);
			}
			/******** LIST request ********/
			else if ( strstr((const char *)request,(const char *)FTP_LIST_REQUEST) != NULL )
			{
				/* Accept incoming connection */
				if(netconn_accept(dataConn,&newDataConn) == ERR_OK)
				{
					/* Response: OK */
					netconn_write(connClient,FTP_FILE_OK_RESPONSE,strlen(FTP_FILE_OK_RESPONSE),NETCONN_NOCOPY);
				}
				/* Prepare the list and send it to the client */
				fat_error = FtpSendListOfFiles(tempString, newDataConn);
				if(fat_error == ERR_OK)
				{
					/* Only respond positive if data was send */
					netconn_write(connClient,FTP_TRANSFER_OK_RESPONSE,strlen(FTP_TRANSFER_OK_RESPONSE),NETCONN_NOCOPY);
				}else{
					/* Error occurred */
					netconn_write(connClient,FTP_ACT_ABORTED,strlen(FTP_ACT_ABORTED),NETCONN_NOCOPY);
				}
				/* Close and delete connection */
				netconn_close(newDataConn);
				netconn_delete(newDataConn);
			}
			/******** RETR request ********/
			else if ( strstr((const char *)request,(const char *)FTP_RETR_REQUEST) != NULL )
			{
				/* Clear tempString */
				tempString[0] = 0;
				/* Get file name from message:RETR file\r\n */
				strcat(tempString, (const char*) &request[5]);/* remove 'RETR ' and glue on tempString*/
				tempString[strlen(tempString)-2] = 0; /* remove last 3 bytes (\r\n & 0) and replace by 0 */
				/* Open file */
				if(f_open(&file, tempString, FA_READ) != FR_OK){
					netconn_write(connClient,FTP_WRITE_FAIL_RESPONSE,strlen(FTP_WRITE_FAIL_RESPONSE),NETCONN_NOCOPY); /* 550 File unavailable */
					continue; /* Exit this case and continue with processing incoming packages */
				}
				/* Accept incoming connection */
				if(netconn_accept(dataConn,&newDataConn) == ERR_OK)
				{
					/* Response 150 Opening BIN/ASCII data connection */
					netconn_write(connClient,FTP_FILE_OK_RESPONSE,strlen(FTP_FILE_OK_RESPONSE),NETCONN_NOCOPY);
				}
				/* Read data in chunks of BLOCK_SIZE bytes and send over data connection */
				for(;;){
					fat_error = f_read(&file, SDbuffer, BLOCK_SIZE, &bytesRead);
					if(fat_error || bytesRead == 0) break; /* Break on error or end of file*/
					lwip_error = netconn_write(newDataConn,SDbuffer,bytesRead,NETCONN_COPY);
					if(lwip_error != ERR_OK) break; /* Break on error */
				}
				f_close(&file); /* Close file */

				if(lwip_error == ERR_OK && fat_error == FR_OK){
					/* Response 226 Transfer complete */
					netconn_write(connClient,FTP_TRANSFER_OK_RESPONSE,strlen(FTP_TRANSFER_OK_RESPONSE),NETCONN_NOCOPY);
				}else{
					/* Response: 451 Local error occurred */
					netconn_write(connClient,FTP_ACT_ABORTED,strlen(FTP_ACT_ABORTED),NETCONN_NOCOPY);
				}
				/* Close data and delete data connection */
				netconn_close(newDataConn);
				netconn_delete(newDataConn);
			}
			/******** STOR request ********/
			else if ( strstr((const char *)request,(const char *)FTP_STOR_REQUEST) != NULL )
			{
				netconn_write(connClient,FTP_WRITE_FAIL_RESPONSE,strlen(FTP_WRITE_FAIL_RESPONSE),NETCONN_NOCOPY); /* 550 Access is denied, Upload not implemented */
			}
			/******** CWD request ********/
			else if ( strstr((const char *)request,(const char *)FTP_CWD_REQUEST) != NULL )
			{
				/* Clear tempString */
				tempString[0] = 0;
				/* Get directory from message:CWD dir\r\n */
				strcat(tempString, (const char*) &request[4]);/* remove 'CWD ' and glue on tempString*/
				tempString[strlen(tempString)-2] = 0; /* remove last 3 bytes (\r\n & 0) and replace by 0 */
				if(tempString[strlen(tempString)-1] == '/') tempString[strlen(tempString)-1] = 0; /*Remove slash*/
				/* Change directory */
				if(f_chdir(tempString) == FR_OK)
				{
					/* Response 250 CWD command successful */
					netconn_write(connClient,FTP_FILE_ACT_OK_RESPONSE,strlen(FTP_FILE_ACT_OK_RESPONSE),NETCONN_NOCOPY);
				}else{
					/* Error occured */
					netconn_write(connClient,FTP_ACT_ABORTED,strlen(FTP_ACT_ABORTED),NETCONN_NOCOPY);
				}
			}
			/******** CDUP request ********/
			else if ( strstr((const char *)request,(const char *)FTP_CDUP_REQUEST) != NULL )
			{
				/* Cchange directory up */
				if(f_chdir("/..") == FR_OK)
				{
					/* Response 250 CDUP command successful */
					netconn_write(connClient,FTP_FILE_ACT_OK_RESPONSE,strlen(FTP_FILE_ACT_OK_RESPONSE),NETCONN_NOCOPY);
				}else{
					/* Error occured */
					netconn_write(connClient,FTP_ACT_ABORTED,strlen(FTP_ACT_ABORTED),NETCONN_NOCOPY);
				}
			}
			/******** MKD request ********/
			else if ( strstr((const char *)request,(const char *)FTP_MKD_REQUEST) != NULL )
			{
				/* Clear tempString */
				tempString[0] = 0;
				/* Get directory name from message:MKD directory\r\n */
				strcat(tempString, (const char*) &request[4]);/* remove 'RETR ' and glue on tempString*/
				tempString[strlen(tempString)-2] = 0; /* remove last 3 bytes (\r\n & 0) and replace by 0 */
				/* Try to make the directory */
				if(f_mkdir(tempString) == FR_OK){
					/* Succesfull creation */
					netconn_write(connClient,FTP_FILE_ACT_OK_RESPONSE,strlen(FTP_FILE_ACT_OK_RESPONSE),NETCONN_NOCOPY);
				}else{
					/* Failed creation */
					netconn_write(connClient,FTP_WRITE_FAIL_RESPONSE,strlen(FTP_WRITE_FAIL_RESPONSE),NETCONN_NOCOPY);
				}
			}
			/******** RMD request ********/
			else if ( strstr((const char *)request,(const char *)FTP_RMD_REQUEST) != NULL )
			{
				/* Clear tempString */
				tempString[0] = 0;
				/* Get directory name from message:RMD directory\r\n */
				strcat(tempString, (const char*) &request[5]);/* remove 'RETR /' and put it in tempString */
				tempString[strlen(tempString)-2] = 0; /* remove last 3 bytes (\r\n & 0) and replace by 0 */
				if(tempString[strlen(tempString)-1] == '/') tempString[strlen(tempString)-1] = 0; /* Remove slash */
				/* Exit the directory so it can be removed */
				f_chdir("/..");
				/* Try to delete the directory */
				if(f_unlink(tempString) == FR_OK){
					/* Succesfull creation */
					netconn_write(connClient,FTP_FILE_ACT_OK_RESPONSE,strlen(FTP_FILE_ACT_OK_RESPONSE),NETCONN_NOCOPY);
				}else{
					/* Failed creation */
					netconn_write(connClient,FTP_WRITE_FAIL_RESPONSE,strlen(FTP_WRITE_FAIL_RESPONSE),NETCONN_NOCOPY);
				}
			}
			/******** RESTART request ********/
			else if( strstr((const char *)request,(const char *)FTP_RESTART_REQUEST) != NULL ) //TODO Jason: This is not correctly implemented, only implemented because it is used by WinSCP as keep alive
			{
				/* Response: TODO Jason: Add real functionality here */
				netconn_write(connClient,FTP_FILE_ACTION_PENDING,strlen(FTP_FILE_ACTION_PENDING),NETCONN_NOCOPY);
			} 
			/******** SIZE request ********/
			else if( strstr((const char *)request,(const char *)FTP_SIZE_REQUEST) != NULL ) //TODO Jason: This is not correctly implemented, only implemented because it is used by WinSCP as keep alive
			{
				/* Response: TODO Jason: add real functionality here */
				netconn_write(connClient,"213 100\r\n",strlen("213 100\r\n"),NETCONN_NOCOPY);
			} 
			/******** Unimplemented request ********/
			else
			{
				if( !FtpQuitOrWrongRequest(connClient,request) )
				{
					break;/* Quit command */
				}
			}
		} /* End of no errors if statement */
	}/* End of while loop */ 

	/* Closing the session */
	netconn_close(connClient);
	netconn_delete(connClient);
	return 1;/* Return close value */
} /*** End of function FtpMainControl() ***/


/***************************************************************************************
* Build a WIN style list of all file names in FAT and send it over the connection
*
* @param tempString pointer to string where to store the list
* @param newDataConn pointer to netconn connection to use for sending
* @return 0 if all is OK, otherwise 1
****************************************************************************************/
static uint8_t FtpSendListOfFiles(char *tempString, struct netconn *newDataConn)
{
	FRESULT errorStatus = 1;
	DIR dir;
	uint8_t length = 100; /* Maximum length of path name */
	FILINFO fno;
#if _USE_LFN
	static char lfn[_MAX_LFN +1];
	fno.lfname = lfn;
	fno.lfsize = sizeof(lfn);
#endif
	char currentDir[length], fileData[32];
	uint32_t year, day, hour, minute;
	uint8_t month;

	tempString[0] = 0;	/* Empty the temporary string */
	fileData[0] = 0;		/* Empty the fileData string */
	errorStatus = f_getcwd(currentDir, length);
	if(errorStatus == FR_OK) /* list will be made of current working directory */
	{
		errorStatus = f_opendir(&dir, currentDir);
		if (errorStatus == FR_OK) {  /* Open the directory */
			for (;;) {
				errorStatus = f_readdir(&dir, &fno);					/* Read a directory item */
				if (errorStatus != FR_OK || fno.fname[0] == 0) break; 	/* Break on error or end of list */
				/* Month bit 8:5 */
				month = (fno.fdate & 0x1E0) >> 5;
				int2str(month, fileData);
				if (month < 10) strcat(tempString, "0"); /* Add a zero before month */
				strcat(tempString, fileData);
				strcat(tempString, "-");
				/* Calculate the day: bit 4:0 */
				day = fno.fdate & 0x1F;
				int2str(day, fileData);
				if (day < 10) strcat(tempString, "0"); /* Add a zero before day */
				strcat(tempString, fileData);
				strcat(tempString, "-");
				/* Calculate the year: bit 15:9 from 1980 */
				year = (1980 + ((fno.fdate & 0xFE00) >> 9));
				if(year > 1999) year = year - 2000; /* Take off 2000 to get '17 */
				int2str(year, fileData);
				if (year < 10) strcat(tempString, "0"); /* Add a zero before year */
				strcat(tempString,fileData);
				strcat(tempString, "    ");
				/* Calculate the hours, bit 15:11*/
				hour = (fno.ftime & 0xF800) >> 11;
				int2str(hour, fileData);
				if (hour < 10) strcat(tempString, "0"); /* Add a zero before hour */
				if (hour == 0) strcat(tempString, "0"); /* Add another zero's for hour */
				strcat(tempString,fileData);
				strcat(tempString, ":");
				/* Calculate the minutes, bit 10:5 */
				minute = (fno.ftime & 0x7E0) >> 5;
				int2str(minute, fileData);
				if (minute < 10) strcat(tempString, "0"); /* Add a zero before minute */
				if (minute == 0) strcat(tempString, "0"); /* Add another zero's for minute */
				strcat(tempString,fileData);
				strcat(tempString, "    ");
				if (fno.fattrib & AM_DIR) {                     /* It is a directory */
					strcat(tempString, "<DIR> ");
				} else {                                        /* It is a file. */
					/* Add the file size */
					int2str(fno.fsize, fileData); /* uint32_t */
					strcat(tempString, fileData);
					strcat(tempString, "    ");
				}
				if(*fno.lfname){ /* If possible use long file name */
					strcat(tempString, fno.lfname);
				}else{
					strcat(tempString, fno.fname); /* Else use short 8.3format */
				}
				strcat(tempString, "\r\n");
				if(strlen(tempString) >= (BLOCK_SIZE - 30)){
					errorStatus = netconn_write(newDataConn,tempString,strlen(tempString),NETCONN_COPY);
					/* Reset tempString */
					tempString[0] = 0;
				}
			} /* End of for loop */
			errorStatus = netconn_write(newDataConn,tempString,strlen(tempString),NETCONN_NOCOPY);
		} /* End of Open dir */
	}
	return errorStatus;
} /*** end of FtpCreateListOfFiles ***/


/***************************************************************************************
* Creat a string with the Month Date and Time modified
*
* @param FILINFO pointer to the file information struct
* @param tempString pointer to string where to store the string
****************************************************************************************/
static void FtpGetMDTM(FILINFO *fileInfo, char *tempString){
	char fileData[5];
	uint32_t year, day, hour, minute;
	uint8_t month;	
	
	tempString = 0; /* clear tempString */
	strcat(tempString, "213 "); /* Start with the response code */
	/* Calculate the year: bit 15:9 from 1980 */
	year = (1980 + ((fileInfo->fdate & 0xFE00) >> 9));
	int2str(year, fileData); /* Convert to string */
	strcat(tempString,fileData);
	/* Month bit 8:5 */
	month = (fileInfo->fdate & 0x1E0) >> 5;
	int2str(month, fileData);
	if (month < 10) strcat(tempString, "0"); /* Add a zero before month */
	strcat(tempString, fileData);
	/* Calculate the day: bit 4:0 */
	day = fileInfo->fdate & 0x1F;
	int2str(day, fileData);
	if (day < 10) strcat(tempString, "0"); /* Add a zero before day */
	strcat(tempString, fileData);
	/* Calculate the hours, bit 15:11*/
	hour = (fileInfo->ftime & 0xF800) >> 11;
	int2str(hour, fileData);
	if (hour < 10) strcat(tempString, "0"); /* Add a zero before hour */
	if (hour == 0) strcat(tempString, "0"); /* Add another zero's for hour */
	strcat(tempString,fileData);
	/* Calculate the minutes, bit 10:5 */
	minute = (fileInfo->ftime & 0x7E0) >> 5;
	int2str(minute, fileData);
	if (minute < 10) strcat(tempString, "0"); /* Add a zero before minute */
	if (minute == 0) strcat(tempString, "0"); /* Add another zero's for minute */
	strcat(tempString,fileData);
	strcat(tempString,"00\r\n"); /* Seconds are always 0 */
} /*** end of FtpCreateListOfFiles ***/


/***********************************************************************
* Convert integer to string
*
* @param integer to convert (only unsigned)
* @return pointer to string
************************************************************************/
static void int2str(uint32_t integer, char* string)
{
	uint8_t NrOfChars = 0;
	uint32_t value;
	uint8_t i;

	value = integer;
	/* Count number of characters */
	while(integer){
		NrOfChars++;
		integer /= 10;
	}
	/* Fill string */
	string[NrOfChars] = 0; /* End character */
	for(i=NrOfChars;i>0;i--){
		string[i-1]=(value%10)+ '0';
		value /= 10;
	}
} /*** End of int2str ***/


/***************************************************************************************
* Starts the passive connection for the data transfer
*
* 
****************************************************************************************/
static int8_t FtpStartDataConnection(uint16_t port)
{
	uint8_t result;
	/* Create a new TCP connection handle */
	dataConn = netconn_new(NETCONN_TCP);

	/* disable Nagle's algorithm */
	tcp_nagle_disable(dataConn->pcb.tcp); /* Todo: Jason this doesn't seem to work, do we need it? */

	/* Bind the connection to data port on any local IP address */
	netconn_bind(dataConn, IP_ADDR_ANY, port);

	/* Put the connection into LISTEN state */
	result = netconn_listen(dataConn);
	return result;
} /*** End of FtpStartDataConnection() ***/


/***************************************************************************************
* Closes or Leave session depending on client request
*
* @param connection descriptor
* @param buffer space 
* @return 0 keep session, otherwise session needs to be closed
****************************************************************************************/
static uint8_t FtpQuitOrWrongRequest(struct netconn *connfd, uint8_t *request)
{
	if( strstr((const char *)request,(const char *)FTP_QUIT_REQUEST) != NULL )
	{
		/* Response: Quit session */
		netconn_write(connfd,FTP_QUIT_RESPONSE,strlen(FTP_QUIT_RESPONSE),NETCONN_NOCOPY);
		return 1; /* Close session */
	}
	else
	{
		/* Response: Unknown request */
		netconn_write(connfd,FTP_UNKNOWN_RESPONSE,strlen(FTP_UNKNOWN_RESPONSE),NETCONN_NOCOPY);
		return 0;/* Keep session */
	}  
}





























// LEGACY CODE


/**
* Receives tcp/udp information copying to a static
*  array or use network buffer directly depending on flag var
*  Info is received thru tcp/udp/raw connection descriptor
*  Features: reentrant
*
* @param connection descriptor
* @param static array to be used to copy network buffers
* @param selector from apps array or use directly from lwIP network buffers
* @param network buffer pointer of pointer
* @return length of buffer. Read conn->err for details:
*    OK, (ERR_OK) CLSD (ERR_CLSD), TIMEOUT (ERR_TIMEOUT), OUT OF MEM (ERR_MEM)
*/
uint16_t netconn_rcv_req(struct netconn *conn, uint8_t *request, void **nbuffer, uint8_t noCopyFlag)
{
	struct netbuf *inbuf;
	struct pbuf *q;
	uint16_t len = 0;

	/* Receive the packet and check if data was received */
	if( (netconn_recv(conn,&inbuf)==ERR_OK) && inbuf != NULL )
	{
		/* If receiver is expecting a big rx packet, use it directly from the network buffers */
		if(noCopyFlag)
		{
			/* Use buffer directly from lwIP network buffers */
			len = inbuf->ptr->tot_len;
			*nbuffer = (void *)inbuf;
			return len;
		} else { /* If not you can copy it to a small buffer */
			/* Start segment index*/
			q = inbuf->ptr;
			do
			{
				memcpy( &request[len], q->payload, q->len );
				len += q->len;
			}
			while( ( q = q->next ) != NULL );

			/*NULL char terminator. Useful for ASCII transfers*/
			request[len] = '\0';

			/* Free pbuf memory */
			netbuf_delete(inbuf);
		}
	}
	return len;/*return value*/
}
