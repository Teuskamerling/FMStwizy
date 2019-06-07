/************************************************************************************//**
* \file         XcpTargetSpecific.c
* \brief        XCP functions to interface the stack with a target device
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*  Copyright (c) 2019  by HAN Automotive  http://www.han.nl           All rights reserved
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
* General information about the XCP stack functionality
****************************************************************************************
* This is a basic target independent implementation of the XCP protocol. At this time it
* only takes care of the parameter upload and variable download functions.
*
* This stack needs the following files:
*	- XcpStack.c
*	- XcpStack.h 
*	- XcpTargetSpecific.c (this file)
*	- XcpTargetSpecific.h
*
* The XcpTargetSpecific.c file holds the stack entry and exit functions wich can be 
* adapted to the desired transport communication bus and target device
*
* XcpStack.c <--> XcpTargetSpecific.c <--> target device
*
* To receive and send the data in a proper way, the incomming and outging data array's
* need to meet a specific build up:
* Incomming data from master:
* 	- dataReceived[0] 		= XCP command
* 	- dataReceived[1...n]	= XCP data
*
* Outgoing data to master:
* 	- dataToSend[0] 		= XCP Message length (excluded the byte for length and checksum)
* 	- dataToSend[1]			= XCP positive (0xff) or negative (0xfe) reply
* 	- dataToSend[2...n]		= XCP data that needs to be send back to the master
* 	- dataToSend[n+1]		= XCP checksum added if checksum is enabled.
****************************************************************************************

****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "XcpStack.h"
#include "XcpTargetSpecific.h"
#include "mac.h"					/* Needed for target specific implementation	*/

#if defined(XCPE407) || defined(XCPP405)
#include "usbcom.h"                              	/* USB COM-port header file    	*/
#include "canio.h"                               	/* CAN I/O module               */
#include "stm32f4xx_flash.h" 						/* Memory programming functions	*/	
#endif

#if defined(XCPOLIMEXINO)
#include "usbcom.h"                             	/* USB COM-port header file    	*/
#include "canio.h"                               	/* CAN I/O module              	*/  
#include "uart.h"                               	/* UART module              	*/  
#include "stm32f10x.h"   
#include "stm32f10x_conf.h"
#include "stm32f10x_flash.h" 						/* Memory programming functions	*/	 
#include "usb_regs.h"								/* Used for USB sendbuffer check*/
#endif

/****************************************************************************************
* Macro definitions
****************************************************************************************/
#define XCPUSB  1
#define XCPETH  2
#define XCPCAN  3
#define XCPUART 4

/****************************************************************************************
* Type definitions
****************************************************************************************/
typedef struct{
uint32_t xcpDtoId;
uint8_t xcpMacCanChannel;
}_xcpCanParameters;

_xcpCanParameters xcpCanParameters;

//TODO Channels must be assigned during timer creation.
_eventChannel eventChannel[3] ={
		{"EvChnl1"},
		{"EvChnl2"},
		{"EvChnl3"}};

/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void XcpUsbSend(uint8_t *data);
static void XcpCanSend(uint8_t *data);
static void XcpEthSend(uint8_t *data);
static void XcpUartSend(uint8_t *data);

/****************************************************************************************
* Data declarations
****************************************************************************************/
static uint8_t dataToSend[120] = {0};
static uint8_t xcpTransmissionBus = 0;

#if defined(XCPOLIMEXINO)
static uint8_t xcpUartChannel;
#endif

//short kXcpStationIdLengthVariable = kXcpStationIdLength;
uint32_t uniqueIdLength = kXcpStationIdLength;
char uniqueIdString[] = kXcpStationIdString;

/************************************************************************************//**
** \brief     This function configures the XCP stack interface for USB connectivity
** \param	  none.
** \return    none.
**
****************************************************************************************/
void XcpUsbConfigureParamaters(void)
{		
  xcpTransmissionBus = XCPUSB; 		// Set USB as active communication bus
}

/************************************************************************************//**
** \brief     This function is the entrypoint for XCP data received over the USB communication
**			  bus. In this function, adaptations can be made in case the incomming data is not 
**			  alligned according the demands of the XCP stack.(see general information in the 
**			  file header) 
** \param	  Pointer to de data array that holds the incomming XCP data 
** \param	  Number of relevant bytes that are available in the
** \return    none.
**
****************************************************************************************/
void XcpUsbReceive (uint8_t *dataReceived, uint32_t receivedLength)
{
  XcpCommunicationHandling(&dataReceived[1],receivedLength,&dataToSend[0]); // dataReceived start @ index 1 because 0 is total message length
}

/************************************************************************************//**
** \brief     This function is the exit point of the XCP stack. This function can be adapted
**			  according the target specific functons to send data over USB
** \param	  A pointer to the data array that holds de data to be send to the master
** \return    none.
**
****************************************************************************************/
static void XcpUsbSend(uint8_t *data)
{
  if(dataToSend[0] != 0)
  {
    /*Target specific function */
    #if defined(XCPE407) || defined(XCPP405) || defined(XCPOLIMEXINO)
    UsbComTransmit(&data[0],(data[0]+2)); // For serial communication 2 are added for Length and checksum
    #endif
  }
}


/************************************************************************************//**
** \brief     This function configures the XCP stack interface for CAN connectivity
** \param	  XCP DTO CAN identifier wich holds the identifier of the messages send to the 
**			  master
** \param	  Sets the CAN channel on wich the XCP stack needs to communicate
** \return    none.
**
****************************************************************************************/
void XcpCanConfigureParamaters(uint32_t xcpDtoId, uint8_t xcpMacCanChannel)
{
  xcpCanParameters.xcpDtoId 			= xcpDtoId;
  xcpCanParameters.xcpMacCanChannel 	= xcpMacCanChannel;
  xcpTransmissionBus 					= XCPCAN;  		// Set CAN as active communication bus
}

/************************************************************************************//**
** \brief     This function is the entrypoint for XCP data received over the CAN communication
**			  bus. In this function, adaptations can be made in case the incomming data is not 
**			  alligned according the demands of the XCP stack. (see general information in the 
**			  file header) 
** \param	  Pointer to de data array that holds the incomming XCP data 
** \param	  Number of relevant bytes that are available in the
** \return    none.
**
****************************************************************************************/
void XcpCanReceive (uint8_t *dataReceived, uint32_t receivedLength)
{
  XcpCommunicationHandling(dataReceived,receivedLength,&dataToSend[0]);
}

/************************************************************************************//**
** \brief     This function is the exit point of the XCP stack. This function can be adapted
**			  according the target specific functons to send data over CAN
** \param	  A pointer to the data array that holds de data to be send to the master
** \return    none.
**
****************************************************************************************/
void XcpCanSend(uint8_t *data)
{
  if(dataToSend[0] != 0)
  {
    /*Target specific function */
    #if defined(XCPE407) || defined(XCPP405) || defined(XCPOLIMEXINO)
    CanTransmit(xcpCanParameters.xcpMacCanChannel, xcpCanParameters.xcpDtoId , data[0], &data[1]); 
    #endif
  }
}


/************************************************************************************//**
** \brief     This function configures the XCP stack interface for UART connectivity
** \param	  none.
** \return    none.
**
****************************************************************************************/
void XcpUartConfigureParamaters(uint8_t channel)
{
#if defined(XCPOLIMEXINO)
    
  xcpUartChannel = channel;
  xcpTransmissionBus = XCPUART; // Set UART as active communication bus
  
#endif  
}

/************************************************************************************//**
** \brief     This function is the entrypoint for XCP data received over the UART communication
**			  bus. In this function, adaptations can be made in case the incomming data is not 
**			  alligned according the demands of the XCP stack.(see general information in the 
**			  file header)
**
**            The function assumes the following SxI Message frame format
**            
**            Field     | Number of bytes
**            ----------+----------------
**            LEN       | 1
**            CTR       | unused
**            XCP Packet| LEN
**            FILL      | unused
**            CS        | 1
**            
**            Refer to Figure 45 (page 55) of [1]
**            
**            [1] Patzer A., Zaiser R. (2014). XCP - The Standard Protocol for ECU Development.
**                Stuttgart: Vector Informatik GmbH
**
** \param	  Pointer to de data array that holds the incomming XCP data 
** \param	  Number of relevant bytes that are available in the
** \return    none.
**
****************************************************************************************/
void XcpUartReceive(void)
{
#if defined(XCPOLIMEXINO)
    
  static uint8_t length = 0;

  // Is a new message expected?
  if(length == 0)
  {
    // Set the number of bytes in the message
    UartGetByte(xcpUartChannel, &length);
  }
  
  // Are all expected bytes received?
  // Add one additional byte for checksum
  if(UartGetNumReceivedBytes(xcpUartChannel) == (length+1))
  {
    uint8_t data[32];
    
    // Copy the message data
    for(uint16_t i=0; i<length; ++i)
    {
      UartGetByte(xcpUartChannel, &data[i]);
    }
    
    // Get checksum
    uint8_t checksum;
    UartGetByte(xcpUartChannel, &checksum);

    // No checksum verification
    
          
    // Handle the message
    XcpCommunicationHandling(data,length,&dataToSend[0]);
    
    // Reset counter and wait for next message
    length = 0;
  }      
  
#endif
  
}

/************************************************************************************//**
** \brief     This function is the exit point of the XCP stack. This function can be adapted
**			  according the target specific functons to send data over UART
** \param	  A pointer to the data array that holds de data to be send to the master
** \return    none.
**
****************************************************************************************/
static void XcpUartSend(uint8_t *data)
{
#if defined(XCPOLIMEXINO)
    
  if(dataToSend[0] != 0)
  {
    // For serial communication 2 are added for Length and checksum
    UartPutBytes(xcpUartChannel, (char *)(&(data[0])), (data[0]+2));    
  }

#endif
  
}

/************************************************************************************//**
** \brief     This function configures the XCP stack interface for Ethernet connectivity
** \param	  none.
** \return    none.
**
****************************************************************************************/
void XcpEthConfigureParamaters(void)
{
  xcpTransmissionBus = XCPETH;   		// Set Ethernet as active communication bus
}

/************************************************************************************//**
** \brief     This function is the entrypoint for XCP data received over the Ethernet communication
**			  bus. In this function, adaptations can be made in case the incomming data is not 
**			  alligned according the demands of the XCP stack. (see general information in the 
**			  file header) 
** \param	  Pointer to de data array that holds the incomming XCP data 
** \param	  Number of relevant bytes that are available in the
** \return    none.
**
****************************************************************************************/
void XcpEthReceive (uint8_t *dataReceived, uint32_t receivedLength)
{
  XcpCommunicationHandling(dataReceived,receivedLength,&dataToSend[0]);
}

/************************************************************************************//**
** \brief     This function is the exit point of the XCP stack. This function can be adapted
**			  according the target specific functons to send data over Ethernet
** \param	  A pointer to the data array that holds de data to be send to the master
** \return    none.
**
****************************************************************************************/
static void XcpEthSend(uint8_t *data)
{
  if(dataToSend[0] != 0)
  {
    /* Target specific function */
    #ifdef XCPE407
    MacNetDtoBufferWrite(&data[1], data[0]);
    #endif
  }
}

/************************************************************************************//**
** \brief     This function sends the data from the stack to the specified communication
**			  bus.
** \param	  A pointer to the data array that holds de data to be send to the master
** \return    Send status 1 if succeded 0 if not send.
**
****************************************************************************************/
uint8_t XcpSendData(uint8_t *data)
{
	static uint8_t sendSemaphore = 0;
	
	if(sendSemaphore == 0) 
	{
		sendSemaphore = 1;
		XcpSendBufferCheck();
		switch(xcpTransmissionBus)
		{
		case XCPUSB:   XcpUsbSend(&data[0]);  	break; //External function for writing data to USB bus
		case XCPCAN:   XcpCanSend(&data[0]);  	break;
		case XCPUART:  XcpUartSend(&data[0]);  	break;	
		case XCPETH:   XcpEthSend(&data[0]);  	break;	
		}
		sendSemaphore = 0;
	return 1;
	}

return 0; // still here so message is not send.
}

/************************************************************************************//**
** \brief     This function checks the sendbuffer for the specified communication bus
**			  if the message is not yet send, wait for it.
** \param	  none
** \return    not implemented yet
**
****************************************************************************************/
//TODO implement anti deadlock mechanism
uint8_t XcpSendBufferCheck(void)
{
	switch(xcpTransmissionBus)
	{
	case XCPUSB:   
		#ifdef XCPOLIMEXINO
		while(GetEPTxStatus(ENDP1) != EP_TX_NAK);
		#endif
	break;
		
	case XCPCAN:
	break;
		
	case XCPUART:
	break;
		
	case XCPETH:
	break;	
	}

return 1;
}
/************************************************************************************//**
** \brief     This function reads the data from a specified memory location. In this 
**			  function, the way to read from a memory location can be programmed.			 
** \param	  A pointer to the data array to store the readed data in
** \param	  The number of bytes to be read from a specific memory location
** \param	  The memory location where to read the data from.
** \return    none.
**
****************************************************************************************/
void XcpReadData(uint8_t *data,uint8_t elements, uint32_t *location)
{
	switch(elements)
	{
	case 1:				*(uint8_t*)data 		= *(uint8_t*)location;			break;
	case 2:				*(uint16_t*)data 		= *(uint16_t*)location;			break;
	case 4:				*(uint32_t*)data 		= *(uint32_t*)location;			break;
	case 8:				*(uint32_t*)data 		= *(uint32_t*)location;
						*(((uint32_t*)data)+1)	= *(((uint32_t*)location)+1);	break;
	}
}

/************************************************************************************//**
** \brief     This function writes the data to a specified memory location. In this 
**			  function, the way to write to a memory location can be programmed.
** \param	  A pointer to the data array that holds the data that needs to be written
** \param	  The number of bytes to be write to a specific memory location
** \param	  The memory location where to write the data to.
** \return    none.
**
****************************************************************************************/
void XcpWriteData(uint8_t *data,uint8_t elements, uint32_t location)
{
//TODO check for write protected area's in memory
	
	switch(elements)
	{	
	case 1:				*(uint8_t*)location 		= *(uint8_t*)&data[0];		break;
	case 2:				*(uint16_t*)location 		= *(uint16_t*)&data[0];		break;
	case 4:				*(uint32_t*)location 		= *(uint32_t*)&data[0];		break;
	case 8:				*(uint32_t*)location 		= *(uint32_t*)&data[0];				
						*(((uint32_t*)location)+1) 	= *(uint32_t*)&data[4];		break;
	}
}

/* End of XcpTargetSpecific.c */
