/************************************************************************************//**
* \file         can.c
* \brief        CAN driver source file.
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
#include "can.h"                                      /* CAN driver header file        */
#include "errorcodes.h"                               /* for error codes               */
#include "errorList.h"                                /* for error list                */
#include "stm32f4xx.h"                                /* STM32 registers               */
#include "stm32f4xx_conf.h"                           /* STM32 peripheral drivers      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* can controller constants */
#define CAN_CLOCK_SPEED_HZ              (configCPU_CLOCK_HZ/4)
#define CAN_BAUD_MIN                    (5000)
#define CAN_BAUD_MAX                    (1000000)
#define CAN_BAUD_TOLERANCE_PCT          (2)
#define CAN_PRESCALER_MIN               (1)
#define CAN_PRESCALER_MAX               (1024)
#define CAN_NUM_TX_BUFFERS              (3)
#define CAN_NUM_RX_BUFFERS              (2)
/* event handler task */
#define CAN_EVENT_TASK_PRIO             (tskIDLE_PRIORITY + 10)
#define CAN_EVENT_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define CAN_EVENT_QUEUE_SIZE_MIN        (4)
/* transmission task */
#define CAN_TRANSMIT_QUEUE_SIZE_MIN     (2)
#define CAN_TRANSMIT_TASK_PRIO          (tskIDLE_PRIORITY + 2)
#define CAN_TRANSMIT_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)
#define CAN_TRANSMIT_QUEUE_SIZE         (CAN_TRANSMIT_BUFFER_SIZE)
#define CAN_TRANSMIT_QUEUE_FULL_DELAY   ((((portTickType)5*1000)/portTICK_PERIOD_US))
/* can interrupt priority */
#define CAN_NVIC_IRQ_PRIO               (configLIBRARY_KERNEL_INTERRUPT_PRIORITY)
/* miscellaneous */
#define CAN_MSG_TRANSMIT_MAX_TIME       (((portTickType)50*1000)/portTICK_PERIOD_US)
/** \brief Configures the total number of callbacks that can be configured through
 *         CanRegisterTransmittedCallback.
 */
#define CAN_TRANSMITTED_CALLBACKS_MAX   (2)
/** \brief Configures the total number of callbacks that can be configured through
 *         CanRegisterReceivedCallback.
 */
#define CAN_RECEIVED_CALLBACKS_MAX      (2)
/** \brief Configures the total number of callbacks that can be configured through
 *         CanRegisterErrorCallback.
 */
#define CAN_ERROR_CALLBACKS_MAX         (2)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Bit timing structure. */
typedef struct t_can_bus_timing
{
  uint8_t prop;                                     /**< CAN propagation segment       */
  uint8_t phase1;                                   /**< CAN phase segment 1           */
  uint8_t phase2;                                   /**< CAN phase segment 2           */
} tCanBitTiming;       

/** \brief CAN message type. */
typedef struct t_can_message
{
  uint8_t channel;                                  /**< can controller channel        */
  uint32_t id;                                      /**< message identifier            */
  uint8_t  dlc;                                     /**< data length code              */
  uint8_t  data[CAN_MAX_DATA_LEN];                  /**< message data                  */
  uint32_t timestamp;                               /**< message time stamp            */
} tCanMessage;

/** \brief CAN event type. */
typedef enum 
{                     
  CAN_EVENT_MSG_TXED = 0,                           /**< message transmitted event     */
  CAN_EVENT_MSG_RXED,                               /**< message received event        */
  CAN_EVENT_RX_OVERRUN,                             /**< hardware rx slot overrun event*/
  CAN_EVENT_BUS_OFF,                                /**< bus off event                 */
} tCanEventType;

/** \brief CAN event info */
typedef struct t_can_event_info
{
  uint8_t channel;                                  /**< can controller channel        */
  tCanEventType type;                               /**< event type                    */
  tCanMessage   msg;                                /**< message related to the event  */
} tCanEventInfo;

/**< \brief Structure type with CAN controller mapping information. */
typedef struct
{
  CAN_TypeDef * can_peripheral;
  uint32_t can_peripheral_clk;
  uint32_t can_rx_port_peripheral_clk;
  GPIO_TypeDef* can_rx_port;
  uint16_t can_rx_pin;
  uint16_t can_rx_source;
  uint32_t can_tx_port_peripheral_clk;
  GPIO_TypeDef* can_tx_port;
  uint16_t can_tx_pin;
  uint16_t can_tx_source;
  uint8_t gpio_af;
  uint8_t tx_irq_channel;
  uint8_t rx0_irq_channel;
  uint8_t rx1_irq_channel;
  uint8_t sce_irq_channel;
} tCanControllerMapping;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static uint8_t  CanCopyAndStartTransmit(tCanMessage *msg);
static void     CanTransmitTask(void *pvParameters);
static void     CanEventTask(void *pvParameters);
static uint8_t  CanGetSpeedConfig(uint32_t baud, uint16_t *prescaler, tCanBitTiming *bitTiming);
static void     CanConfigureInterrupt(uint8_t channel, uint8_t enable);


/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief Array with all configuration parameters of a CAN controller. */
const static tCanControllerMapping canControllerMapping[CAN_CHANNELS_MAX] =
{
  { CAN1, RCC_APB1Periph_CAN1, RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_8,  GPIO_PinSource8,  RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_9,  GPIO_PinSource9,  GPIO_AF_CAN1, CAN1_TX_IRQn, CAN1_RX0_IRQn, CAN1_RX1_IRQn, CAN1_SCE_IRQn },
  { CAN2, RCC_APB1Periph_CAN2, RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_12, GPIO_PinSource12, RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_13, GPIO_PinSource13, GPIO_AF_CAN2, CAN2_TX_IRQn, CAN2_RX0_IRQn, CAN2_RX1_IRQn, CAN2_SCE_IRQn }
};


/** \brief According to the CAN protocol 1 bit-time can be made up of between 8..25 time
 *         quanta (TQ). The total TQ in a bit is SYNC + PROP + PHASE1 + PHASE2 with SYNC
 *         always being 1. The sample point is (SYNC + PROP + PHASE1) / (SYNC + PROP +
 *         PHASE1 + PHASE2) * 100%. This array contains possible and valid time quanta
 *         configurations with a sample point between 60..78%.
 */
static const tCanBitTiming canTiming[] =
{                     /*  TQ | PROP | PHASE1 | PHASE2 | SP  */
                      /* ---------------------------------- */
  { 1, 1, 2 },       /*   5 |   1  |   1    |   2    | 60% */
  { 1, 2, 2 },       /*   6 |   1  |   2    |   2    | 67% */
  { 2, 2, 2 },       /*   7 |   2  |   2    |   2    | 71% */
  { 3, 2, 2 },       /*   8 |   3  |   2    |   2    | 75% */
  { 4, 2, 2 },       /*   9 |   4  |   2    |   2    | 78% */
  { 3, 3, 3 },       /*  10 |   3  |   3    |   3    | 70% */
  { 4, 3, 3 },       /*  11 |   4  |   3    |   3    | 73% */
  { 5, 3, 3 },       /*  12 |   5  |   3    |   3    | 75% */
  { 6, 3, 3 },       /*  13 |   6  |   3    |   3    | 77% */
  { 5, 4, 4 },       /*  14 |   5  |   4    |   4    | 71% */
  { 6, 4, 4 },       /*  15 |   6  |   4    |   4    | 73% */
  { 7, 4, 4 },       /*  16 |   7  |   4    |   4    | 75% */
  { 8, 4, 4 },       /*  17 |   8  |   4    |   4    | 76% */
  { 7, 5, 5 },       /*  18 |   7  |   5    |   5    | 72% */
  { 8, 5, 5 },       /*  19 |   8  |   5    |   5    | 74% */
  { 8, 6, 5 },       /*  20 |   8  |   6    |   5    | 75% */
  { 8, 7, 5 },       /*  21 |   8  |   7    |   5    | 76% */
  { 8, 7, 6 },       /*  22 |   8  |   7    |   6    | 73% */
  { 8, 8, 6 },       /*  23 |   8  |   8    |   6    | 74% */
  { 8, 8, 7 },       /*  24 |   8  |   8    |   7    | 71% */
  { 8, 8, 8 }        /*  25 |   8  |   8    |   8    | 68% */
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Queue handle for the CAN event queue. */
static xQueueHandle     canEventPendingQ;
/** \brief Queue handle for the CAN transmit queue. */
static xQueueHandle     canTransmitQ;
/** \brief Task handle for the CAN event task. */
static xTaskHandle      canEventTaskHandle;
/** \brief Task handle for the CAN transmit task. */
static xTaskHandle      canTransmitTaskHandle;
/** \brief Array to keep track of the message id that is being transmitted in a hardware
 *         message slot. This is necessary to know the identifier in the application
 *         event callback.
 */
static uint32_t         canTxPendingId[CAN_CHANNELS_MAX][CAN_NUM_TX_BUFFERS];
/** \brief Timestamp of when the CAN driver was connected to the CAN bus to be able to
 *         have message timestamps that start at 0 ms.
 */
static uint32_t         canStartTimeMs[CAN_CHANNELS_MAX];
/** \brief CAN connection state variable. */
static uint8_t          canConnected[CAN_CHANNELS_MAX];
/** \brief Size of the transmit queue. */
static uint16_t         canTransmitQsize;
/** \brief Size of the event queue. */
static uint16_t         canEventQsize;
/** \brief Function pointer for the generic message transmitted callback handler. */
static tCanCallbackGenericTransmitted canCallbackGenericTransmitted[CAN_CHANNELS_MAX][CAN_TRANSMITTED_CALLBACKS_MAX];
/** \brief Function pointer for the generic message received callback handler. */
static tCanCallbackGenericReceived canCallbackGenericReceived[CAN_CHANNELS_MAX][CAN_RECEIVED_CALLBACKS_MAX];
/** \brief Function pointer for the CAN error callback handler. */
static tCanCallbackError canCallbackError[CAN_CHANNELS_MAX][CAN_ERROR_CALLBACKS_MAX];


/************************************************************************************//**
** \brief     Initializes the CAN driver. Prior to calling this function. Note that
**            this function should only be called once, typically during the
**            software program initialization.
** \param     transmitBufferSize Size of the transmit buffer. This is in addition
**                               to the already available 3 hardware transmit slots.
** \param     eventBufferSize Size of the buffer to store events such as new message
**                            reception, hardware reception slot overrun and bus off.
** \return    none.
**
****************************************************************************************/
void CanInit(uint16_t transmitBufferSize, uint16_t eventBufferSize)
{
  uint8_t idx;
  uint8_t cbIdx;
  portBASE_TYPE    result;

  /* initialize the callback handlers and connection state */
  for (idx=0; idx<CAN_CHANNELS_MAX; idx++)
  {
    for (cbIdx=0; cbIdx<CAN_TRANSMITTED_CALLBACKS_MAX; cbIdx++)
    {
      canCallbackGenericTransmitted[idx][cbIdx] = NULL;
    }
    for (cbIdx=0; cbIdx<CAN_RECEIVED_CALLBACKS_MAX; cbIdx++)
    {
      canCallbackGenericReceived[idx][cbIdx] = NULL;
    }
    for (cbIdx=0; cbIdx<CAN_ERROR_CALLBACKS_MAX; cbIdx++)
    {
      canCallbackError[idx][cbIdx] = NULL;
    }
    canConnected[idx] = FALSE;
  }
  /* store the configured queue sizes */
  canTransmitQsize = transmitBufferSize;
  canEventQsize = eventBufferSize;
  /* overwrite them with the minimal sizes to automatically fix a misconfiguration */
  if (transmitBufferSize < CAN_TRANSMIT_QUEUE_SIZE_MIN)
  {
    canTransmitQsize = CAN_TRANSMIT_QUEUE_SIZE_MIN;
  }
  if (eventBufferSize < CAN_EVENT_QUEUE_SIZE_MIN)
  {
    canEventQsize = CAN_EVENT_QUEUE_SIZE_MIN;
  }
  /* create the can event signaling queue */
  canEventPendingQ = xQueueCreate(canEventQsize, sizeof(tCanEventInfo));
  vQueueAddToRegistry(canEventPendingQ, "CanEvPendingQ");
  if (!(canEventPendingQ != NULL))
  {
    ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_INVALID, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* create the can transmit message queue */
  canTransmitQ = xQueueCreate(canTransmitQsize, sizeof(tCanMessage));
  vQueueAddToRegistry(canTransmitQ, "CanTransmitQ");
  if (!(canTransmitQ != NULL))
  {
    ErrCodesSetError(ER_CODE_CAN_TRANSMIT_QUEUE_INVALID, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* create can event handler task */
  result = xTaskCreate(CanEventTask, "CanEventTask", \
                       CAN_EVENT_TASK_STACK_SIZE, NULL, CAN_EVENT_TASK_PRIO, \
                       &canEventTaskHandle);
  if (!(result == pdPASS))
  {
    ErrCodesSetError(ER_CODE_CAN_EVENT_TASK_INVALID, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* create can transmission task */
  result = xTaskCreate(CanTransmitTask, "CanTransmitTask", \
                       CAN_TRANSMIT_TASK_STACK_SIZE, NULL, CAN_TRANSMIT_TASK_PRIO, \
                       &canTransmitTaskHandle);
  if (!(result == pdPASS))
  {
    ErrCodesSetError(ER_CODE_CAN_TRANSMIT_TASK_INVALID, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* place tasks in suspended mode */
  vTaskSuspend(canEventTaskHandle);
  vTaskSuspend(canTransmitTaskHandle);
} /*** end of CanInit ***/


/************************************************************************************//**
** \brief     Initializes the CAN controller and synchronizes with the CAN bus.
**            Prior to calling this function, the CAN driver must be initialized
**            (CanInit) and the SPI driver must be initialized.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \param     baud The desired CAN communication speed in bits per second
** \param     filter pointer to acceptance filter configuration information.
** \return    TRUE is successful, FALSE otherwise.
**
****************************************************************************************/
uint8_t CanConnect(uint8_t channel, uint32_t baud, tCanFilter *filter)
{
  CAN_InitTypeDef       canInit;
  CAN_FilterInitTypeDef canFilterInit;
  uint16_t              prescaler;
  tCanBitTiming         bitTiming;
  uint8_t               canFilterOffset = 0;
  GPIO_InitTypeDef      GPIO_InitStructure;
  uint32_t              saved_cs_state;

  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* note that CAN2 piggy backs on CAN1, so for CAN2 the CAN1 clock must also be
   * enabled and a filter offset should be configured. the first 14 filters are for CAN1
   * and the other 14 for CAN2.
   */
  if (channel == 1)
  {
    /* enable CAN1 clock */
    RCC_APB1PeriphClockCmd(canControllerMapping[0].can_peripheral_clk, ENABLE);
    canFilterOffset = 14;
  }
  /* enable CAN clock */
  RCC_APB1PeriphClockCmd(canControllerMapping[channel].can_peripheral_clk, ENABLE);
  /* enable clocks for CAN transmitter and receiver pins */
  RCC_AHB1PeriphClockCmd(canControllerMapping[channel].can_rx_port_peripheral_clk, ENABLE);
  RCC_AHB1PeriphClockCmd(canControllerMapping[channel].can_tx_port_peripheral_clk, ENABLE);
  /* select alternate function for the CAN pins */
  GPIO_PinAFConfig(canControllerMapping[channel].can_rx_port, canControllerMapping[channel].can_rx_source, canControllerMapping[channel].gpio_af);
  GPIO_PinAFConfig(canControllerMapping[channel].can_tx_port, canControllerMapping[channel].can_tx_source, canControllerMapping[channel].gpio_af);
  /* configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = canControllerMapping[channel].can_rx_pin;
  GPIO_Init(canControllerMapping[channel].can_rx_port, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = canControllerMapping[channel].can_tx_pin;
  GPIO_Init(canControllerMapping[channel].can_tx_port, &GPIO_InitStructure);

  /* check parameter range */
  if (!( (baud >= CAN_BAUD_MIN) && (baud <= CAN_BAUD_MAX) ))
  {
    return FALSE;
  }
  if (!( (filter->mask <= 0x1fffffff) && (filter->code <= 0x1fffffff) ))
  {
    return FALSE;
  }
  if (!( (filter->mode == CAN_FILTER_MODE_STDID_ONLY) | \
             (filter->mode == CAN_FILTER_MODE_EXTID_ONLY) | \
             (filter->mode == CAN_FILTER_MODE_MIXED_ID) ))
  {
    return FALSE;
  }
  /* try and find bittiming configuration */
  if (CanGetSpeedConfig(baud, &prescaler, &bitTiming) == FALSE)
  {
    return FALSE;
  }
  /* obtain mutual exclusive access to the CAN controller */
  saved_cs_state = OsEnterCriticalSection();
  /* CAN register init */
  CAN_DeInit(canControllerMapping[channel].can_peripheral);
  CAN_StructInit(&canInit);
  /* CAN cell init */
  canInit.CAN_TTCM = DISABLE;
  canInit.CAN_ABOM = DISABLE;
  canInit.CAN_AWUM = DISABLE;
  canInit.CAN_NART = DISABLE;
  canInit.CAN_RFLM = DISABLE;
  /* configure transmit buffers to work as a FIFO */
  canInit.CAN_TXFP = ENABLE;
  canInit.CAN_Mode = CAN_Mode_Normal;
  /* CAN baudrate configuration */
  canInit.CAN_SJW = CAN_SJW_1tq;
  canInit.CAN_BS1 = (bitTiming.prop+bitTiming.phase1)-1;
  canInit.CAN_BS2 = bitTiming.phase2-1;
  canInit.CAN_Prescaler = prescaler;
  CAN_Init(canControllerMapping[channel].can_peripheral, &canInit);
  /* CAN filter init */
  switch (filter->mode)
  {
    /* receive only 11-bit message identifiers. configure both rx buffer 0 and 1 for
     * the same filter.
     */
    case CAN_FILTER_MODE_STDID_ONLY:
      canFilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
      canFilterInit.CAN_FilterScale = CAN_FilterScale_16bit;
      /* convert 11-bit id code to the value the register expects */
      canFilterInit.CAN_FilterIdHigh = filter->code << 5;
      canFilterInit.CAN_FilterIdLow = filter->code << 5;
      /* convert 11-bit id mask to the value the register expects */
      canFilterInit.CAN_FilterMaskIdHigh = filter->mask << 5;
      canFilterInit.CAN_FilterMaskIdLow = filter->mask << 5;
      /* set IDE bit in the id mask registers to make sure only 11-bit STD id's are
       * received.
       */
      canFilterInit.CAN_FilterMaskIdHigh |= 0x08;
      canFilterInit.CAN_FilterMaskIdLow |= 0x08;
      canFilterInit.CAN_FilterActivation = ENABLE;
      canFilterInit.CAN_FilterNumber = 0 + canFilterOffset;
      canFilterInit.CAN_FilterFIFOAssignment = 0;
      CAN_FilterInit(&canFilterInit);
      canFilterInit.CAN_FilterNumber = 1 + canFilterOffset;
      canFilterInit.CAN_FilterFIFOAssignment = 1;
      CAN_FilterInit(&canFilterInit);
      break;
    /* receive only 29-bit message identifiers. configure both rx buffer 0 and 1 for
     * the same filter.
     */
    case CAN_FILTER_MODE_EXTID_ONLY:
      canFilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
      canFilterInit.CAN_FilterScale = CAN_FilterScale_32bit;
      /* convert 29-bit id code to the value the register expects */
      canFilterInit.CAN_FilterIdHigh = (uint16_t)((filter->code << 3) >> 16);
      canFilterInit.CAN_FilterIdLow = (uint16_t)(filter->code << 3);
      /* set IDE bit in the id code registers to make sure to configure the id code
       * as a 29-bit.
       */
      canFilterInit.CAN_FilterIdLow |= 0x04;
      /* convert 29-bit id mask to the value the register expects */
      canFilterInit.CAN_FilterMaskIdHigh = (uint16_t)((filter->mask << 3) >> 16);
      canFilterInit.CAN_FilterMaskIdLow = (uint16_t)(filter->mask << 3);
      /* set IDE bit in the id mask registers to make sure only 29-bit STD id's are
       * received.
       */
      canFilterInit.CAN_FilterMaskIdLow |= 0x04;
      canFilterInit.CAN_FilterActivation = ENABLE;
      canFilterInit.CAN_FilterNumber = 0 + canFilterOffset;
      canFilterInit.CAN_FilterFIFOAssignment = 0;
      CAN_FilterInit(&canFilterInit);
      canFilterInit.CAN_FilterNumber = 1 + canFilterOffset;
      canFilterInit.CAN_FilterFIFOAssignment = 1;
      CAN_FilterInit(&canFilterInit);
      break;
    /* receive both 11-bit and 29-bit CAN identifiers. in this case rx buffer 0 is
     * dedicated to the 11-bit message identifiers and rx buffer 1 for the 29-bit
     * message identifiers.
     */
    default:
      canFilterInit.CAN_FilterMode = CAN_FilterMode_IdMask;
      canFilterInit.CAN_FilterScale = CAN_FilterScale_16bit;
      /* convert 11-bit id code to the value the register expects */
      canFilterInit.CAN_FilterIdHigh = filter->code << 5;
      canFilterInit.CAN_FilterIdLow = filter->code << 5;
      /* convert 11-bit id mask to the value the register expects */
      canFilterInit.CAN_FilterMaskIdHigh = filter->mask << 5;
      canFilterInit.CAN_FilterMaskIdLow = filter->mask << 5;
      /* set IDE bit in the id mask registers to make sure only 11-bit STD id's are
       * received.
       */
      canFilterInit.CAN_FilterMaskIdHigh |= 0x08;
      canFilterInit.CAN_FilterMaskIdLow |= 0x08;
      canFilterInit.CAN_FilterActivation = ENABLE;
      canFilterInit.CAN_FilterNumber = 0 + canFilterOffset;
      canFilterInit.CAN_FilterFIFOAssignment = 0;
      CAN_FilterInit(&canFilterInit);
      canFilterInit.CAN_FilterScale = CAN_FilterScale_32bit;
      /* convert 29-bit id code to the value the register expects */
      canFilterInit.CAN_FilterIdHigh = (uint16_t)((filter->code << 3) >> 16);
      canFilterInit.CAN_FilterIdLow = (uint16_t)(filter->code << 3);
      /* set IDE bit in the id code registers to make sure to configure the id code
       * as a 29-bit.
       */
      canFilterInit.CAN_FilterIdLow |= 0x04;
      /* convert 29-bit id mask to the value the register expects */
      canFilterInit.CAN_FilterMaskIdHigh = (uint16_t)((filter->mask << 3) >> 16);
      canFilterInit.CAN_FilterMaskIdLow = (uint16_t)(filter->mask << 3);
      /* set IDE bit in the id mask registers to make sure only 29-bit STD id's are
       * received.
       */
      canFilterInit.CAN_FilterMaskIdLow |= 0x04;
      canFilterInit.CAN_FilterNumber = 1 + canFilterOffset;
      canFilterInit.CAN_FilterFIFOAssignment = 1;
      CAN_FilterInit(&canFilterInit);
      break;
  }
  /* enable CAN interrupts */
  CanConfigureInterrupt(channel, TRUE);
  /* set to connected */
  canConnected[channel] = TRUE;
  /* store the start time for timestamping of received messages */
  canStartTimeMs[channel] = (xTaskGetTickCount()*portTICK_PERIOD_US)/1000;
  /* release mutual exclusive access */
  OsLeaveCriticalSection(saved_cs_state);
  /* place tasks in resumed mode */
  vTaskResume(canEventTaskHandle);
  vTaskResume(canTransmitTaskHandle);
  /* still here so all is okay */
  return TRUE;
} /*** end of CanConnect ***/


/************************************************************************************//**
** \brief     Releases the CAN driver by disabling the CAN related interrupts and
**            placing the CAN controller in its reset state, which takes it off the
**            CAN bus. To continue using the CAN driver again, a call to CanInit is
**            required.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    none.
**
****************************************************************************************/
void CanDisconnect(uint8_t channel)
{
  uint32_t saved_cs_state;

  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* there could still be CAN messages placed in the hardware transmit buffers so delay
   * a little here to give the CAN controller a chance to finish there before placing
   * the CAN controller back into its reset state. otherwise the transmit is canceled,
   * possibly resulting in an error frame on the bus.
   */
  vTaskDelay(CAN_MSG_TRANSMIT_MAX_TIME * CAN_NUM_TX_BUFFERS);
  /* place tasks in suspended mode */
  vTaskSuspend(canTransmitTaskHandle);
  vTaskSuspend(canEventTaskHandle);
  /* obtain mutual exclusive access to the CAN controller */
  saved_cs_state = OsEnterCriticalSection();
  /* set to disconnected */
  canConnected[channel] = FALSE;
  /* disable CAN interrupts */
  CanConfigureInterrupt(channel, FALSE);
  /* place CAN controller back into its reset state */
  CAN_DeInit(canControllerMapping[channel].can_peripheral);
  /* release mutual exclusive access */
  OsLeaveCriticalSection(saved_cs_state);
} /*** end of CanDisconnect ***/


/************************************************************************************//**
** \brief     Determines if the CAN driver is connected to the CAN bus.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    TRUE is connected, FALSE otherwise.
**
****************************************************************************************/
uint8_t CanIsConnected(uint8_t channel)
{
  /* make sure the channel is valid before using it as an array indexer */
  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* return the connection state */
  return canConnected[channel];
} /*** end of CanIsConnected ***/


/************************************************************************************//**
** \brief     Submits a CAN message for transmission.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \param     id   message identifier
** \param     dlc  number of data bytes in the message
** \param     data value of the data bytes
** \return    TRUE is successful, FALSE otherwise.
**
****************************************************************************************/
uint8_t CanTransmit(uint8_t channel, uint32_t id, uint8_t dlc, uint8_t *data)
{
  signed portBASE_TYPE xHigherPrioTaskWokenDummy;
  unsigned portBASE_TYPE numMsgInQueue;
  tCanMessage txMsg;
  uint8_t     txDataCnt;
  
  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* store the message in the structure */
  txMsg.channel = channel;
  txMsg.id = id;
  txMsg.dlc = dlc;
  for (txDataCnt=0; txDataCnt<dlc; txDataCnt++)
  {
    txMsg.data[txDataCnt] = data[txDataCnt];
  }

  /* add the message to the queue to have it sent out by the transmit task */
  /* make sure there is always 1 entry left in the queue after adding this transmit
   * message. CanTransmitTask reads a msg from the queue but must be able to place it
   * back in the queue if the hardware transmit slots are busy. this function can
   * interrupt CanTransmitTask can otherwise steal away the last slot.
   */
  numMsgInQueue = uxQueueMessagesWaitingFromISR(canTransmitQ);
  if ((canTransmitQsize - numMsgInQueue) <= 1)
  {
    /* transmit queue is full. only flag it as an error if we know this is not the only
     * node on the bus. if we are the only node on the bus, the transmit error counter
     * will be > 0.
     */
    if (CanGetErrorCount(channel, CAN_ERR_COUNT_TRANSMISSION) == 0)
    {
      ErrCodesSetError(ER_CODE_CAN_TRANSMIT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    return FALSE;
  }
  /* place them messag in the transmit queue. this will place the transmit task in a
   * ready-to-run state and a potential context switch happens on the next timer tick.
   * not that the XxxFromISR functions are used to make it interrupt safe.
   */
  if (xQueueSendToBackFromISR(canTransmitQ, &txMsg, &xHigherPrioTaskWokenDummy) == errQUEUE_FULL)
  {
    /* should not happen because we just checked if spots are available */
    ErrCodesSetError(ER_CODE_CAN_TRANSMIT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }
  /* still here so all is okay */
  return TRUE;
} /*** end of CanTransmit ***/


/************************************************************************************//**
** \brief     Cancels the transmission of all transmission that are scheduled or in
**            progress.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    none.
**
****************************************************************************************/
void CanCancelTransmit(uint8_t channel)
{
  uint8_t txBufIdx;

  /* only access CAN controller when connected */
  if (CanIsConnected(channel) == TRUE)
  {
    for (txBufIdx=0; txBufIdx<CAN_NUM_TX_BUFFERS; txBufIdx++)
    {
      CAN_CancelTransmit(canControllerMapping[channel].can_peripheral, txBufIdx);
    }
  }
} /*** end of CanCancelTransmit ***/


/************************************************************************************//**
** \brief     Reads out the counter value of the specified error counter.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \param     counter CAN_ERR_COUNT_RECEPTION or CAN_ERR_COUNT_TRANSMISSION.
** \return    error counter value
**
****************************************************************************************/
uint8_t CanGetErrorCount(uint8_t channel, tCanErrCounter counter)
{
  uint8_t  errCounterValue;
  uint32_t saved_cs_state;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* check the parameter */
  if (!( (counter == CAN_ERR_COUNT_RECEPTION) || (counter == CAN_ERR_COUNT_TRANSMISSION) ))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_COUNTER, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* only access CAN error counter registers when bus is connected */
  if (CanIsConnected(channel) == FALSE)
  {
    return 0;
  }
  /* obtain mutual exclusive access to the CAN controller */
  saved_cs_state = OsEnterCriticalSection();
  /* read out the correct error counter */
  if (counter == CAN_ERR_COUNT_RECEPTION)
  {
    errCounterValue = CAN_GetReceiveErrorCounter(canControllerMapping[channel].can_peripheral);
  }
  else
  {
    errCounterValue = CAN_GetLSBTransmitErrorCounter(canControllerMapping[channel].can_peripheral);
  }
  /* release mutual exclusive access */
  OsLeaveCriticalSection(saved_cs_state);
  /* return the results */
  return errCounterValue;
} /*** end of CanGetErrorCount ***/


/************************************************************************************//**
** \brief     Registers the callback handler that gets called by the CAN driver each time
**            a CAN message was successfully transmitted.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    none.
**
****************************************************************************************/
void CanRegisterGenericTransmittedCallback(uint8_t channel, tCanCallbackGenericTransmitted callbackPtr)
{
  uint8_t cbIdx;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* find next free slot */
  for (cbIdx=0; cbIdx<CAN_TRANSMITTED_CALLBACKS_MAX; cbIdx++)
  {
    if (canCallbackGenericTransmitted[channel][cbIdx] == NULL)
    {
      canCallbackGenericTransmitted[channel][cbIdx] = callbackPtr;
      break;
    }
  }
} /*** end of CanRegisterGenericTransmittedCallback ***/


/************************************************************************************//**
** \brief     Registers the callback handler that gets called by the CAN driver each time
**            a CAN message was successfully received.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    none.
**
****************************************************************************************/
void CanRegisterGenericReceivedCallback(uint8_t channel, tCanCallbackGenericReceived callbackPtr)
{
  uint8_t cbIdx;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* find next free slot */
  for (cbIdx=0; cbIdx<CAN_RECEIVED_CALLBACKS_MAX; cbIdx++)
  {
    if (canCallbackGenericReceived[channel][cbIdx] == NULL)
    {
      canCallbackGenericReceived[channel][cbIdx] = callbackPtr;
      break;
    }
  }
} /*** end of CanRegisterGenericReceivedCallback ***/


/************************************************************************************//**
** \brief     Registers the callback handler that gets called by the CAN driver each time
**            an error was detected such as bus off or hardware message reception slot
**            overrun.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    none.
**
****************************************************************************************/
void CanRegisterErrorCallback(uint8_t channel, tCanCallbackError callbackPtr)
{
  uint8_t cbIdx;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* find next free slot */
  for (cbIdx=0; cbIdx<CAN_ERROR_CALLBACKS_MAX; cbIdx++)
  {
    if (canCallbackError[channel][cbIdx] == NULL)
    {
      canCallbackError[channel][cbIdx] = callbackPtr;
      break;
    }
  }
} /*** end of CanRegisterErrorCallback ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a new
**            message was received.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    pdTRUE is a higher priority task was woken, pdFALSE otherwise.
**
****************************************************************************************/
static portBASE_TYPE CanRx0Interrupt(uint8_t channel)
{
  tCanEventInfo eventInfo;
  portBASE_TYPE xHigherPrioTaskWoken = pdFALSE;
  portBASE_TYPE xHigherPrioTaskWokenMsg = pdFALSE;
  portBASE_TYPE result;
  uint8_t       msgCnt;
  uint8_t       byteCnt;
  CanRxMsg      rxMsg;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* prepare the event channel */
  eventInfo.channel = channel;

  /* was a message received in the FIFO? */
  if (CAN_GetITStatus(canControllerMapping[channel].can_peripheral, CAN_IT_FMP0) == SET)
  {
    /* prepare the event information */
    eventInfo.type = CAN_EVENT_MSG_RXED;
    /* read out how many messages are in the FIFO */
    msgCnt = CAN_MessagePending(canControllerMapping[channel].can_peripheral, CAN_FIFO0);
    /* empty out the FIFO */
    while (msgCnt > 0)
    {
      CAN_Receive(canControllerMapping[channel].can_peripheral, CAN_FIFO0, &rxMsg);
      eventInfo.msg.timestamp = ((xTaskGetTickCount()*portTICK_PERIOD_US)/1000) - canStartTimeMs[channel];
      eventInfo.msg.channel = channel;
      eventInfo.msg.dlc = rxMsg.DLC;
      if (rxMsg.IDE == CAN_ID_STD)
      {
        eventInfo.msg.id = rxMsg.StdId;
      }
      else
      {
        eventInfo.msg.id = rxMsg.ExtId | CAN_EXTID_MASK;
      }
      if (rxMsg.RTR == CAN_RTR_DATA)
      {
        for (byteCnt=0; byteCnt < rxMsg.DLC; byteCnt++)
        {
          eventInfo.msg.data[byteCnt] = rxMsg.Data[byteCnt];
        }
      }
      else
      {
        eventInfo.msg.id |= CAN_RTR_FRAME_MASK;
      }
      /* continue with the next one */
      msgCnt--;
    }
    /* add the event to the queue for further processing at task level */
    result = xQueueSendToBackFromISR(canEventPendingQ, &eventInfo, &xHigherPrioTaskWokenMsg);
    if (!(result != errQUEUE_FULL))
    {
      ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* did this one cause a higher priority task to be woken? */
    if (xHigherPrioTaskWokenMsg == pdTRUE)
    {
      xHigherPrioTaskWoken = pdTRUE;
    }
    /* clear the message received pending bit */
    CAN_ClearITPendingBit(canControllerMapping[channel].can_peripheral, CAN_IT_FMP0);
  }
  /* did a message overrun occur in the FIFO */
  else if (CAN_GetITStatus(canControllerMapping[channel].can_peripheral, CAN_IT_FOV0) == SET)
  {
    /* prepare the event information */
    eventInfo.type = CAN_EVENT_RX_OVERRUN;
    /* add the event to the queue for further processing at task level */
    result = xQueueSendToBackFromISR(canEventPendingQ, &eventInfo, &xHigherPrioTaskWoken);
    if (!(result != errQUEUE_FULL))
    {
      ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* clear the overrun flag */
    CAN_ClearFlag(canControllerMapping[channel].can_peripheral, CAN_FLAG_FOV0);
    /* clear the message overrun pending bit */
    CAN_ClearITPendingBit(canControllerMapping[channel].can_peripheral, CAN_IT_FOV0);
  }
  return xHigherPrioTaskWoken;
} /*** end of CanRx0Interrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a new
**            message was received.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    pdTRUE is a higher priority task was woken, pdFALSE otherwise.
**
****************************************************************************************/
static portBASE_TYPE CanRx1Interrupt(uint8_t channel)
{
  tCanEventInfo eventInfo;
  portBASE_TYPE xHigherPrioTaskWoken = pdFALSE;
  portBASE_TYPE xHigherPrioTaskWokenMsg = pdFALSE;
  portBASE_TYPE result;
  uint8_t       msgCnt;
  uint8_t       byteCnt;
  CanRxMsg      rxMsg;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* prepare the event channel */
  eventInfo.channel = channel;

  /* was a message received in the FIFO? */
  if (CAN_GetITStatus(canControllerMapping[channel].can_peripheral, CAN_IT_FMP1) == SET)
  {
    /* prepare the event information */
    eventInfo.type = CAN_EVENT_MSG_RXED;
    /* read out how many messages are in the FIFO */
    msgCnt = CAN_MessagePending(canControllerMapping[channel].can_peripheral, CAN_FIFO1);
    /* empty out the FIFO */
    while (msgCnt > 0)
    {
      CAN_Receive(canControllerMapping[channel].can_peripheral, CAN_FIFO1, &rxMsg);
      eventInfo.msg.timestamp = ((xTaskGetTickCount()*portTICK_PERIOD_US)/1000) - canStartTimeMs[channel];
      eventInfo.msg.channel = channel;
      eventInfo.msg.dlc = rxMsg.DLC;
      if (rxMsg.IDE == CAN_ID_STD)
      {
        eventInfo.msg.id = rxMsg.StdId;
      }
      else
      {
        eventInfo.msg.id = rxMsg.ExtId | CAN_EXTID_MASK;
      }
      if (rxMsg.RTR == CAN_RTR_DATA)
      {
        for (byteCnt=0; byteCnt < rxMsg.DLC; byteCnt++)
        {
          eventInfo.msg.data[byteCnt] = rxMsg.Data[byteCnt];
        }
      }
      else
      {
        eventInfo.msg.id |= CAN_RTR_FRAME_MASK;
      }
      /* continue with the next one */
      msgCnt--;
    }
    /* add the event to the queue for further processing at task level */
    result = xQueueSendToBackFromISR(canEventPendingQ, &eventInfo, &xHigherPrioTaskWokenMsg);
    if (!(result != errQUEUE_FULL))
    {
      ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* did this one cause a higher priority task to be woken? */
    if (xHigherPrioTaskWokenMsg == pdTRUE)
    {
      xHigherPrioTaskWoken = pdTRUE;
    }
    /* clear the message received pending bit */
    CAN_ClearITPendingBit(canControllerMapping[channel].can_peripheral, CAN_IT_FMP1);
  }
  /* did a message overrun occur in the FIFO */
  else if (CAN_GetITStatus(canControllerMapping[channel].can_peripheral, CAN_IT_FOV1) == SET)
  {
    /* prepare the event information */
    eventInfo.type = CAN_EVENT_RX_OVERRUN;
    /* add the event to the queue for further processing at task level */
    result = xQueueSendToBackFromISR(canEventPendingQ, &eventInfo, &xHigherPrioTaskWoken);
    if (!(result != errQUEUE_FULL))
    {
      ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* clear the overrun flag */
    CAN_ClearFlag(canControllerMapping[channel].can_peripheral, CAN_FLAG_FOV1);
    /* clear the message overrun pending bit */
    CAN_ClearITPendingBit(canControllerMapping[channel].can_peripheral, CAN_IT_FOV1);
  }
  return xHigherPrioTaskWoken;
} /*** end of CanRx1Interrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a CAN
**            transmit mailbox gets empty.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    pdTRUE is a higher priority task was woken, pdFALSE otherwise.
**
****************************************************************************************/
static portBASE_TYPE CanTxInterrupt(uint8_t channel)
{
  uint8_t       txBufNr = 0;
  tCanEventInfo eventInfo;
  portBASE_TYPE xHigherPrioTaskWoken = pdFALSE;
  portBASE_TYPE result;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* confirm that this was the transmit mailbox empty interrupt */
  if (CAN_GetITStatus(canControllerMapping[channel].can_peripheral, CAN_IT_TME) == SET)
  {
    /* determine the mailbox that caused the interrupt */
    if (CAN_GetFlagStatus(canControllerMapping[channel].can_peripheral, CAN_FLAG_RQCP0) == SET)
    {
      /* clear the flag and set the buffer number */
      CAN_ClearFlag(canControllerMapping[channel].can_peripheral, CAN_FLAG_RQCP0);
      txBufNr = 0;
    }
    else if (CAN_GetFlagStatus(canControllerMapping[channel].can_peripheral, CAN_FLAG_RQCP1) == SET)
    {
      /* clear the flag and set the buffer number */
      CAN_ClearFlag(canControllerMapping[channel].can_peripheral, CAN_FLAG_RQCP1);
      txBufNr = 1;
    }
    else if (CAN_GetFlagStatus(canControllerMapping[channel].can_peripheral, CAN_FLAG_RQCP2) == SET)
    {
      /* clear the flag and set the buffer number */
      CAN_ClearFlag(canControllerMapping[channel].can_peripheral, CAN_FLAG_RQCP2);
      txBufNr = 2;
    }
    else
    {
      /* no transmit mailbox caused the interrupt. should not happen */
      configASSERT(FALSE);
    }
    /* prepare the event information */
    eventInfo.channel = channel;
    eventInfo.type = CAN_EVENT_MSG_TXED;
    eventInfo.msg.channel = channel;
    eventInfo.msg.id = canTxPendingId[channel][txBufNr];
    /* add the event to the queue for further processing at task level */
    result = xQueueSendToBackFromISR(canEventPendingQ, &eventInfo, &xHigherPrioTaskWoken);
    if (!(result != errQUEUE_FULL))
    {
      ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* clear the transmit mailbox empty pending bit */
    CAN_ClearITPendingBit(canControllerMapping[channel].can_peripheral, CAN_IT_TME);
  }
  return xHigherPrioTaskWoken;
} /*** end of CanTxInterrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a bus off
**            or reception overrun event occurred.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \return    pdTRUE is a higher priority task was woken, pdFALSE otherwise.
**
****************************************************************************************/
static portBASE_TYPE CanErrorInterrupt(uint8_t channel)
{
  tCanEventInfo eventInfo;
  portBASE_TYPE xHigherPrioTaskWoken = pdFALSE;
  portBASE_TYPE result;

  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }


  /* confirm that this was the bus off interrupt */
  if (CAN_GetITStatus(canControllerMapping[channel].can_peripheral, CAN_IT_BOF) == SET)
  {
    /* prepare the event information */
    eventInfo.channel = channel;
    eventInfo.type = CAN_EVENT_BUS_OFF;
    /* add the event to the queue for further processing at task level */
    result = xQueueSendToBackFromISR(canEventPendingQ, &eventInfo, &xHigherPrioTaskWoken);
    if (!(result != errQUEUE_FULL))
    {
      ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }
    /* clear the transmit mailbox empty pending bit */
    CAN_ClearITPendingBit(canControllerMapping[channel].can_peripheral, CAN_IT_BOF);
  }
  return xHigherPrioTaskWoken;
} /*** end of CanErrorInterrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a new
**            message was received.
** \return    none.
**
****************************************************************************************/
void Can1Rx0Interrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;
  
  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanRx0Interrupt(0);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can1Rx0Interrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a new
**            message was received.
** \return    none.
**
****************************************************************************************/
void Can1Rx1Interrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;
  
  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanRx1Interrupt(0);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can1Rx1Interrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a CAN
**            transmit mailbox gets empty.
** \return    none.
**
****************************************************************************************/
void Can1TxInterrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanTxInterrupt(0);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can1TxInterrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a bus off
**            or reception overrun event occured.
** \return    none.
**
****************************************************************************************/
void Can1ErrorInterrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanErrorInterrupt(0);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can1ErrorInterrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a new
**            message was received.
** \return    none.
**
****************************************************************************************/
void Can2Rx0Interrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanRx0Interrupt(1);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can2Rx0Interrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a new
**            message was received.
** \return    none.
**
****************************************************************************************/
void Can2Rx1Interrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanRx1Interrupt(1);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can2Rx1Interrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a CAN
**            transmit mailbox gets empty.
** \return    none.
**
****************************************************************************************/
void Can2TxInterrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanTxInterrupt(1);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can2TxInterrupt ***/


/************************************************************************************//**
** \brief     CAN Interrupt Service Routine that gets triggered each time a bus off
**            or reception overrun event occured.
** \return    none.
**
****************************************************************************************/
void Can2ErrorInterrupt(void)
{
  portBASE_TYPE xHigherPrioTaskWoken;

  /* inform the OS about the ISR start */
  OsIrqEnter();
  /* process interrupt */
  xHigherPrioTaskWoken = CanErrorInterrupt(1);
  /* inform the OS about the ISR end */
  OsIrqExit();
  /* should the context be switched so the ISR returns to a different task? */
  portEND_SWITCHING_ISR(xHigherPrioTaskWoken);
} /*** end of Can2ErrorInterrupt ***/


/************************************************************************************//**
** \brief     Allocates a free hardware transmit slot and, if one found, copies the
**            CAN message to it and starts the actual transmission of the CAN  message.
** \param     msg pointer to CAN message that is to be transmitted.
** \return    TRUE is successful, FALSE otherwise.
**
****************************************************************************************/
static uint8_t CanCopyAndStartTransmit(tCanMessage *msg)
{
  uint8_t  result = FALSE;
  uint8_t  txUsedBufNr;
  uint8_t  byteCnt;
  CanTxMsg txMsg;
  uint32_t orgId;
  uint32_t saved_cs_state;
  
  /* check channel validity */
  if (!(msg->channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    return result;
  }

  /* verify parameters */
  if ( (msg->dlc > 8) || (msg->data == NULL) )
  {
    return result;
  }
  /* only transmit message if the CAN bus is connected */
  if (CanIsConnected(msg->channel) == FALSE)
  {
    return result;
  }
  /* store original id with RTR and EXTID mask bits */
  orgId = msg->id;
  /* store the data length */
  txMsg.DLC = msg->dlc;
  /* process the frame time */
  if ((msg->id & CAN_RTR_FRAME_MASK) == 0)
  {
    /* data frame */
    txMsg.RTR = CAN_RTR_DATA;
    for (byteCnt=0; byteCnt<msg->dlc; byteCnt++)
    {
      txMsg.Data[byteCnt] = msg->data[byteCnt];
    }
  }
  else
  {
    /* remote frame without data */
    msg->id &= ~CAN_RTR_FRAME_MASK;
    txMsg.RTR = CAN_RTR_REMOTE;
  }
  /* store the identifier */  
  if ((msg->id & CAN_EXTID_MASK) == 0)
  {
    /* standard 11-bit message identifier */
    txMsg.IDE = CAN_ID_STD;
    txMsg.StdId = msg->id;
    txMsg.ExtId = 0;
  }
  else
  {
    /* extended 29-bit message identifier */
    msg->id &= ~CAN_EXTID_MASK;
    txMsg.IDE = CAN_ID_EXT;
    txMsg.ExtId = msg->id;
    txMsg.StdId = 0;
  }
  /* restore the original id now that parsing is done */
  msg->id = orgId;
  /* start the transmission */
  saved_cs_state = OsEnterCriticalSection();
  txUsedBufNr = CAN_Transmit(canControllerMapping[msg->channel].can_peripheral, &txMsg);
  /* check that an available transmit buffer was found */
  if (txUsedBufNr != CAN_TxStatus_NoMailBox)
  {
    /* store the message identifier so that upon transmit complete, we know which
     * message was sent. 
     */
    canTxPendingId[msg->channel][txUsedBufNr] = orgId;
    result = TRUE;
  }
  OsLeaveCriticalSection(saved_cs_state);
  /* still here so all is okay */
  return result;
} /*** end of CanCopyAndStartTransmit ***/


/************************************************************************************//**
** \brief     CAN transmission task.
** \param     pvParameters pointer to task parameters
** \return    none.
**
****************************************************************************************/
static void CanTransmitTask(void *pvParameters)
{
  tCanMessage   canMsg;
  portBASE_TYPE result;

  for (;;)
  {
    /* wait for a new message to be added to the transmit queue */
    result = xQueueReceive(canTransmitQ, &canMsg, portMAX_DELAY);
    /* it is unexpected that this fails so trigger an assertion if it does */
    if (!(result == pdTRUE))
    {
      ErrCodesSetError(ER_CODE_CAN_TRANSMIT_QUEUE_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }

    /* start the actual CAN message transmission */
    if (CanCopyAndStartTransmit(&canMsg) == FALSE)
    {
      /* the CAN driver has 3 transmit message slots, so it is unexpected that it cannot
       * transmit under normal situations, unless this is the only node on the bus. in
       * that case, place the message back into the front of the queue to keep on
       * retrying for when the node does come back online. another possibility is when
       * the host submits more messages for transmission then the CAN bus can handle.
       */
      if (xQueueSendToFront(canTransmitQ, &canMsg, 0) == errQUEUE_FULL)
      {
        /* this should not fail because we just read from the queue so there should be
         * space to place it back into the queue.
         */
        ErrCodesSetError(ER_CODE_CAN_TRANSMIT_QUEUE_FULL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
      }
    }
  }
} /*** end of CanTransmitTask ***/


/************************************************************************************//**
** \brief     CAN event handler task.
** \param     pvParameters pointer to task parameters
** \return    none.
**
****************************************************************************************/
static void CanEventTask(void *pvParameters)
{
  tCanEventInfo eventInfo;
  portBASE_TYPE result;
  uint8_t cbIdx;
  
  for (;;)
  {
    /* wait until we have something to do */
    result = xQueueReceive(canEventPendingQ, &eventInfo, portMAX_DELAY);
    /* it is unexpected that this fails so trigger an assertion if it does */
    if (!(result == pdTRUE))
    {
      ErrCodesSetError(ER_CODE_CAN_EVENT_QUEUE_ERROR, ER_PARAM_SEVERITY_CRITICAL, TRUE);
    }

    /* process the event, but only when the CAN driver is in the connected state */
    if (CanIsConnected(eventInfo.channel) == TRUE)
    {
      /* find out which event needs processing */
      switch(eventInfo.type)
      {
        /*----------- process message transmit complete interrupt ---------------------*/
        case CAN_EVENT_MSG_TXED: 
          /* invoke callbacks to pass on message transmit complete event */
          for (cbIdx=0; cbIdx<CAN_TRANSMITTED_CALLBACKS_MAX; cbIdx++)
          {
            if (canCallbackGenericTransmitted[eventInfo.channel][cbIdx] != NULL)
            {
              canCallbackGenericTransmitted[eventInfo.channel][cbIdx](eventInfo.msg.id);
            }
          }
          break;
        /*----------- process new message reception interrupt -------------------------*/
        case CAN_EVENT_MSG_RXED: 
          /* invoke callback functions to pass on message reception event */
          for (cbIdx=0; cbIdx<CAN_RECEIVED_CALLBACKS_MAX; cbIdx++)
          {
            if (canCallbackGenericReceived[eventInfo.channel][cbIdx] != NULL)
            {
              canCallbackGenericReceived[eventInfo.channel][cbIdx](eventInfo.msg.id, eventInfo.msg.dlc,
                                                                   eventInfo.msg.data, eventInfo.msg.timestamp);
            }
          }
          break;
        /*----------- process can bus off related interrupt ---------------------------*/
        case CAN_EVENT_RX_OVERRUN: 
          /* invoke callback functions to pass on error event */
          for (cbIdx=0; cbIdx<CAN_ERROR_CALLBACKS_MAX; cbIdx++)
          {
            if (canCallbackError[eventInfo.channel][cbIdx] != NULL)
            {
              canCallbackError[eventInfo.channel][cbIdx](CAN_ERR_STAT_RX_OVERFLOW);
            }
          }


          break;
        /*----------- process can bus off related interrupt ---------------------------*/
        case CAN_EVENT_BUS_OFF: 
          /* invoke callback functions to pass on error event */
          for (cbIdx=0; cbIdx<CAN_ERROR_CALLBACKS_MAX; cbIdx++)
          {
            if (canCallbackError[eventInfo.channel][cbIdx] != NULL)
            {
              canCallbackError[eventInfo.channel][cbIdx](CAN_ERR_STAT_BUS_OFF);
            }
          }

          break;
        /*----------- unexpected event ------------------------------------------------*/
        default:
          ErrCodesSetError(ER_CODE_CAN_UNEXPECTED_EVENT, ER_PARAM_SEVERITY_CRITICAL, TRUE);
          break;
      }
    }
  }
} /*** end of CanEventTask ***/


/************************************************************************************//**
** \brief     Search algorithm to match the desired baudrate to a possible bus
**            timing configuration.
** \param     baud The desired baudrate in bps..
** \param     prescaler Pointer to where the value for the prescaler will be stored.
** \param     bitTiming Pointer to struct where bittiming values will be stored.
** \return    TRUE if valid CAN bustiming register values were found, FALSE otherwise.
**
****************************************************************************************/
static uint8_t CanGetSpeedConfig(uint32_t baud, uint16_t *prescaler, tCanBitTiming *bitTiming)
{
  int8_t  cnt;
  uint8_t  tqSetting;
  uint32_t baudActual;
  int8_t   deviationPct;

  /* loop through all possible time quanta configurations to find a match */
  for (cnt=(sizeof(canTiming)/sizeof(canTiming[0]))-1; cnt>=0; cnt--)
  {
    /* determine how many timequanta in one bit for this configuration */
    tqSetting = (1+canTiming[cnt].prop+canTiming[cnt].phase1+canTiming[cnt].phase2);
    /* compute the prescaler that goes with this TQ configuration */
    *prescaler = CAN_CLOCK_SPEED_HZ / (baud * tqSetting);
    /* make sure the prescaler is valid */
    if (!( (*prescaler >= CAN_PRESCALER_MIN) && (*prescaler <= CAN_PRESCALER_MAX) ))
    {
      /* no valid prescaler available for this configuration, so try the next one */
      continue;
    }
    /* determine the actual baudrate this setting would result in */
    baudActual = CAN_CLOCK_SPEED_HZ / (*prescaler * tqSetting);
    /* determine deviation from requested baudrate */
    deviationPct = ((int32_t)(baudActual - baud) * 100) / baud;
    /* get the absolute value because it could be a negative percentage */
    deviationPct *= (deviationPct < 0) ? -1 : 1; 
    /* check if this is within the accepted baudrate tolerance */
    if (deviationPct <= CAN_BAUD_TOLERANCE_PCT)
    {
      /* store the bittiming configuration */
      bitTiming->prop   = canTiming[cnt].prop;
      bitTiming->phase1 = canTiming[cnt].phase1;
      bitTiming->phase2 = canTiming[cnt].phase2;
      /* found a good bus timing configuration */
      return TRUE;
    }
  }
  /* could not find a good bus timing configuration */
  return FALSE;
} /*** end of CanGetSpeedConfig ***/


/************************************************************************************//**
** \brief     Configures the CAN related interrutps.
** \param     channel the CAN controller channel. should be 0..(CAN_CHANNELS_MAX-1)
** \param     enable TRUE to enable the interrupt, FALSE to disable.
** \return    none.
**
****************************************************************************************/
static void CanConfigureInterrupt(uint8_t channel, uint8_t enable)
{
  NVIC_InitTypeDef NVIC_InitStruct;

  /* check channel validity */
  if (!(channel < CAN_CHANNELS_MAX))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_CHANNEL, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  /* check the parameter */
  if (!( (enable == TRUE) || (enable == FALSE) ))
  {
    ErrCodesSetError(ER_CODE_CAN_INVALID_INTERRUPT_IRQ, ER_PARAM_SEVERITY_CRITICAL, TRUE);
  }

  if (enable == TRUE)
  {
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = CAN_NVIC_IRQ_PRIO;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* enable the CAN TX interrupt */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].tx_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
    /* enable the CAN RX interrupts */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].rx0_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
    /* enable the CAN RX interrupts */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].rx1_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
    /* enable the CAN Error interrupts */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].sce_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
    /* transmit mailbox empty interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_TME, ENABLE);
    /* reception FIFO x message pending interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FMP0, ENABLE);
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FMP1, ENABLE);
    /* reception FIFO x overrun interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FOV0, ENABLE);
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FOV1, ENABLE);
    /* bus off error interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_BOF, ENABLE);
  }
  else
  {
    /* transmit mailbox empty interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_TME, DISABLE);
    /* reception FIFO x message pending interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FMP0, DISABLE);
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FMP1, DISABLE);
    /* reception FIFO x overrun interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FOV0, DISABLE);
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_FOV1, DISABLE);
    /* bus off error interrupt */
    CAN_ITConfig(canControllerMapping[channel].can_peripheral, CAN_IT_BOF, DISABLE);
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = CAN_NVIC_IRQ_PRIO;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = DISABLE;
    /* disable the CAN TX interrupt */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].tx_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
    /* disable the CAN RX interrupts */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].rx0_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
    /* disable the CAN RX interrupts */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].rx1_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
    /* disable the CAN Error interrupts */
    NVIC_InitStruct.NVIC_IRQChannel = canControllerMapping[channel].sce_irq_channel;
    NVIC_Init(&NVIC_InitStruct);
  }
} /*** end of CanConfigureInterrupt ***/


/************************************ end of can.c *************************************/


