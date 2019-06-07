/************************************************************************************//**
* \file         vectors.c
* \brief        Interrupt vector table source file.
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
#include <stdint.h>                                   /* ANSI C types                  */
#include "can.h"                                      /* CAN driver header file        */
#include "uart.h"                                     /* UART driver header file       */
#include "timein.h"                                   /* Timer input driver            */
#include "timeout.h"                                  /* Timer input driver            */
#include "pwmout.h"                                   /* PWM driver header file        */
#include "usbcom.h"                                   /* USB COM-port header file      */


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Type definition for a vector table entry. */
typedef union
{
  void (*func)(void);                                 /* for ISR function pointers     */
  uint32_t ptr;                                       /* for stack pointer entry       */
}tIsrFunc;                                            /* type for vector table entries */


/****************************************************************************************
* External function prototypes
****************************************************************************************/
extern void Reset_Handler(void);                      /* implemented in cstart.c       */
extern void xPortPendSVHandler(void);                 /* implemented in port.c         */
extern void xPortSysTickHandler(void);                /* implemented in port.c         */
extern void vPortSVCHandler( void );                  /* implemented in port.c         */


/****************************************************************************************
* External data declarations
****************************************************************************************/
/** \brief External variable that holds the start address of the stack. The variable
 *         is declared by the linker.
 */
extern uint32_t _estack;


/************************************************************************************//**
** \brief     IRQ handler for TIM1 Update that maps it to the interrupt service
**            routines of the individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer1UpdateInterrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein1UpdateInterrupt();
  Pwmout1UpdateInterrupt();
  Timeout1UpdateInterrupt();
} /*** end of Timer1UpdateTimein10Interrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM1 Capture Compare that maps it to the interrupt service
**            routines of the individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer1CaptureInterrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein1CaptureInterrupt();
  Timeout1CaptureInterrupt();
} /*** end of Timer1CaptureInterrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM2 that maps it to the interrupt service routines of the
**            individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer2Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein2Interrupt();
  Pwmout2Interrupt();
  Timeout2Interrupt();
} /*** end of Timer2Interrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM3 that maps it to the interrupt service routines of the
**            individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer3Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein3Interrupt();
  Pwmout3Interrupt();
  Timeout3Interrupt();
} /*** end of Timer3Interrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM4 that maps it to the interrupt service routines of the
**            individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer4Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein4Interrupt();
  Pwmout4Interrupt();
  Timeout4Interrupt();
} /*** end of Timer4Interrupt ***/


/************************************************************************************//**
** \brief     Muxer for the CAN Rx0 and USB interrupt handlers. They share the same
**            vector address. The ISR handlers themselves have an added protection that
**            its code will only execute if the module is actually initialized. Note
**            that on the STM32F103 there is a limitation where CAN and USB cannot be
**            used at the same time.
** \return    none.
**
****************************************************************************************/
static void UsbCanRx0MuxedIrqHandler(void)
{
  UsbComIRQHandler();
  CanRx0Interrupt();
} /*** end of UsbCanMuxedIrqHandler ***/


/****************************************************************************************
*                I N T E R R U P T     V E C T O R     T A B L E
****************************************************************************************/
/** \brief Interrupt vector table. The .isr_vector section attribute makes sure that the
 *         linker places this table at the hardware defined region for the vector table.
 */
__attribute__ ((section(".isr_vector")))
const tIsrFunc _vectab[] =
{
  { .ptr = (unsigned long)&_estack },                 /* the initial stack pointer     */
  { Reset_Handler },                                  /* the reset handler             */
  { 0 },                                              /* NMI Handler                   */
  { 0 },                                              /* Hard Fault Handler            */
  { 0 },                                              /* MPU Fault Handler             */
  { 0 },                                              /* Bus Fault Handler             */
  { 0 },                                              /* Usage Fault Handler           */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { vPortSVCHandler },                                /* SVCall Handler                */
  { 0 },                                              /* Debug Monitor Handler         */
  { 0 },                                              /* Reserved                      */
  { xPortPendSVHandler },                             /* PendSV Handler                */
  { xPortSysTickHandler },                            /* SysTick Handler               */
  { 0 },                                              /* Window Watchdog               */
  { 0 },                                              /* PVD through EXTI Line detect  */
  { 0 },                                              /* Tamper                        */
  { 0 },                                              /* RTC                           */
  { 0 },                                              /* Flash                         */
  { 0 },                                              /* RCC                           */
  { 0 },                                              /* EXTI Line 0                   */
  { 0 },                                              /* EXTI Line 1                   */
  { 0 },                                              /* EXTI Line 2                   */
  { 0 },                                              /* EXTI Line 3                   */
  { 0 },                                              /* EXTI Line 4                   */
  { 0 },                                              /* DMA1 Channel 1                */
  { 0 },                                              /* DMA1 Channel 2                */
  { 0 },                                              /* DMA1 Channel 3                */
  { 0 },                                              /* DMA1 Channel 4                */
  { 0 },                                              /* DMA1 Channel 5                */
  { 0 },                                              /* DMA1 Channel 6                */
  { 0 },                                              /* DMA1 Channel 7                */
  { 0 },                                              /* ADC1 and ADC2                 */
  { CanTxInterrupt },                                 /* CAN1 TX                       */
  { UsbCanRx0MuxedIrqHandler },                       /* CAN1 RX0                      */
  { CanRx1Interrupt },                                /* CAN1 RX1                      */
  { CanErrorInterrupt },                              /* CAN1 SCE                      */
  { 0 },                                              /* EXTI Line 9..5                */
  { 0 },                                              /* TIM1 Break                    */
  { Timer1UpdateInterrupt },                          /* TIM1 Update                   */
  { 0 },                                              /* TIM1 Trigger and Commutation  */
  { Timer1CaptureInterrupt },                         /* TIM1 Capture Compare          */
  { Timer2Interrupt },                                /* TIM2                          */
  { Timer3Interrupt },                                /* TIM3                          */
  { Timer4Interrupt },                                /* TIM4                          */
  { 0 },                                              /* I2C1 Event                    */
  { 0 },                                              /* I2C1 Error                    */
  { 0 },                                              /* I2C2 Event                    */
  { 0 },                                              /* I2C1 Error                    */
  { 0 },                                              /* SPI1                          */
  { 0 },                                              /* SPI2                          */
  { Uart1Interrupt },                                 /* USART1                        */
  { Uart2Interrupt },                                 /* USART2                        */
  { Uart3Interrupt },                                 /* USART3                        */
  { 0 },                                              /* EXTI Line 15..10              */
  { 0 },                                              /* RTC alarm through EXTI line   */
  { 0 },                                              /* USB OTG FS Wakeup             */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* Reserved                      */
  { 0 },                                              /* TIM5                          */
  { 0 },                                              /* SPI3                          */
  { 0 },                                              /* UART4                         */
  { 0 },                                              /* UART5                         */
  { 0 },                                              /* TIM6                          */
  { 0 },                                              /* TIM7                          */
  { 0 },                                              /* DMA2 Channel1                 */
  { 0 },                                              /* DMA2 Channel2                 */
  { 0 },                                              /* DMA2 Channel3                 */
  { 0 },                                              /* DMA2 Channel4                 */
  { 0 },                                              /* DMA2 Channel5                 */
  { 0 },                                              /* Ethernet                      */
  { 0 },                                              /* Ethernet Wakeup               */
  { 0 },                                              /* CAN2 TX                       */
  { 0 },                                              /* CAN2 RX0                      */
  { 0 },                                              /* CAN2 RX1                      */
  { 0 },                                              /* CAN2 SCE                      */
  { 0 },                                              /* USB OTG FS                    */
  { (void*)0x55AA11EE }                               /* Reserved for OpenBLT checksum */
};

/************************************ end of vectors.c *********************************/
