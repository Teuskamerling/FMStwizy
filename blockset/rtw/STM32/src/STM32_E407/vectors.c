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
#include "timein.h"                                   /* Timer input driver            */
#include "timeout.h"                                  /* Timer output driver           */
#include "pwmout.h"                                   /* PWM driver header file        */
#include "uart.h"                                     /* UART driver header file       */
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
extern void EXTI3_IRQHandler(void);
extern void ETH_IRQHandler(void);
extern void SDIO_IRQHandler(void);
extern void DMA2_Stream3_IRQHandler(void);


/****************************************************************************************
* External data declarations
****************************************************************************************/
/** \brief External variable that holds the start address of the stack. The variable
 *         is declared by the linker.
 */
extern uint32_t _estack;


/************************************************************************************//**
** \brief     IRQ handler for TIM1 Update and TIM10 that maps it to the interrupt service
**            routines of the individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer1UpdateTimer10Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein1UpdateTimein10Interrupt();
  Pwmout1UpdatePwmout10Interrupt();
  Timeout1UpdateTimeout10Interrupt();
} /*** end of Timer1UpdateTimer10Interrupt ***/


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
** \brief     IRQ handler for TIM8 Update and TIM13 that maps it to the interrupt service
**            routines of the individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer8UpdateTimer13Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein8UpdateTimein13Interrupt();
  Pwmout8UpdatePwmout13Interrupt();
  Timeout8UpdateTimeout13Interrupt();
} /*** end of Timer8UpdateTimer13Interrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM8 Capture Compare that maps it to the interrupt service
**            routines of the individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer8CaptureInterrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein8CaptureInterrupt();
  Timeout8CaptureInterrupt();
} /*** end of Timer8CaptureInterrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM9 that maps it to the interrupt service routines of the
**            individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer9Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein9Interrupt();
  Pwmout9Interrupt();
  Timeout9Interrupt();
} /*** end of Timer9Interrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM11 that maps it to the interrupt service routines of the
**            individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer11Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein11Interrupt();
  Pwmout11Interrupt();
  Timeout11Interrupt();
} /*** end of Timer11Interrupt ***/


/************************************************************************************//**
** \brief     IRQ handler for TIM14 that maps it to the interrupt service routines of the
**            individual modules.
** \return    none.
**
****************************************************************************************/
static void Timer14Interrupt(void)
{
  /* call handlers of all modules. modules internally keep track if the IRQ source
   * needs processing.
   */
  Timein14Interrupt();
  Pwmout14Interrupt();
  Timeout14Interrupt();
} /*** end of Timer14Interrupt ***/


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
  { EXTI3_IRQHandler },                               /* EXTI Line 3                   */
  { 0 },                                              /* EXTI Line 4                   */
  { 0 },                                              /* DMA1 Channel 0                */
  { 0 },                                              /* DMA1 Channel 1                */
  { 0 },                                              /* DMA1 Channel 2                */
  { 0 },                                              /* DMA1 Channel 3                */
  { 0 },                                              /* DMA1 Channel 4                */
  { 0 },                                              /* DMA1 Channel 5                */
  { 0 },                                              /* DMA1 Channel 6                */
  { 0 },                                              /* ADC1 and ADC2, ADC3s          */
  { Can1TxInterrupt },                                /* CAN1 TX                       */
  { Can1Rx0Interrupt },                               /* CAN1 RX0                      */
  { Can1Rx1Interrupt },                               /* CAN1 RX1                      */
  { Can1ErrorInterrupt },                             /* CAN1 SCE                      */
  { 0 },                                              /* EXTI Line 9..5                */
  { Timer9Interrupt },                                /* TIM1 Break and TIM9           */
  { Timer1UpdateTimer10Interrupt },                   /* TIM1 Update and TIM10         */
  { Timer11Interrupt },                               /* TIM1 Trigger/Comm. and TIM11  */
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
  { UsbComWakeUpIRQHandler },                         /* USB OTG FS Wakeup             */
  { 0 },                                              /* TIM8 Break and TIM12          */
  { Timer8UpdateTimer13Interrupt },                   /* TIM8 Update and TIM13         */
  { Timer14Interrupt },                               /* TIM8 Trigger/Comm. and TIM14  */
  { Timer8CaptureInterrupt },                         /* TIM8 Capture Compare          */
  { 0 },                                              /* DMA1 Stream7                  */
  { 0 },                                              /* FSMC                          */
  { SDIO_IRQHandler },                                /* SDIO                          */
  { 0 },                                              /* TIM5                          */
  { 0 },                                              /* SPI3                          */
  { 0 },                                              /* UART4                         */
  { 0 },                                              /* UART5                         */
  { 0 },                                              /* TIM6 and DAC1&2 underrun err. */
  { 0 },                                              /* TIM7                          */
  { 0 },                                              /* DMA2 Stream 0                 */
  { 0 },                                              /* DMA2 Stream 1                 */
  { 0 },                                              /* DMA2 Stream 2                 */
  { DMA2_Stream3_IRQHandler },                        /* DMA2 Stream 3                 */
  { 0 },                                              /* DMA2 Stream 4                 */
  { ETH_IRQHandler },                                 /* Ethernet                      */
  { 0 },                                              /* Ethernet Wakeup               */
  { Can2TxInterrupt },                                /* CAN2 TX                       */
  { Can2Rx0Interrupt },                               /* CAN2 RX0                      */
  { Can2Rx1Interrupt },                               /* CAN2 RX1                      */
  { Can2ErrorInterrupt },                             /* CAN2 SCE                      */
  { UsbComIRQHandler },                               /* USB OTG FS                    */
  { 0 },                                              /* DMA2 Stream 5                 */
  { 0 },                                              /* DMA2 Stream 6                 */
  { 0 },                                              /* DMA2 Stream 7                 */
  { Uart6Interrupt },                                 /* USART6                        */
  { 0 },                                              /* I2C3 event                    */
  { 0 },                                              /* I2C3 error                    */
  { 0 },                                              /* USB OTG HS End Point 1 Out    */
  { 0 },                                              /* USB OTG HS End Point 1 In     */
  { 0 },                                              /* USB OTG HS Wakeup through EXTI*/
  { 0 },                                              /* USB OTG HS                    */
  { 0 },                                              /* DCMI                          */
  { 0 },                                              /* CRYP crypto                   */
  { 0 },                                              /* Hash and Rng                  */
  { 0 },                                              /* FPU                           */
  { (void*)0x55AA11EE }                               /* Reserved for OpenBLT checksum */
};

/************************************ end of vectors.c *********************************/
