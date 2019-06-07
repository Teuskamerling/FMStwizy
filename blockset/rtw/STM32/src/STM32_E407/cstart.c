/************************************************************************************//**
* \file         cstart.c
* \brief        GCC startup code source file.
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
#include "os.h"                                       /* for operating system          */
#include "stm32f4xx.h"                                 /* STM32 registers              */
#include "stm32f4xx_conf.h"                            /* STM32 peripheral drivers     */


/****************************************************************************************
* External function prototypes
****************************************************************************************/
extern int main(void);


/****************************************************************************************
* External data declarations
****************************************************************************************/
/* these externals are declared by the linker */
extern uint32_t _etext;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;
extern uint32_t _estack;


/****************************************************************************************
** NAME:           Reset_Handler
** PARAMETER:      none
** RETURN VALUE:   none
** DESCRIPTION:    Reset interrupt service routine. Configures the stack, initializes
**                 RAM and jumps to function main.
**
****************************************************************************************/
void Reset_Handler(void)
{
  uint32_t *pSrc, *pDest;

  /* initialize stack pointer */
  __asm("    ldr r1, =_estack\n"
        "    mov sp, r1");
  /* copy the data segment initializers from flash to SRAM */
  pSrc = &_etext;
  for(pDest = &_data; pDest < &_edata; )
  {
    *pDest++ = *pSrc++;
  }
  /* zero fill the bss segment. this is done with inline assembly since this will
   * clear the value of pDest if it is not kept in a register.
   */
  __asm("    ldr     r0, =_bss\n"
        "    ldr     r1, =_ebss\n"
        "    mov     r2, #0\n"
        "    .thumb_func\n"
        "zero_loop:\n"
        "        cmp     r0, r1\n"
        "        it      lt\n"
        "        strlt   r2, [r0], #4\n"
        "        blt     zero_loop");
  /* initialize the system and its clocks */
  SystemInit();
  /* configure NVIC priority grouping. for more information refer to:
   * http://www.freertos.org/RTOS-Cortex-M3-M4.html.
   */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  /* start the software application by calling its entry point */
  main();
} /*** end of Reset_Handler ***/


/************************************ end of cstart.c **********************************/
