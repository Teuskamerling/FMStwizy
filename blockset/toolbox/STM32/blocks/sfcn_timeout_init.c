/************************************************************************************//**
* file         sfcn_timeout_init.c
* brief        Level-2 S-Function for initializing a timer module for output compare
*              purposes.
*
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
****************************************************************************************/

/*=====================================*
 * Required setup for C MEX S-Function *
 *=====================================*/

#define S_FUNCTION_NAME sfcn_timeout_init
#define S_FUNCTION_LEVEL 2



/*========================*
 * General Defines/macros *
 *========================*/



 /*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h" 



// -------------------------------------------------------------------------------
// Macros to access the S-function parameter values
// -------------------------------------------------------------------------------

#define NUMBER_OF_ARG		0					// Number of s-function input arguments



// ----------------------------------------------------------------------------------------------------
// S-Function methods
// ----------------------------------------------------------------------------------------------------

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
  /* Function: mdlCheckParameters =============================================
   * Abstract:
   *    Validate our parameters to verify they are okay.
   */
static void mdlCheckParameters(SimStruct *S)
{
}
#endif /* MDL_CHECK_PARAMETERS */


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Initialize the sizes array
 */
/* Function: mdlInitializeSizes ===============================================
 *
 */
static void mdlInitializeSizes (SimStruct *S)
{
  /* set the expexted number of parameters for the S-function block */
  ssSetNumSFcnParams(S, NUMBER_OF_ARG);
   
  /* verify the number of parametrs that are specified for the S-function block */
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
  {
    /* incorrect number of parameters */
    mexPrintf("Incorrect number of parameters (%d). Should be %d\n",
             ssGetSFcnParamsCount(S), ssGetNumSFcnParams(S));
    return;
  }

  /* needed to force checking on the parameter data types */
  mdlCheckParameters(S);
  if (ssGetErrorStatus(S) != NULL)
  {
    return;
  }

  /* set number of input ports for the S-function block */
  if(!ssSetNumInputPorts(S, 0))
  {
    mexPrintf("Could not set number of input ports to %d\n", 0);
    return;
  }
   
  /* set number of output ports forthe S-function block */
  if(!ssSetNumOutputPorts(S, 1))
  {
    mexPrintf("Could not set number of output ports to %d\n", 1);
    return;
  }

  /* set the port width of the output port */
  ssSetOutputPortWidth(S, 0, 1);

  /* output is a function call. note that only the first output of an
   * S-function can be of this type.
   */
  ssSetOutputPortDataType(S, 0, SS_FCN_CALL); 
  
  /* only 1 sampletime in this S-Function */
  ssSetNumSampleTimes (S, 1);

  ssSetNumContStates (S, 0);					// number of continuous states
  ssSetNumDiscStates (S, 0);					// number of discrete states
  ssSetNumIWork (S, 0);						// number of integer work vector elements
  ssSetNumRWork (S, 0);						// number of real work vector elements
  ssSetNumPWork (S, 0);						// number of pointer work vector elements
  ssSetNumDWork(S, 0); // number of data type work vector elements
   
  /* options */
  ssSetOptions(S,SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE);
}


/* Function: mdlInitializeSampleTimes =========================================
 *
 */
static void mdlInitializeSampleTimes (SimStruct *S)
{
  /* the first element of output port 0 triggers the function call */
  ssSetCallSystemOutput(S, 0); 
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
  ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


/* Function: mdlStart =========================================================
 *
 */
#define MDL_START
static void mdlStart(SimStruct *S)
{
   /* not used here */
}


/*
 * mdlOutputs - compute the outputs
 *
 * In this function, you compute the outputs of your S-function
 * block.  The outputs are placed in the y variable.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
   /* call the function call subsystem */
  if (!ssCallSystemWithTid(S, 0, tid)) 
  {
    /* No function call subsystem connected or error occurred, which will 
     * be reported by the Simulink engine.
     */
    return;   
  }
}


/*
 * mdlTerminate - called when the simulation is terminated.
 *
 * In this function, you should perform any actions that are necessary
 * at the termination of a simulation.  For example, if memory was allocated
 * in mdlInitializeConditions, this is the place to free it.
 */
static void mdlTerminate (SimStruct *S)
{
   /* not used here */
}


// the define 'MATLAB_MEX_FILE' has to be specified when recompiling this module to a DLL.
#ifdef MATLAB_MEX_FILE
   #include "simulink.c"
#else
# error "Attempt to use <s_function>.c as non-inlined S-function"
#endif
/********************************* end of sfcn_timeout_init.c **************************/

