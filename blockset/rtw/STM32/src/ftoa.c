/************************************************************************************//**
* \file         ftoa.c
* \brief        Float to ASCII source file. Derived from http://ideone.com/XC2gPw.
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
#include <stdio.h>
#include <limits.h>


static void reverse(char* p, char* q)
{
	char c;
	for(; p < q; ++p, --q){
		c = *p;
		*p = *q;
		*q = c;
	}
} 

static char* inc(char* s, char* e, char decimalSeparator)
{
	int co = 1;
	char* t = e;
 
	//increase from end to start
	for(; t >= s; --t){
		if(*t == decimalSeparator) continue;
		*t += co;
		if(*t > '9'){
			*t = '0';
			co = 1;
		}
		else{
			co = 0;
			break;
		}
	}
	//check if there's still carry out
	if(co){
		for(t = ++e; t > s; --t) *t = *(t - 1);
		*s = '1';
	}
	return e;
}


/************************************************************************************//**
** \brief     Converts a floating point number to a string.
** \param     dest Pointer to character array where the resulting string will be stored.
** \param     num The floating point number to convert.
** \param     afterPoint Number of digits for after the decimal point. If num is 5.4321
**                       and afterPoint is 2, then the result will be 5.43.
** \param     decimalSeparator Character to be used as the decimal separator. Typically
**                             '.' or ','.
** \return    pointer to the start of the resulting string.
**
****************************************************************************************/
char* ftoa(char* dest, double num, int afterPoint, char decimalSeparator)
{
	char* p = dest;
	int integer = (int)num;
	double decimal = num - integer;
 
	// note that this conversion has a restriction. it cannot convert positive floats
	// that are greater than the max of an int (INT_MAX in limits.h).
	if (num > INT_MAX)
	{
	  integer = INT_MAX;
	  decimal = 0;
	}

	if(num < 0){
		*p++ = '-';
		integer = -integer;
		decimal = -decimal;
	}
	//parse integer
	if(integer){
		char* q = p;
		for(; integer; integer /= 10){
			*q++ = '0' + integer % 10;
		}
		reverse(p, q - 1);
		p = q;
	}
	else *p++ ='0';
	//parse decimal
	if(afterPoint > 0){
		*p++ = decimalSeparator;
		for(; afterPoint; --afterPoint){
			decimal *= 10;
			*p++ = '0' + (int)decimal;
			decimal -= (int)decimal;
		}
		if((int)(decimal + 0.5)) p = inc(dest, p - 1, decimalSeparator) + 1;
	}
 
	*p = '\0';
	return dest;
}


/*********************************** end of ftoa.c *************************************/
