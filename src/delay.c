/*
 * delay.c
 *
 *  Created on: 21.03.2014
 *      Author: Labor
 */

#include "delay.h"

// Function is used to set a delay
void delayfunc(int cycles)
{
	  DWT->CYCCNT = 0;
	  while (DWT->CYCCNT < cycles);
}
