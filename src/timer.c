/*
 * timer.c
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */

#include "timer.h"

void TIMER_start(void)
{
	TIMER0->CMD = ( 1 << 0 );
}

void TIMER_stop(void)
{

	TIMER0->CMD = ( 1 << 1 );
	TIMER0->CNT = 0;
}

void TIMER_intclear(void)
{
	TIMER0->IFC = 0x1;
}
