/*
 * timer.c
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */

#include "timer.h"

void TIMER0_start(void)
{
	TIMER0->CMD = ( 1 << 0 );
}

void TIMER0_stop(void)
{

	TIMER0->CMD = ( 1 << 1 );
	TIMER0->CNT = 0;
}

void TIMER0_intclear(void)
{
	TIMER0->IFC = 0x1;
}

void TIMER1_start(void)
{
	TIMER1->CMD = ( 1 << 0 );
}

void TIMER1_stop(void)
{

	TIMER1->CMD = ( 1 << 1 );
	TIMER1->CNT = 0;
}

void TIMER1_intclear(void)
{
	TIMER1->IFC = 0x1;
}
