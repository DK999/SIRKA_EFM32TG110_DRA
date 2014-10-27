/*
 * timer.h
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "efm32tg110f32.h"

void TIMER0_start(void);
void TIMER0_stop(void);
void TIMER0_intclear(void);
void TIMER1_start(void);
void TIMER1_stop(uint8_t reset);
void TIMER1_intclear(void);
#endif /* TIMER_H_ */
