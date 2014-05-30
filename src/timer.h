/*
 * timer.h
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "efm32tg110f32.h"

void TIMER_start(void);
void TIMER_stop(void);
void TIMER_intclear(void);
#endif /* TIMER_H_ */
