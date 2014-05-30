/*
 * irq.h
 *
 *  Created on: 17.03.2014
 *      Author: Labor
 *
 *  Functions do interrupt handling
 */

#ifndef IRQ_H_
#define IRQ_H_


#include "efm32tg110f32.h"
#include "timer.h"

void enable_interrupts(void);
void USART0_RX_IRQHandler(void);
void TIMER0_IRQHandler(void);

#endif /* IRQ_H_ */
