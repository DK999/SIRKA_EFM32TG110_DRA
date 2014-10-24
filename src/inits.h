/*
 * inits.h
 *
 *  Created on: 21.03.2014
 *      Author: Labor
 *
 *  Sets up all needed Interfaces and Clocks
 */

#ifndef INITS_H_
#define INITS_H_

#include <stdint.h>
#include "efm32tg110f32.h"


void SPI_Init(void);
void USART_Init(void);
void Clocks_Init(void);
void Timer_Init(void);
void GPIO_Init(void);
void WD_Init(void);
//void RTC_Init(void);

#endif /* INITS_H_ */
