/*
 * watchdog.h
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_
#include "efm32tg110f32.h"

#define bool	_Bool

void WDOG_Enable(bool enable);
void WDOG_Feed(void);
__STATIC_INLINE void BITBAND_Peripheral(volatile uint32_t *addr,uint32_t bit,uint32_t val);


#endif /* WATCHDOG_H_ */
