/*
 * msc.h
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */

#ifndef MSC_H_
#define MSC_H_

#include "efm32tg110f32.h"

int8_t ErasePage(uint32_t *startAddress);
int8_t WriteWord(uint32_t *address, void const *data, int numBytes);
int8_t LoadData(uint32_t* data, int num);
int8_t LoadAddress(uint32_t* address);
#endif /* MSC_H_ */
