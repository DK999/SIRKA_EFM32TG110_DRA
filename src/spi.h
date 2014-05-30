/*
 * spi.h
 *
 *  Created on: 24.03.2014
 *      Author: Labor
 *
 *  Basic SPI Routines for Chip Select and Write / Read data
 */

#ifndef SPI_H_
#define SPI_H_

#include "efm32tg110f32.h"
#include "em_gpio.h"
#include <stdint.h>

void CS_Pin_clr(uint8_t CS_PIN);
void CS_Pin_set(uint8_t CS_PIN);
uint8_t SPI_WriteByte(uint8_t byte);

#endif /* SPI_H_ */
