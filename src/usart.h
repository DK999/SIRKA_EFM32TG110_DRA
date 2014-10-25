/*
 * usart.h
 *
 *  Created on: 25.03.2014
 *      Author: Labor
 *
 *  USART communication with host
 */

#ifndef USART_H_
#define USART_H_

#include <stdint.h>
#include "efm32tg110f32.h"
#include "crc.h"

void print_byte(uint8_t byte);
void send_data(int16_t Data[],uint8_t type);
void send_data_all(int16_t Gyr[], int16_t Acc[], int16_t Mag[],uint8_t broadcast);
void send_resolution(uint8_t Config[]);

#endif /* USART_H_ */
