/*
 * crc.h
 *
 *  Created on: 24.03.2014
 *      Author: Labor
 *
 *  CRC Routine
 */

#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>

void crcInit(void);
uint16_t CRC16(uint16_t crc, uint8_t data);

#endif
