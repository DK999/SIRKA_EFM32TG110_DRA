/*
 * debug.h
 *
 *  Created on: 19.03.2014
 *      Author: Labor
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "spi.h"
#include "calc.h"

void selftest_Acc(int16_t Data_difference[],uint8_t *setting);
void selftest_Gyro(void);
void selftest_Mag(void);
void advanced_selftest_mag(void);
void send_hello(void);
void showDeviceID(void);
void print_byte_setup(uint8_t byte);
void print_data(int16_t Data[], uint8_t length, uint8_t type);
void print_data_dec(int16_t Data[], uint8_t length, uint8_t type);


#endif /* DEBUG_H_ */
