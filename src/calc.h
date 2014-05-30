/*
 * calc.h
 *
 *  Created on: 17.03.2014
 *      Author: Labor
 *
 *  Includes integer to char conversion and building arithmetic average of values
 */

#ifndef CALC_H_
#define CALC_H_

#include "BMX055.h"

void ItoA( int z, char* Buffer );
void arithmetic_average ( uint8_t rx_data[],int16_t Data[],uint16_t duration,uint8_t meas_per_cycle);

#endif /* CALC_H_ */
