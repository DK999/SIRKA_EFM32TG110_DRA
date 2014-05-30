/*
 * spi.h
 *
 *  Created on: 14.03.2014
 *      Author: Labor
 *
 *  Includes basic functions for using BMX055 Sensor
 *  Initializes interface, sets up sensors, gets data
 */

#ifndef BMX055_H_
#define BMX055_H_

#include <stdint.h>
#include "efm32tg110f32.h"
#include "calc.h"
#include "spi.h"
#include "delay.h"
#include "irq.h"
#include "inits.h"
#include "usart.h"


void interface_init(void);
void getAccData(int16_t Data[3],int16_t measurements);
void getGyrData(int16_t Data[3],int16_t measurements);
void setup_Mag(uint8_t preset);
void setup_Acc(uint8_t range);
void setup_Gyro(uint8_t range);
void setup_Gyro_Range(uint8_t range);
void getMagData_forced(int16_t Data[]);
void setup_Address(uint8_t address);
void get_resolution(uint8_t Config[]);


#endif /* BMX055_H_ */
