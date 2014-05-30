/*
 * calc.c
 *
 *  Created on: 17.03.2014
 *      Author: Labor
 */
#include "calc.h"

/*
 * Converts Integer to Char / String for displaying
 */
void ItoA( int z, char* Buffer )
{
  int i = 0;
  int j;
  char tmp;
  unsigned u;

    if( z < 0 ) {						// If Number is negative, put - in front of string
      Buffer[0] = '-';
      Buffer++;
      u = ( (unsigned)-(z+1) ) + 1;
    }
    else {
      u = (unsigned)z;
    }

    do {								// Split into numbers and save it to string
      Buffer[i++] = '0' + u % 10;
      u /= 10;
    } while( u > 0 );


    for( j = 0; j < i / 2; ++j ) {		// Invert string
      tmp = Buffer[j];
      Buffer[j] = Buffer[i-j-1];
      Buffer[i-j-1] = tmp;
    }
    Buffer[i] = '\0';					// End string
}

/*
 * Calculates the arithmetic average of duration / meas_per_cycle measurements
 */
void arithmetic_average ( uint8_t rx_data[],int16_t Data[],uint16_t duration,uint8_t meas_per_cycle)
{	/*
	 * Create Temp Variable for adding values
	 */
	int32_t Tempx = 0;
	int32_t Tempy = 0;
	int32_t Tempz = 0;
	/*
	 * Copy both Bytes into one 16Bit signed integer, then add to Temp-Variables
	 * duration describes how many cycles the measurement lasts, total amount of cycles by duration / meas_per_cycle
	 */
	for ( int i = 0; i<duration; i+=meas_per_cycle)
	{
		Data[0] = (int8_t) rx_data[i+1]; 		// Cast MSB X to int8_t and make it signed in 16Bit Array
		Data[0] <<=4;							// Shift MSB X 4 Bits left to have space for LSB X
		Data[0] |= (rx_data[i] >> 4);			// Do OR with first 4 Bits of LSB X
		Tempx += Data[0];						// Add Data to Temp-Var
		Data[1] = (int8_t) rx_data[i+3];		// Cast MSB Y to int8_t and make it signed in 16Bit Array
		Data[1] <<=4;							// Shift MSB X 4 Bits left to have space for LSB X
		Data[1] |= (rx_data[i+2] >> 4);			// Do OR with first 4 Bits of LSB X
		Tempy += Data[1];						// Add Data to Temp-Var
		Data[2] = (int8_t) rx_data[i+5];		// Cast MSB Z to int8_t and make it signed in 16Bit Array
		Data[2] <<=4;							// Shift MSB Z 4 Bits left to have space for LSB Z
		Data[2] |= (rx_data[i+4] >> 4);			// Do OR with first 4 Bits of LSB Z
		Tempz += Data[2];						// Add Data to Temp-Var
	}
	/*
	 * calculate arithmetic average and save into array
	 */
	Data[0] = (int16_t)Tempx / (duration / meas_per_cycle);
	Data[1] = (int16_t)Tempy / (duration / meas_per_cycle);
	Data[2] = (int16_t)Tempz / (duration / meas_per_cycle);
}
