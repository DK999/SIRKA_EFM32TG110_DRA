/*
 * usart.c
 *
 *  Created on: 25.03.2014
 *      Author: Labor
 */

#include "usart.h"

#define COM_USART USART0
#define USART_CS_PORT 2
#define USART_CS_PIN 14


// This function is used to display bytes over UART
void print_byte(uint8_t byte) {
  while( !(COM_USART->STATUS & (1 << 6)) ); // wait for TX buffer to empty
  COM_USART->TXDATA = byte;                 // send byte over UART
  while (!(COM_USART->STATUS & USART_STATUS_TXC)) ; // Waiting for transmission of last byte
}

void send_data(int16_t Data[],uint8_t type)
{	uint8_t byte[13];
	uint8_t counter = 4;
	volatile uint16_t crc = 0x0000;					// Init CRC-Value

	byte[0]= 0xAA;
	byte[1]= 0xAA;
	byte[3]= 0x00;

	for ( int i = 0; i < 3 ; i++)					// For ACC/GYRO put Data into separate Bytes
	{
		byte[counter] = (uint8_t) Data[i];
		byte[counter+1] = (uint8_t) (Data[i] >> 8);
		counter+=2;
	}
	if(type == 2)									// If MAGNET, put HALL Data into separate Bytes
	{	byte[10] = (uint8_t) Data[3];
		byte[11] = (uint8_t) (Data[3] >> 8);
		GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);				// Set RS485 for Write

		byte[2] = 0x0E;							// Sending Frame length ( 14 Byte )

		for ( int i = 0; i < 12 ; i++)				// Send all 8 Bytes of Mag Data
		{
			crc = CRC16(crc,byte[i]);				// Calculate CRC
			print_byte(byte[i]);					// Send over USART
		}
		byte[12] = (uint8_t)crc;						// Split CRC into two Bytes
		byte[13] = (uint8_t)(crc >> 8 );
		print_byte(byte[12]);						// Send both bytes over USART
		print_byte(byte[13]);

	}
	else
	{	GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);				// Set RS485 for Write

		byte[2] = 0x0C;							// Sending Frame length ( 12 Byte )

		for ( int i = 0; i < 10 ; i++)				// Send all 8 Bytes of ACC/GYRO Data
		{
			crc = CRC16(crc,byte[i]);				// Calculate CRC
			print_byte(byte[i]);					// Send over USART
		}

		byte[10] = (uint8_t)crc;						// Split CRC into two Bytes
		byte[11] = (uint8_t)(crc >> 8 );
		print_byte(byte[10]);						// Send both bytes over USART
		print_byte(byte[11]);

	}
	GPIO->P[USART_CS_PORT].DOUTCLR = (1 << USART_CS_PIN);					// Clear RS485 for Read
}

void send_data_all(int16_t Gyr[], int16_t Acc[], int16_t Mag[])
{	uint8_t byte[24];
	uint8_t counter = 4;
	volatile uint16_t crc = 0x0000;

	byte[0]= 0xAA;
	byte[1]= 0xAA;
	byte[2]= 0x18;
	byte[3]= 0x00;

	for ( int i = 0; i < 3 ; i++)
	{
		byte[counter] = (uint8_t) Gyr[i];
		byte[counter+1] = (uint8_t) (Gyr[i] >> 8);
		counter+=2;
	}
	for ( int i = 0; i < 3 ; i++)
	{
		byte[counter] = (uint8_t) Acc[i];
		byte[counter+1] = (uint8_t) (Acc[i] >> 8);
		counter+=2;
	}
	for ( int i = 0; i < 3 ; i++)
	{
		byte[counter] = (uint8_t) Mag[i];
		byte[counter+1] = (uint8_t) (Mag[i] >> 8);
		counter+=2;
	}
	GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);

	for ( int i = 0; i < 22 ; i++)
	{
		crc = CRC16(crc,byte[i]);				// create CRC
		print_byte(byte[i]);					// Send bytes
	}
	byte[0] = (uint8_t)crc;						// split CRC in two bytes
	byte[1] = (uint8_t)(crc >> 8 );

	print_byte(byte[0]);						// send CRC LSB
	print_byte(byte[1]);						// send CRC MSB

	GPIO->P[USART_CS_PORT].DOUTCLR = (1 << USART_CS_PIN);					// Clear RS485 for Read

}

void send_resolution(uint8_t Config[])
{	volatile uint16_t crc = 0x0000;
	uint8_t byte[6];

	byte[0]= 0xAA;							// Setting Preamble
	byte[1]= 0xAA;
	byte[2]= 0x08;							// Setting Host Address
	byte[3]= 0x00;							// Setting Frame length (8 Byte)
	byte[4] = Config[0];					// Setting config Gyrometer
	byte[5] = Config[1];					// Setting config Accelerometer

	GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);			// Set RS485 for Write

	for ( int i = 0; i < 6 ; i++)
	{
		crc = CRC16(crc,byte[i]);			// create CRC
		print_byte(byte[i]);				// Send bytes
	}
	byte[0] = (uint8_t)crc;					// split CRC in two bytes
	byte[1] = (uint8_t)(crc >> 8 );

	print_byte(byte[0]);					// send CRC LSB
	print_byte(byte[1]);					// send CRC MSB

	GPIO->P[USART_CS_PORT].DOUTCLR = (1 << USART_CS_PIN);			// Clear RS485 for Read
}
