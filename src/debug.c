/*
 * debug.c
 *
 *  Created on: 19.03.2014
 *      Author: Labor
 */

#include "debug.h"


#define SPI_USART USART1
#define SPI_COM_PORT 3 		// gpioPortD (USART1 location #1)
#define SPI_TX_pin 0   		// PD0 - MOSI
#define SPI_RX_pin 1   		// PD1 - MISO
#define SPI_CS_GYR_pin 3   	// PC15 - Chip Select
#define SPI_CS_ACC_pin 4   	// PD6 - Chip Select
#define SPI_CS_MAG_pin 5   	// PD7 - Chip Select
#define USART_CS_PIN 14
#define USART_CS_PORT 2

uint8_t print_byte_array[4] = { 0x30, 0x78, 0x20, 0x20 }; // array for displaying bytes one nibble at a time
extern uint8_t sirka_config[8];
/*
 * ####### DEBUG FUNCTIONS #######
 */

/*
 * Creates an offset around sensors, if X/Y-Value > 200 | X/Y-Value <-200 and Z-Value > 102 | Z-Value < -102 Test succeded
 */
void selftest_Acc(int16_t Data_difference[],uint8_t *setting)
{	uint8_t rx_data_plus[6] = { 0,0,0,0,0,0};
	uint8_t rx_data_minus[6] = { 0,0,0,0,0,0};
	int16_t Data_plus[3]= { 0, 0, 0};
	int16_t Data_minus[3]= { 0, 0, 0};

	CS_Pin_clr(SPI_CS_ACC_pin);	 	// Clear CS
	SPI_WriteByte(0x8F);		 	// send WRITE to Sensor Control command
	*setting = SPI_WriteByte(0xFF);	// Read Setting for Sensor Resolution
	CS_Pin_set(SPI_CS_ACC_pin);

	*setting &= ~0xF0;				// deleting unnecessary bits

	CS_Pin_clr(SPI_CS_ACC_pin);	 	// Clear CS
	SPI_WriteByte(0x0F);			// send WRITE to Sensor Control command
	SPI_WriteByte(0x08);		 	// set Range to 8g for Test
	CS_Pin_set(SPI_CS_ACC_pin);
	/*
	 * Excitation positive
	 */
	CS_Pin_clr(SPI_CS_ACC_pin);	 	// Clear CS
	SPI_WriteByte(0x32);		 	// send WRITE to PMU_SELF_TEST
	SPI_WriteByte(0x15);		 	// Self-Test for x-axis
	CS_Pin_set(SPI_CS_ACC_pin);
	delayfunc(3200000);				//wait 100ms for Test

	CS_Pin_clr(SPI_CS_ACC_pin);
	SPI_WriteByte(0x82);		 	// send READ LSB X command
	rx_data_plus[0] = SPI_WriteByte(0xFF); // Read LSB X
	rx_data_plus[1] = SPI_WriteByte(0xFF); // Read MSB X
	CS_Pin_set(SPI_CS_ACC_pin);

	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x32);		 // send WRITE to PMU_SELF_TEST
	SPI_WriteByte(0x16);		 // Self-Test for y-axis
	CS_Pin_set(SPI_CS_ACC_pin);
	delayfunc(3200000);			//wait 100ms for Test

	CS_Pin_clr(SPI_CS_ACC_pin);
	SPI_WriteByte(0x84);		 // send READ LSB Y command
	rx_data_plus[2] = SPI_WriteByte(0xFF); // Read LSB Y
	rx_data_plus[3] = SPI_WriteByte(0xFF); // Read MSB Y
	CS_Pin_set(SPI_CS_ACC_pin);

	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x32);		 // send WRITE to PMU_SELF_TEST
	SPI_WriteByte(0x17);		 // Self-Test for y-axis
	CS_Pin_set(SPI_CS_ACC_pin);
	delayfunc(3200000);			//wait 100ms for Test

	CS_Pin_clr(SPI_CS_ACC_pin);
	SPI_WriteByte(0x86);		 // send READ LSB Z command
	rx_data_plus[4] = SPI_WriteByte(0xFF); // Read LSB Z
	rx_data_plus[5] = SPI_WriteByte(0xFF); // Read MSB Z
	CS_Pin_set(SPI_CS_ACC_pin);
	/*
	 * Excitation negative
	 */
	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x32);		 // send WRITE to PMU_SELF_TEST
	SPI_WriteByte(0x11);		 // Self-Test for x-axis
	CS_Pin_set(SPI_CS_ACC_pin);
	delayfunc(3200000);			//wait 100ms for Test

	CS_Pin_clr(SPI_CS_ACC_pin);
	SPI_WriteByte(0x82);		 // send READ LSB X command
	rx_data_minus[0] = SPI_WriteByte(0xFF); // Read LSB X
	rx_data_minus[1] = SPI_WriteByte(0xFF); // Read MSB X
	CS_Pin_set(SPI_CS_ACC_pin);

	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x32);		 // send WRITE to PMU_SELF_TEST
	SPI_WriteByte(0x12);		 // Self-Test for y-axis
	CS_Pin_set(SPI_CS_ACC_pin);
	delayfunc(3200000);			//wait 100ms for Test

	CS_Pin_clr(SPI_CS_ACC_pin);
	SPI_WriteByte(0x84);		 // send READ LSB Y command
	rx_data_minus[2] = SPI_WriteByte(0xFF); // Read LSB Y
	rx_data_minus[3] = SPI_WriteByte(0xFF); // Read MSB Y
	CS_Pin_set(SPI_CS_ACC_pin);

	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x32);		 // send WRITE to PMU_SELF_TEST
	SPI_WriteByte(0x13);		 // Self-Test for y-axis
	CS_Pin_set(SPI_CS_ACC_pin);
	delayfunc(3200000);			//wait 100ms for Test

	CS_Pin_clr(SPI_CS_ACC_pin);
	SPI_WriteByte(0x86);		 // send READ LSB Y command
	rx_data_minus[4] = SPI_WriteByte(0xFF); // Read LSB Y
	rx_data_minus[5] = SPI_WriteByte(0xFF); // Read MSB Y
	CS_Pin_set(SPI_CS_ACC_pin);

	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x14);		 // send BGW_Softreset command
	SPI_WriteByte(0xB6);		 // set reset
	CS_Pin_set(SPI_CS_ACC_pin);

	arithmetic_average(rx_data_plus,Data_plus,6,1,0);
	arithmetic_average(rx_data_minus,Data_minus,6,1,0);

	Data_difference[0] = Data_plus[0] - Data_minus[0];
	Data_difference[1] = Data_plus[1] - Data_minus[1];
	Data_difference[2] = Data_plus[2] - Data_minus[2];
}
/*
 * Reads the Value of "rate_ok" from BIST-Register, '1' indicates proper sensor function
 */
void selftest_Gyro(void)
{
	uint8_t byte[7];
	volatile uint16_t crc = 0x0000;

	byte[0] = 0xAA;
	byte[1] = 0xAA;
	byte[2] = 0x00;
	byte[3] = 0x07;
	CS_Pin_clr(SPI_CS_GYR_pin);
	SPI_WriteByte(0xBC);
	byte[4] = SPI_WriteByte(0xFF);
	CS_Pin_set(SPI_CS_GYR_pin);
	byte[4]<<=3;
	byte[4]>>=7;

	for(int i=0; i<5; i++)
	{
		crc = CRC16(crc,byte[i]);
		print_byte(byte[i]);
	}
	byte[5] = (uint8_t)crc;						// Split CRC into two Bytes
	byte[6] = (uint8_t)(crc >> 8 );
	print_byte(byte[5]);						// Send both bytes over USART
	print_byte(byte[6]);
}
/*
 * Starts Self-Test, reads Bit0 of each LSB Register
 * If '1', Self-Test is successful
 */
void selftest_Mag(void)
{	uint8_t rx_data[3];
	uint8_t byte[9];
	volatile uint16_t crc = 0x0000;

	byte[0] = 0xAA;
	byte[1] = 0xAA;
	byte[2] = 0x00;
	byte[3] = 0x09;

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4B);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x01);		 // set into sleep mode
	CS_Pin_set(SPI_CS_MAG_pin);
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4C);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x2F);		 // set to 20HZ and sleep mode ( OpMode 11 ), perform normal self test
	CS_Pin_set(SPI_CS_MAG_pin);
	delayfunc(3200000);			// Wait 100ms for Self-Test

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0xC2);		 // send READ LSB X command
	rx_data[0] = SPI_WriteByte(0xFF); // Read LSB X
	SPI_WriteByte(0xFF); // Read MSB X
	rx_data[1] = SPI_WriteByte(0xFF); // Read LSB Y
	SPI_WriteByte(0xFF); // Read MSB Y
	rx_data[2] = SPI_WriteByte(0xFF); // Read LSB Z
	CS_Pin_set(SPI_CS_MAG_pin);

	byte[4] = rx_data[0] & 0x01;
	byte[5] = rx_data[1] & 0x01;
	byte[6] = rx_data[2] & 0x01;

	for(int i=0; i<7; i++)
		{
			crc = CRC16(crc,byte[i]);
			print_byte(byte[i]);
		}
	byte[7] = (uint8_t)crc;						// Split CRC into two Bytes
	byte[8] = (uint8_t)(crc >> 8 );
	print_byte(byte[7]);						// Send both bytes over USART
	print_byte(byte[8]);

}

void advanced_selftest_mag()
{	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4B);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x01);		 // set into sleep mode
	CS_Pin_set(SPI_CS_MAG_pin);

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4E);		 // send WRITE to Axis Enable command
	SPI_WriteByte(0x1F);		 // set Channel Z,DR-Polarity, Interrupt-Latch and Interrupt-Polarity, disable X/Y
	CS_Pin_set(SPI_CS_MAG_pin);

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4C);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0xBA);		 // set to 20HZ and Forced Mode ( OpMode 01 ), perform AST with neg current
	CS_Pin_set(SPI_CS_MAG_pin);
	delayfunc(3200000);			// wait 100ms for measurement

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0xC7);		 // send READ LSB X command
	SPI_WriteByte(0xFF); // Read LSB Z
	SPI_WriteByte(0xFF); // Read MSB Z
	SPI_WriteByte(0xFF); // Read LSB HALL
	SPI_WriteByte(0xFF); // Read MSB HALL
	CS_Pin_set(SPI_CS_MAG_pin);

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4C);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0xFA);		 // set to 20HZ and Forced Mode ( OpMode 01 ), perform AST with pos current
	CS_Pin_set(SPI_CS_MAG_pin);
	delayfunc(3200000);			// wait 100ms for measurement

	CS_Pin_clr(SPI_CS_MAG_pin);	// Clear CS
	SPI_WriteByte(0xC7);		// send READ LSB X command
	SPI_WriteByte(0xFF); 		// Read LSB Z
	SPI_WriteByte(0xFF); 		// Read MSB Z
	SPI_WriteByte(0xFF); 		// Read LSB HALL
	SPI_WriteByte(0xFF); 		// Read MSB HALL
	CS_Pin_set(SPI_CS_MAG_pin);

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4C);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x28);		 // set to 20HZ and normal mode ( OpMode 00 )
	CS_Pin_set(SPI_CS_MAG_pin);

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4B);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x00);		 // set into suspend mode
	CS_Pin_set(SPI_CS_MAG_pin);

}


void send_hello(void)
{	volatile uint16_t crc = 0x0000;
	uint8_t byte[7];

	byte[0] = 0xAA;							// Setting Preamble
	byte[1] = 0xAA;
	byte[2] = 0x07;							// Setting Frame length (7 Byte)
	byte[3] = 0x00;							// Setting Host Address
	byte[4] = sirka_config[0];

	GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);			// Set RS485 for Write

	for ( int i = 0; i < 5 ; i++)
	{
		crc = CRC16(crc,byte[i]);			// create CRC
		print_byte(byte[i]);				// Send bytes
	}
	byte[5] = (uint8_t)crc;					// split CRC in two bytes
	byte[6] = (uint8_t)(crc >> 8 );

	print_byte(byte[5]);					// send CRC LSB
	print_byte(byte[6]);					// send CRC MSB

	GPIO->P[USART_CS_PORT].DOUTCLR = (1 << USART_CS_PIN);			// Clear RS485 for Read
}

void showDeviceID(void){
	uint8_t rxdata[3] = { 0, 0, 0};
	char Gyro[] = "\n\rGyro-ID\n\r";
	char Acc[] = "\n\rAcc-ID\n\r";
	char Mag[] = "\n\rMag-ID\n\r";
	/*
	 * Getting DeviceID information, storing to rxdata
	 */
	CS_Pin_clr(SPI_CS_GYR_pin);		// drive CS pin low
	SPI_WriteByte(0x80);			// send 'Device ID' command
	rxdata[0] = SPI_WriteByte(0x00); 	// store manuf. id
	CS_Pin_set(SPI_CS_GYR_pin);		// drive CS pin high

	CS_Pin_clr(SPI_CS_ACC_pin);		// drive CS pin low
	SPI_WriteByte(0x80);			// send 'Device ID' command
	rxdata[1] = SPI_WriteByte(0x00); 	// store manuf. id
	CS_Pin_set(SPI_CS_ACC_pin);		// drive CS pin high

	CS_Pin_clr(SPI_CS_MAG_pin);		// drive CS pin low
	SPI_WriteByte(0xC0);			// send 'Device ID' command
	rxdata[2] = SPI_WriteByte(0x00); // store manuf. id
	CS_Pin_set(SPI_CS_MAG_pin);		// drive CS pin high
	/*
	 * prepare for uart transfer
	 */
	GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);			// Set RS485 for Write

	for ( int i = 0; Gyro[i]!=0; i++)
		print_byte(Gyro[i]);			// Sending chars
	print_byte_setup(rxdata[0]);    	// break up byte into individual digits (nibbles)
	for( int i=0; i<4; i++) 			// Sending value in Hex
	    print_byte(print_byte_array[i]);// Transmit high and low nibbles over UART

	for ( int i = 0; Acc[i]!=0; i++)
			print_byte(Acc[i]);			// Sending chars
	print_byte_setup(rxdata[1]);    	// break up byte into individual digits (nibbles)
	for( int i=0; i<4; i++) 			// Sending value in Hex
		print_byte(print_byte_array[i]);// Transmit high and low nibbles over UART

	for ( int i = 0; Mag[i]!=0; i++)
		print_byte(Mag[i]);			// Sending chars
	print_byte_setup(rxdata[2]);    	// break up byte into individual digits (nibbles)
	for( int i=0; i<4; i++)				// Sending value in Hex
		print_byte(print_byte_array[i]);// Transmit high and low nibbles over UART

	GPIO->P[USART_CS_PORT].DOUTCLR = (1 << USART_CS_PIN);			// Clear RS485 for Read
}

// Changes Bit to Hexadecimal for UART Transfer
void print_byte_setup(uint8_t byte) {

  if (((byte & 0xF0) >> 4) <= 0x09) {                      // if high nibble is less than 0xA
    print_byte_array[2] = ((byte & 0xF0) >> 4) + 0x30;     // store ASCII char
  }
  if ((((byte & 0xF0) >> 4) >= 0x0A) && (((byte & 0xF0) >> 4)<= 0x0F)) { // if high nibble is between 0xA and 0xF
    print_byte_array[2] = ((byte & 0xF0) >> 4) + 0x37;     // store ASCII char
  }
  if ((byte & 0x0F) <= 0x09) {                             // if low nibble is less than 0xA
    print_byte_array[3] = (byte & 0x0F) + 0x30;            // store ASCII char
  }
  if (((byte & 0x0F) >= 0x0A) && ((byte & 0x0F)<= 0x0F)) { // if low nibble is between 0xA and 0xF
    print_byte_array[3] = (byte & 0x0F) + 0x37;            // store ASCII char
  }
}

void print_data(int16_t Data[],uint8_t length, uint8_t type){
	/*
	 * Output of Sensor type
	 */
	uint8_t counter = 0;
	uint8_t bytes[6] = { 0, 0, 0, 0, 0, 0};
	GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);			// Set RS485 for Write
	if(type == 0){
		char GYRO[] = "\n\rGYROMETER \n\r";
		for ( int i = 0; GYRO[i]!=0; i++){
			print_byte(GYRO[i]);
		}
	}else if(type == 1){
		char ACC[] = "\n\rACCELEROMETER \n\r";
		for ( int i = 0; ACC[i]!=0; i++){
			print_byte(ACC[i]);
		}
	}else if(type == 2){
		char MAG[] = "\n\rMAGNETOMETER \n\r";
		for ( int i = 0; MAG[i]!=0; i++){
			print_byte(MAG[i]);
		}
	}else if(type == 3){
		char TEMP[] = "\n\rTEMPERATURE \n\r";
		for ( int i = 0; TEMP[i]!=0; i++){
			print_byte(TEMP[i]);
		}
	}

	/*
	 * seperate data into Bytes
	 */
	for ( int i=0; i<length; i++){
		bytes[counter] = (uint8_t) Data[i];
		bytes[counter+1] = (uint8_t) (Data[i] >> 8);
		counter+=2;
	}
	/*
	 * Send Bytes to UART
	 */

	for ( int x = 0; x<length*2;x++)
		{
		  print_byte_setup(bytes[x]);    // break up byte into individual digits (nibbles)
		    for( int i=0; i<4; i++) {
		      print_byte(print_byte_array[i]); // Transmit high and low nibbles over UART
		    }

		    print_byte(0x20); // send whitespace
		}
	GPIO->P[USART_CS_PORT].DOUTCLR = (1 << USART_CS_PIN);			// Clear RS485 for Read
}

void print_data_dec(int16_t Data[], uint8_t length, uint8_t type)
{
	/*
	 * Output of Sensor type
	 */
	char transmit1[20];
	char transmit2[20];
	char transmit3[20];
	char transmit4[20];
	GPIO->P[USART_CS_PORT].DOUTSET = (1 << USART_CS_PIN);			// Set RS485 for Write
	if(type == 0){
		char GYRO[] = "\n\rGYROMETER \n\r";
		for ( int i = 0; GYRO[i]!=0; i++){
			print_byte(GYRO[i]);
		}
	}else if(type == 1){
		char ACC[] = "\n\rACCELEROMETER \n\r";
		for ( int i = 0; ACC[i]!=0; i++){
			print_byte(ACC[i]);
		}
	}else if(type == 2){
		char MAG[] = "\n\rMAGNETOMETER \n\r";
		for ( int i = 0; MAG[i]!=0; i++){
			print_byte(MAG[i]);
		}
	}else if(type == 3){
		char TEMP[] = "\n\rTEMPERATURE \n\r";
		for ( int i = 0; TEMP[i]!=0; i++){
			print_byte(TEMP[i]);
		}
	}

	/*
	 * convert to string
	 */
	ItoA(Data[0],transmit1);
	ItoA(Data[1],transmit2);
	ItoA(Data[2],transmit3);
	/*
	 * Send string to UART
	 */

	for ( int i = 0; transmit1[i]!=0; i++)
				print_byte(transmit1[i]);
	print_byte(0x20); // send whitespace
	for ( int i = 0; transmit2[i]!=0; i++)
				print_byte(transmit2[i]);
	print_byte(0x20); // send whitespace
	for ( int i = 0; transmit3[i]!=0; i++)
				print_byte(transmit3[i]);
	// If Magnetometer, send out 4th value
	if(type == 2){
				ItoA(Data[3],transmit4);
				print_byte(0x20); // send whitespace
				for ( int i = 0; transmit4[i]!=0; i++)
							print_byte(transmit4[i]);
				}
	GPIO->P[USART_CS_PORT].DOUTCLR = (1 << USART_CS_PIN);			// Clear RS485 for Read
}

