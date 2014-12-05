/*
 * spi.c
 *
 *  Created on: 14.03.2014
 *      Author: Labor
 */


#include "BMX055.h"


#define SPI_USART USART1
#define SPI_COM_PORT 3 // gpioPortD (USART1 location #1)
#define SPI_TX_pin 0   // PD0 - MOSI
#define SPI_RX_pin 1   // PD1 - MISO
#define SPI_CS_GYR_pin 3   // PC15 - Chip Select
#define SPI_CS_ACC_pin 4   // PD6 - Chip Select
#define SPI_CS_MAG_pin 5   // PD7 - Chip Select

#define SIRKA_ADDRESS 0
#define ACC_RES 1
#define GYRO_RES 2
#define MAG_RES 3

// GLOBAL VARIABLES
extern uint16_t sirka_save;
uint8_t XY = 0x01 ,Z = 0x02;
extern uint8_t *acc_res;
extern uint8_t *gyro_res;
extern uint8_t *mag_res;
extern uint8_t sirka_config[8];

void interface_init(){
	  // Setup Clock Tree
		Clocks_Init();

	  // Configure GPIO
		GPIO_Init();

	  // Configure SPI Port (USART1 in sync mode)
		SPI_Init();

	  // Configure COM Port (USART0 in async mode)
		USART_Init();

		WD_Init();

		delayfunc(48000);						  	// 1,5ms delay
}



/*
 * Function gets Data from Gyroscope
 */
void getGyrData(int16_t Data[], int16_t measurements){
	uint8_t rx_data[192];			 // Temp Variable to store Sensor Measurements
	if( (measurements != 0x01) &&  (measurements != 0x04) &&  (measurements != 0x08) &&  (measurements != 0x10) )
		measurements = 0x10;
		/*
		 * Get the Data from Accelerometer
		 */
		for ( int i = 0; i<(6*measurements); i+=6)		//  Get maximum of 32 measurements
		{
			CS_Pin_clr(SPI_CS_GYR_pin);
			SPI_WriteByte(0x82);		 // send READ LSB X command
			rx_data[i] = SPI_WriteByte(0xFF); // Read LSB X
			rx_data[i+1] = SPI_WriteByte(0xFF); // Read MSB X
			rx_data[i+2] = SPI_WriteByte(0xFF); // Read LSB Y
			rx_data[i+3] = SPI_WriteByte(0xFF); // Read MSB Y
			rx_data[i+4] = SPI_WriteByte(0xFF); // Read LSB Z
			rx_data[i+5] = SPI_WriteByte(0xFF); // Read MSB Z
			CS_Pin_set(SPI_CS_GYR_pin);
		}
		/*
		 * Sort Data from Accelerometer
		 */
		arithmetic_average(rx_data,Data,(6*measurements),6,1);
}

/*
 * Function gets Data from Accelerometer
 */
void getAccData(int16_t Data[], int16_t measurements){
	uint8_t rx_data[192];			 // Temp Variable to store Sensor Measurements
	if( (measurements != 0x01) &&  (measurements != 0x04) &&  (measurements != 0x08) &&  (measurements != 0x10) )
			measurements = 0x10;
	/*
	 * Get the Data from Accelerometer
	 */
	for ( int i = 0; i<(6*measurements); i+=6)			//  Get 32 measurements
	{
		CS_Pin_clr(SPI_CS_ACC_pin);
		SPI_WriteByte(0x82);		 // send READ LSB X command
		rx_data[i] = SPI_WriteByte(0xFF); // Read LSB X
		rx_data[i+1] = SPI_WriteByte(0xFF); // Read MSB X
		rx_data[i+2] = SPI_WriteByte(0xFF); // Read LSB Y
		rx_data[i+3] = SPI_WriteByte(0xFF); // Read MSB Y
		rx_data[i+4] = SPI_WriteByte(0xFF); // Read LSB Z
		rx_data[i+5] = SPI_WriteByte(0xFF); // Read MSB Z
		CS_Pin_set(SPI_CS_ACC_pin);
	}
	/*
	 * Sort Data from Accelerometer
	 */
	arithmetic_average(rx_data,Data,(6*measurements),6,0);

}

/*
 * Function gets Data from Magnetometer in FORCE MODE
 */
void getMagData_forced(int16_t Data[]){
	uint8_t rx_data[7];			 // Temp Variable to store Sensor Measurements
	/*
	 * Set to Forced Mode
	 */
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4C);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x2E);		 // set to 20HZ and Sleep Mode ( OpMode 11 )
	CS_Pin_set(SPI_CS_MAG_pin);

	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4C);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x2A);		 // set to 20HZ and Forced Mode ( OpMode 01 )
	CS_Pin_set(SPI_CS_MAG_pin);
	if ( XY == 0x01 )
		delayfunc(320); 		// wait 100�s for measurement
	else if ( XY == 0x04 )
		delayfunc(320000);		// wait 10ms for measurement
	else if ( XY == 0x07 )
		delayfunc(640000);		// wait 20ms for measurement
	else
		delayfunc(1600000);		// wait 50ms for measurement

	/*
	 * Get the Data from Magnetometer
	 */
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0xC2);		 // send READ LSB X command
	rx_data[0] = SPI_WriteByte(0xFF); // Read LSB X
	rx_data[1] = SPI_WriteByte(0xFF); // Read MSB X
	rx_data[2] = SPI_WriteByte(0xFF); // Read LSB Y
	rx_data[3] = SPI_WriteByte(0xFF); // Read MSB Y
	rx_data[4] = SPI_WriteByte(0xFF); // Read LSB Z
	rx_data[5] = SPI_WriteByte(0xFF); // Read MSB Z
	rx_data[6] = SPI_WriteByte(0xFF); // Read LSB Hall
	rx_data[7] = SPI_WriteByte(0xFF); // Read MSB Hall
	CS_Pin_set(SPI_CS_MAG_pin);
	/*
	 * Sort Data from Magnetometer
	 */
	Data[0] = (int8_t) rx_data[1]; 		// Cast MSB X to int8_t and make it signed in 16Bit Array
	Data[0] <<=5;							// Shift MSB X 4 Bits left to have space for LSB X
	Data[0] |= (rx_data[0] >> 3);			// Do OR with first 4 Bits of LSB X
	Data[1] = (int8_t) rx_data[3];		// Cast MSB Y to int8_t and make it signed in 16Bit Array
	Data[1] <<=5;							// Shift MSB X 4 Bits left to have space for LSB X
	Data[1] |= (rx_data[2] >> 3);			// Do OR with first 4 Bits of LSB X
	Data[2] = (int8_t) rx_data[5];		// Cast MSB Z to int8_t and make it signed in 16Bit Array
	Data[2] <<=7;							// Shift MSB Z 4 Bits left to have space for LSB Z
	Data[2] |= (rx_data[4] >> 1);			// Do OR with first 4 Bits of LSB Z
	Data[3] = (int8_t) rx_data[7];		// Cast MSB Hall to int8_t and make it signed in 16Bit Array
	Data[3] <<=6;							// Shift MSB Hall 6 Bits left to have space for LSB Hall
	Data[3] |= (rx_data[6] >> 2);			// Do OR with first 6 Bits of LSB Hall
}


/*
 * Sets up Magnet Sensor with pre defined values for Repetitions in measurement
 * 0x00 	= 3 Measurements, above 300Hz possible
 * 0x01 	= 9 / 15 Measurements, 100Hz possible
 * 0x02 	= 15 / 27 Measurements, 50Hz possible
 * 0x03+ 	= 47 / 83 Measurements, 20Hz possible
 * Resolution is fixed to 0,3 �T / LSB
 * Maximum Resolution X/Y = +- 1300�T
 * Maximum Resolution Z   = +- 2500�T
 */
void setup_Mag(uint8_t preset){
	if( preset == 0x00 )		// XY = 3, Z = 3
		{
			XY = 0x01;
			Z = 0x02;
		}
	else if( preset == 0x01 )	//XY = 9, Z = 15
		{
			XY = 0x04;
			Z = 0x0E;
		}
	else if( preset == 0x02 )	// XY = 15, Z = 27
		{
			XY = 0x07;
			Z = 0x1A;
		}
	else 						// XY = 47, Z = 83
		{
			XY = 0x2E;
			Z = 0x52;
		}
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4B);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x01);		 // set into sleep mode
	CS_Pin_set(SPI_CS_MAG_pin);
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4C);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x28);		 // set to 20HZ and normal mode
	CS_Pin_set(SPI_CS_MAG_pin);
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x4E);		 // send WRITE to Axis Enable command
	SPI_WriteByte(0x07);		 // set Channel Z,Y,X, DR-Polarity, Interrupt-Latch and Interrupt-Polarity
	CS_Pin_set(SPI_CS_MAG_pin);
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x51);		 // send WRITE to Repetitions Register command
	SPI_WriteByte(XY);		 // set to 47 X/Y Repetitions
	CS_Pin_set(SPI_CS_MAG_pin);
	CS_Pin_clr(SPI_CS_MAG_pin);	 // Clear CS
	SPI_WriteByte(0x52);		 // send WRITE to Repetitions Register command
	SPI_WriteByte(Z);		 // set to 83 Z Repetitions
	CS_Pin_set(SPI_CS_MAG_pin);
	delayfunc(16000);			// Wait 0,5ms for Mag to configure
}

/*
 * Sets up Accelerometer
 * 0x03 	= +- 2g Range,	0,98mg/LSB
 * 0x05		= +- 4g Range,	1,95mg/LSB
 * 0x08		= +- 8g Range,	3,91mg/LSB
 * 0x0C		= +-16g Range,	7,81mg/LSB
 */
void setup_Acc(uint8_t range, uint8_t bandwidth){
	if( !((range == 0x0C) | (range == 0x08) | (range == 0x05) | (range == 0x03)) )			// 2g Range standard
		range = 0x03;
	if( !((bandwidth == 0x0F) | (bandwidth == 0x0E) | (bandwidth == 0x0D) | (bandwidth == 0x0C) | (bandwidth == 0x0B) | (bandwidth == 0x0A) | (bandwidth == 0x09) | (bandwidth == 0x08)) )			// Sets bandwidth
		bandwidth = 0x0F;

	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x14);		 // send BGW_Softreset command
	SPI_WriteByte(0xB6);		 // set reset
	CS_Pin_set(SPI_CS_ACC_pin);
	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x0F);		 // send WRITE to PMU Range command
	SPI_WriteByte(range);		 // set Range
	CS_Pin_set(SPI_CS_ACC_pin);
	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x10);		 // send WRITE to Sensor Control command
	SPI_WriteByte(bandwidth);	 // Set Bandwidth
	CS_Pin_set(SPI_CS_ACC_pin);
	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x11);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x00);		 // set to Normal Mode
	CS_Pin_set(SPI_CS_ACC_pin);
	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x13);		 // send WRITE to Sensor Control command
	SPI_WriteByte(0x80);		 // set to Filtered Mode
	CS_Pin_set(SPI_CS_ACC_pin);
}
/*
 * Sets up Gyrometer
 * 0x00 	= +-2000�/s Range,  61,0 m�/s / LSB
 * 0x01		= +-1000�/s Range,  30,5 m�/s / LSB
 * 0x02		= +-500�/s  Range,  15,3 m�/s / LSB
 * 0x03		= +-250�/s  Range,   7,6 m�/s / LSB
 * 0x04		= +-125�/s  Range,   3,8m�/s  / LSB
 */
void setup_Gyro(uint8_t range, uint8_t bandwidth){
	if(!( (range == 0x00) | (range == 0x01) | (range == 0x02) | (range == 0x03) | (range == 0x04) ))			// 2000�/s , 1000�/s, 500�/s, 250�/s, 125�/s Range
		{
			range = 0x00;
		}
	if(!( (bandwidth == 0x00) | (bandwidth == 0x01) | (bandwidth == 0x02) | (bandwidth == 0x03) | (bandwidth == 0x04) | (bandwidth == 0x05) | (bandwidth == 0x06) )) // checks for valid bandwidth
			{
				bandwidth = 0x00;
			}
	CS_Pin_clr(SPI_CS_GYR_pin);	 // Clear CS
	SPI_WriteByte(0x14);		 // send BGW_Softreset command
	SPI_WriteByte(0xB6);		 // set reset
	CS_Pin_set(SPI_CS_GYR_pin);

//	delayfunc(16000);			 // Wait 0,5ms for Gyro to configure
	CS_Pin_clr(SPI_CS_GYR_pin);	 // Clear CS
	SPI_WriteByte(0x0F);		 // send WRITE to Range Control command
	SPI_WriteByte(range);		 // set to defined range
	CS_Pin_set(SPI_CS_GYR_pin);
	SPI_WriteByte(0x10);		 // send WRITE to Bandwith Control command
	SPI_WriteByte(bandwidth);	 // set Bandwidth
	CS_Pin_set(SPI_CS_GYR_pin);
	CS_Pin_clr(SPI_CS_GYR_pin);	 // Clear CS
	SPI_WriteByte(0x11);		 // send WRITE to Powermodes Control command
	SPI_WriteByte(0x00);		 // set to Normal Mode
	CS_Pin_set(SPI_CS_GYR_pin);
	CS_Pin_clr(SPI_CS_GYR_pin);	 // Clear CS
	SPI_WriteByte(0x13);		 // send WRITE to HBW Control command
	SPI_WriteByte(0x00);		 // set to filtered mode
	CS_Pin_set(SPI_CS_GYR_pin);
}

/*
 * Sets Range of Gyroscope
 */
void setup_Gyro_Range(uint8_t range){
	if(!( (range == 0x00) | (range == 0x01) | (range == 0x02) | (range == 0x03) | (range == 0x04) ))			// 2000�/s , 1000�/s, 500�/s, 250�/s, 125�/s Range
		{
			range = 0x00;
		}
	CS_Pin_clr(SPI_CS_GYR_pin);	 // Clear CS
	SPI_WriteByte(0x0F);		 // send WRITE to Range Control command
	SPI_WriteByte(range);		 // set to defined range
	CS_Pin_set(SPI_CS_GYR_pin);
}
/*
 * Changes �C Address
 */

void setup_Address(uint8_t address)
{
	sirka_config[SIRKA_ADDRESS] = address;	// New Address
//	sirka_config[ACC_RES] = *acc_res;		// Saves ACC Resolution
//	sirka_config[GYRO_RES] = *gyro_res;		// Saves Gyro Resolution
//	sirka_config[MAG_RES] = *mag_res;		// Saves Magnet Resolution
	ErasePage(sirka_save);					// Deletes Page at 0x7800
	WriteWord(sirka_save,&sirka_config,8);	// writes SIRKA Config at 0x7800
}


void get_resolution(uint8_t Config[])
{
	CS_Pin_clr(SPI_CS_GYR_pin);	 // Clear CS
	SPI_WriteByte(0x8F);		 // send WRITE to Range Control command
	Config[0] = SPI_WriteByte(0xFF);	 // set to defined range
	CS_Pin_set(SPI_CS_GYR_pin);

	CS_Pin_clr(SPI_CS_ACC_pin);	 // Clear CS
	SPI_WriteByte(0x8F);		 // send WRITE to PMU Range command
	Config[1] = SPI_WriteByte(0xFF);	 // set Range
	CS_Pin_set(SPI_CS_ACC_pin);

	Config[0]<<=5;
	Config[0]>>=5;
	Config[1]<<=4;
	Config[1]>>=4;
}
