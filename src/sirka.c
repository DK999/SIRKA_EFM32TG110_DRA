/*
 *
 * CLOCK = 			48MHz
 * SPI-CLOCK =		10MHz
 * UART = 			3000000 Bd/s
 *
 * ##### SENSORS #####
 * ACC Range 	: 0,98µg per Bit, Maximum 2g
 * GYRO Range 	: 30,5m°/s per Bit, Maximum 1000°/s
 * MAG Range	: 0,3µT per Bit, Maximum 1300µT for XY, 2500µT for Z
 */
#include "efm32tg110f32.h"
#include "em_device.h"
#include "msc.h"
#include "watchdog.h"
#include "em_chip.h"    // required for CHIP_Init() function
#include "BMX055.h"
#include "debug.h"
#include "irq.h"
#include "emu.h"
#include "inits.h"


#define PREAMBLE_ONE 0
#define PREAMBLE_TWO 1
#define LENGTH 2
#define ADDRESS 3
#define COMMAND 4
#define PARAMETER 5
#define PARAMETER_TWO 6
#define BCID 6
//#define CRCL 6
//#define CRCH 7

#define SIRKA_ADDRESS 0
#define ACC_RES 1
#define GYRO_RES 2
#define MAG_RES 3
#define CRC_PACK 0
#define FILE_SIZE 2
#define FW_CHECK 6
#define BOOT_FLAG 7

enum sensors
{
	Gyrometer,
	Accelerometer,
	Magnetometer,
	Temperature
};
volatile uint16_t systime = 0;
volatile uint32_t systime_rec = 0;
int16_t gyrodata[3] = { 0, 0, 0};
int16_t accdata[3] = { 0, 0, 0};
int16_t magdata[4] = { 0, 0, 0, 0};
uint8_t resolution[2] = { 0, 0};
uint8_t settings_acc = 0x00;
uint8_t bcid = 0x00;

volatile uint8_t received_frame[30] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 40, 0, 0, 0, 0, 0, 0};
volatile short frame_position = 0;

uint16_t sirka_save = 0x7800;					// Declares SIRKA Config at 0x7800
uint16_t boot_save = 0x3800;					// Declares Bootloader Config at 0x3800
uint8_t *address = (uint8_t*)0x7800;			// Address Pointer to 0x7800
uint8_t *acc_res = (uint8_t*)0x7801;			// ACC Pointer to 0x7801
uint8_t *gyro_res = (uint8_t*)0x7802;			// Gyro Pointer to 0x7802
uint8_t *mag_res = (uint8_t*)0x7803;			// Mag Pointer to 0x7803
uint16_t *crc_pack = (uint16_t*)0x3800;			// CRC of App Pointer to 0x3800
uint32_t *file_size = (uint32_t*)0x3802;		// Filesize of App Pointer to 0x3802
uint8_t *fw_check = (uint8_t*)0x3806;			// APP_OK Pointer to 0x3806
uint8_t *boot_flag = (uint8_t*)0x3807;			// Bootflag Pointer to 0x3807
uint8_t sirka_config[8] = { 0,0,0,0,0,0,0,0 };
uint8_t boot_config[8] = { 0,0,0,0,0,0,0,0 };


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{ volatile uint16_t crc = 0x0000;					// Init CRC-Value
  volatile uint16_t crc_ex = 0x0000;
  uint8_t changed = 0;
  sirka_config[SIRKA_ADDRESS] = *address;			// Reads SIRKA Address from Flash
  sirka_config[ACC_RES] = *acc_res;					// Reads ACC Resolution from Flash
  sirka_config[GYRO_RES] = *gyro_res;				// Reads Gyro Resolution from Flash
  sirka_config[MAG_RES] = *mag_res;					// Reads Mag Resolution from Flash
  boot_config[CRC_PACK] = (uint8_t)*crc_pack;		// Reads CRC of App from Flash
  boot_config[CRC_PACK+1] = (uint8_t)(*crc_pack>>8);
  boot_config[FILE_SIZE] = (uint8_t)*file_size;		// Reads Filesize of App from Flash
  boot_config[FILE_SIZE+1] = (uint8_t)(*file_size>>8);
  boot_config[FILE_SIZE+2] = (uint8_t)(*file_size>>16);
  boot_config[FILE_SIZE+3] = (uint8_t)(*file_size>>24);
  boot_config[FW_CHECK]= *fw_check;					// Reads APP_OK Flag from Flash
  boot_config[BOOT_FLAG]= 0x00;						// Sets Bootloader Flag to 0

  /* Chip errata */
  CHIP_Init();

  /* Check for unallowed configs */
  if( sirka_config[SIRKA_ADDRESS] == 0 || sirka_config[SIRKA_ADDRESS] >127)
	  {
	  	  sirka_config[SIRKA_ADDRESS] = 0x01;
	  	  changed = 1;
	  }
  if(!( (sirka_config[GYRO_RES] == 0x00) || (sirka_config[GYRO_RES] == 0x01) || (sirka_config[GYRO_RES] == 0x02) || (sirka_config[GYRO_RES] == 0x03) || (sirka_config[GYRO_RES] == 0x04) ))
	  {
	  	  sirka_config[GYRO_RES] = 0x10;
	  	  changed = 1;
	  }
  if( !((sirka_config[ACC_RES] == 0x0C) || (sirka_config[ACC_RES] == 0x08) || (sirka_config[ACC_RES] == 0x05) || (sirka_config[ACC_RES] == 0x03)) )
	  {
	  	  sirka_config[ACC_RES] = 0x03;
	  	  changed = 1;
	  }
  if( !((sirka_config[MAG_RES] == 0x00) || (sirka_config[MAG_RES] == 0x01) || (sirka_config[MAG_RES] == 0x02) || (sirka_config[MAG_RES] == 0x03)) )
	  {
	  	  sirka_config[MAG_RES] = 0x00;
	  	  changed = 1;
	  }
  /* Erase and save config if not correct */
  if(changed == 1)
  {
	  ErasePage(sirka_save);
	  WriteWord(sirka_save,&sirka_config,8);
  }


  /* Init CRC */
  crcInit();
  /* Init SPI, USART, GPIO and Clocks  */
  interface_init();
  /* Setting up BMX055 with default values */
  setup_Acc(sirka_config[ACC_RES],0x0F);
  setup_Gyro(sirka_config[GYRO_RES],0);
  setup_Mag(sirka_config[MAG_RES]);
  setup_Gyro_Range(sirka_config[GYRO_RES]);
  /* Configure Interrupt handling */
  enable_interrupts();
  /* Configure Timer for Timeout */
  Timer_Init();

  WD_Init();

  TIMER1_start();

  while(1)
	  {
		  frame_position = 0;					// points to last received frame byte
		  received_frame[LENGTH] = 30;
		  while( frame_position !=  received_frame[LENGTH] )			// stay in sleep mode while not received frame length and address
		  {
			  EMU_EnterEM1();
		  }
//  		  crc = 0x0000;
//  		  for(int i = 0; i < (received_frame[LENGTH]-2) ; i++)
//  		  {
//  			  crc = CRC16(crc,received_frame[i]);
//  		  }
//  		  crc_ex = (uint16_t)received_frame[received_frame[LENGTH]-1];               // Store CRC-m_checksum
//  		  crc_ex <<= 8;
//  		  crc_ex |= received_frame[received_frame[LENGTH]-2];
//  		  crc_ex = (uint16_t)received_frame[CRCH];               // Store CRC-m_checksum
//  		  crc_ex <<= 8;
//  		  crc_ex |= received_frame[CRCL];
  		  /*
  		   * Check if Preamble OK and frame fully received
  		   */

  		  if(true)
  		  {
		  if((received_frame[PREAMBLE_ONE] == 0xAA) && (received_frame[PREAMBLE_TWO] == 0xAA))
			{
			  /*
			   * Check for address
			   */
			  if(received_frame[ADDRESS] == sirka_config[SIRKA_ADDRESS])
			  {	/*
				 * select command
				 */
				switch(received_frame[COMMAND])
				  {		/*
						 *	Get all data from sensors, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x00:
									getGyrData(gyrodata,received_frame[PARAMETER]);
									getAccData(accdata,received_frame[PARAMETER]);
									getMagData_forced(magdata);

									send_data_all(gyrodata,accdata,magdata,false);

									break;
						/*
						 *  Get Gyroscope data, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x01: 	getGyrData(gyrodata,received_frame[PARAMETER]);
									send_data(gyrodata,Gyrometer);
									break;
						/*
						 *  Get Accelerometer data, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x02: 	getAccData(accdata,received_frame[PARAMETER]);
									send_data(accdata,Accelerometer);
									break;
						/*
						 *  Get Magnetometer data, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x03: 	getMagData_forced(magdata);
									send_data(magdata,Magnetometer);
									break;
						/*
						 *  Get current resolution
						 */
						case 0x04:	get_resolution(resolution);
									send_resolution(resolution);
									break;
						/*
						 *  Do selftest of Gyrometer
						 */
						case 0x05:	selftest_Gyro();
									break;
						/*
						 *  Do selftest of Accelerometer
						 */
						case 0x06:	selftest_Acc(accdata,&settings_acc);
									send_data(accdata,Accelerometer);
									setup_Acc(settings_acc,0x0F);
									break;
						/*
						 *  Do selftest of Magnetometer
						 */
						case 0x07:	selftest_Mag();
									break;
						/*
						 *  Set Gyrometer Range +-2000°/s down to +-125°/s
						 */
						case 0x10:	//setup_Gyro_Range(received_frame[PARAMETER]);
									setup_Gyro(received_frame[PARAMETER],received_frame[PARAMETER_TWO]);
//									sirka_config[SIRKA_ADDRESS] = *address;
//									sirka_config[ACC_RES] = *acc_res;
									sirka_config[GYRO_RES] = received_frame[PARAMETER];
//									sirka_config[MAG_RES] = *mag_res;
									ErasePage(sirka_save);
									WriteWord(sirka_save,&sirka_config,8);
									break;
						/*
						 *  Set Accelerometer Range 2g up to 16g
						 */
						case 0x11:	setup_Acc(received_frame[PARAMETER],received_frame[PARAMETER_TWO]);
//									sirka_config[SIRKA_ADDRESS] = *address;
									sirka_config[ACC_RES] = received_frame[PARAMETER];
//									sirka_config[GYRO_RES] = *gyro_res;
//									sirka_config[MAG_RES] = *mag_res;
									ErasePage(sirka_save);
									WriteWord(sirka_save,&sirka_config,8);
									break;
						/*
						 *  Set Magnetometer number of measurements
						 */
						case 0x12:	setup_Mag(received_frame[PARAMETER]);
//									sirka_config[SIRKA_ADDRESS] = *address;
//									sirka_config[ACC_RES] = *acc_res;
//									sirka_config[GYRO_RES] = *gyro_res;
									sirka_config[MAG_RES] = received_frame[PARAMETER];
									ErasePage(sirka_save);
									WriteWord(sirka_save,&sirka_config,8);
									break;
						/*
						 *  Change Address of µC
						 */
						case 0x13:	setup_Address(received_frame[PARAMETER]);
									break;
						/*
						 *  Sends current address
						 */
						case 0x14:	send_hello();
									break;
						/*
						 * Reboots Device, sets Bootloader-Flag
						 */
						case 0x15:	boot_config[BOOT_FLAG] = 0x01;
									ErasePage(boot_save);
									WriteWord(boot_save,&boot_config,8);
									WDOG_Enable(true);
									break;
						/*
						 * Sends all Data got from last Broadcast command, resets BroadcastID
						 */
						case 0x16:	send_data_all(gyrodata,accdata,magdata,true);
									bcid = 0;
									break;

						/*  ########################################
						 *  ########## 	DEBUG FUNCTIONS! ###########
						 *  ########################################
						 *  ##	     Sending Data in ASCII!		  ##
						 *  ##		 For use with terminal		  ##
						 *  ########################################
						 *  ########################################
						 *  Get all data from sensors, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x20:	getGyrData(gyrodata,received_frame[PARAMETER]);
									getAccData(accdata,received_frame[PARAMETER]);
									getMagData_forced(magdata);

									print_data_dec(gyrodata,3,Gyrometer);
									print_data_dec(accdata,3,Accelerometer);
									print_data_dec(magdata,4,Magnetometer);
									break;
						/*
						 *  Get Gyroscope data, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x21:	getGyrData(gyrodata,received_frame[PARAMETER]);
									print_data_dec(gyrodata,3,Gyrometer);
									break;
						/*
						 *  Get Accelerometer data, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x22:	getAccData(accdata,received_frame[PARAMETER]);
									print_data_dec(accdata,3,Accelerometer);
									break;
						/*
						 *  Get Magnetometer data, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x23:	getMagData_forced(magdata);
									print_data_dec(magdata,4,Magnetometer);
									break;

						/*
						 *  Shows Device ID
						 */
						case 0xFF: 	showDeviceID();
									break;

						default:
									break;
					  } // End SWITCH

				  } // End Address-Check
			  else if(received_frame[ADDRESS] == 128)	// if BROADCAST
			  {
				  switch(received_frame[COMMAND])
				  {		/*
						 *	Get all data from sensors, parameter defines number of Measurements ( maximum of 32 ), just stores data and BCID
						 */
				  	  	  case 0x00:
									getGyrData(gyrodata,received_frame[PARAMETER]);
									getAccData(accdata,received_frame[PARAMETER]);
									getMagData_forced(magdata);
									bcid = received_frame[BCID];
									systime_rec = 0;						// Set systime_rec to 0 for OR operation
									TIMER1_stop(0);							// Stop Timer
									systime_rec = (uint16_t) systime;		// Set first 16Bit
									systime_rec <<= 16;						// Shift 16Bit left to higher values
									systime_rec |= (uint16_t) TIMER1->CNT;	// Do OR with 16Bit Timer Counter
									TIMER1_start();							// Start Timer again
									break;
						/*
						 * Reset Systime
						 */
				  	  	  case 0x01:
				  	  		  	  	TIMER1_stop(1);				// Stop Timer1 complete and reset CNT
				  	  		  	  	systime = 0;				// Reset Systime
				  	  		  	  	TIMER1_start();				// Start Counter again
				  	  		  	  	break;
				  	  	/*
				  	  	 * Send data Broadcast chain
				  	  	 */
				  	  	  case 0x02:
				  	  		  	  	if ( sirka_config[SIRKA_ADDRESS] == 0x01 )
				  	  		  	  	{
				  	  		  	  		send_data_all(gyrodata,accdata,magdata,true);									// sends data immediately if units address is 01
				  	  		  	  	}
				  	  		  	  	else
				  	  		  	  	{
				  	  		  	  		volatile uint32_t timeout=1;		// creating non-zero timeout variable, maximum of 65 units!
				  	  		  	  		received_frame[23] = 40;
				  	  		  	  		/*
				  	  		  	  		 * Receives Package, checks if package is from Address-1
				  	  		  	  		 * If no package is received, a timeout starts transmission anyway
				  	  		  	  		 * timeout multiplied by own address to prevent bus conflicts if timeout happened
				  	  		  	  		 */
				  	  		  	  		while( received_frame[23] != (sirka_config[SIRKA_ADDRESS]-1) && (timeout > 0) )
				  	  		  	  		{
				  	  		  	  			timeout = 1000*sirka_config[SIRKA_ADDRESS];									// resets timeout after each received package
				  	  		  	  			frame_position = 0;															// points to last received frame byte
											received_frame[LENGTH] = 30;												// set length of received frame to value > 3
											while( frame_position !=  received_frame[LENGTH] && (timeout > 0))			// stay in loop while not received frame length and address or timeout
											{
												--timeout;
											}
				  	  		  	  		}
				  	  		  	  			send_data_all(gyrodata,accdata,magdata,true);								// sends data
				  	  		  	  	}
				  	  		  	  	break;

				  }
			  }
			  } // End Preamble-Check
  		  } // End CRC
	  } // End While(1)

  } // End Main
