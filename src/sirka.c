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
#define CRCL 6
#define CRCH 7

#define SIRKA_ADDRESS 0
#define ACC_RES 1
#define GYRO_RES 2
#define MAG_RES 3
#define CRC_PACK 4
#define FILE_SIZE 6
#define FW_CHECK 12
#define BOOT_FLAG 13

enum sensors
{
	Gyrometer,
	Accelerometer,
	Magnetometer,
	Temperature
};
int16_t gyrodata[3] = { 0, 0, 0};
int16_t accdata[3] = { 0, 0, 0};
int16_t magdata[4] = { 0, 0, 0, 0};
uint8_t resolution[2] = { 0, 0};
uint8_t settings_acc = 0x00;

volatile uint8_t received_frame[30] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile short frame_position = 0;

uint32_t flash_save = 0x3800;
uint8_t *address = (uint8_t*)0x3800;
uint8_t *acc_res = (uint8_t*)0x3801;
uint8_t *gyro_res = (uint8_t*)0x3802;
uint8_t *mag_res = (uint8_t*)0x3803;
uint16_t *crc_pack = (uint16_t*)0x3804;
uint32_t *file_size = (uint32_t*)0x3806;
uint8_t *fw_check = (uint8_t*)0x380C;
uint8_t *boot_flag = (uint8_t*)0x380D;
uint8_t config[16];


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{ volatile uint16_t crc = 0x0000;					// Init CRC-Value
  volatile uint16_t crc_ex = 0x0000;
  config[SIRKA_ADDRESS] = *address;
  config[ACC_RES] = *acc_res;
  config[GYRO_RES] = *gyro_res;
  config[MAG_RES] = *mag_res;
  config[CRC_PACK] = (uint8_t)*crc_pack;
  config[CRC_PACK+1] = (uint8_t)(*crc_pack>>8);
  config[FILE_SIZE] = (uint8_t)*file_size;
  config[FILE_SIZE+1] = (uint8_t)(*file_size>>8);
  config[FILE_SIZE+2] = (uint8_t)(*file_size>>16);
  config[FILE_SIZE+3] = (uint8_t)(*file_size>>24);
  config[10]= 0xFF;
  config[11]= 0xFF;
  config[FW_CHECK]= *fw_check;
  config[BOOT_FLAG]= 0x00;
  config[14]= 0x00;
  config[15]= 0x00;
  /* Chip errata */
  CHIP_Init();

  /* Read Config from Flash */
  if( config[SIRKA_ADDRESS] == 0 || config[SIRKA_ADDRESS] >127)
	config[SIRKA_ADDRESS] = 0x01;
  if(!( (config[GYRO_RES] == 0x00) | (config[GYRO_RES] == 0x01) | (config[GYRO_RES] == 0x02) | (config[GYRO_RES] == 0x03) | (config[GYRO_RES] == 0x04) ))
	config[GYRO_RES] = 0x10;
  if( !((config[ACC_RES] == 0x0C) | (config[ACC_RES] == 0x08) | (config[ACC_RES] == 0x05) | (config[ACC_RES] == 0x03)) )
	config[ACC_RES] = 0x03;
  if( !((config[MAG_RES] == 0x00) | (config[MAG_RES] == 0x01) | (config[MAG_RES] == 0x02) | (config[MAG_RES] == 0x03)) )
	config[MAG_RES] = 0x00;
  ErasePage(flash_save);
  WriteWord(flash_save,&config,16);

  /* Init CRC */
  crcInit();
  /* Init SPI, USART, GPIO and Clocks  */
  interface_init();
  /* Setting up BMX055 with default values */
  setup_Acc(config[ACC_RES]);
  setup_Gyro(config[GYRO_RES]);
  setup_Mag(config[MAG_RES]);
  setup_Gyro_Range(config[GYRO_RES]);
  /* Configure Interrupt handling */
  enable_interrupts();
  /* Configure Timer for Timeout */
  Timer_Init();


  while(1)
	  {


		  frame_position = 0;					// points to last received frame byte
		  received_frame[LENGTH] = 10;
		  while( frame_position !=  received_frame[LENGTH] )			// stay in sleep mode while not received frame length and address
		  {
			  EMU_EnterEM1();
		  }
//  		  crc = 0x0000;
//  		  for(int i = 0; i < (received_frame[LENGTH]-2) ; i++)
//  		  {
//  			  crc = CRC16(crc,received_frame[i]);
//  		  }
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
			  if(received_frame[ADDRESS] == config[SIRKA_ADDRESS])
			  {	/*
				 * select received_frame
				 */
				switch(received_frame[COMMAND])
				  {		/*
						 *	Get all data from sensors, parameter defines number of Measurements ( maximum of 32 )
						 */
						case 0x00:
									getGyrData(gyrodata,received_frame[PARAMETER]);
									getAccData(accdata,received_frame[PARAMETER]);
									getMagData_forced(magdata);

									send_data_all(gyrodata,accdata,magdata);

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
									setup_Acc(settings_acc);
									break;
						/*
						 *  Do selftest of Magnetometer
						 */
						case 0x07:	selftest_Mag();
									break;
						/*
						 *  Set Gyrometer Range +-2000°/s down to +-125°/s
						 */
						case 0x10:	setup_Gyro_Range(received_frame[PARAMETER]);
									config[SIRKA_ADDRESS] = *address;
									config[ACC_RES] = *acc_res;
									config[GYRO_RES] = received_frame[PARAMETER];
									config[MAG_RES] = *mag_res;
									ErasePage(flash_save);
									WriteWord(flash_save,&config,16);
									break;
						/*
						 *  Set Accelerometer Range 2g up to 16g
						 */
						case 0x11:	setup_Acc(received_frame[PARAMETER]);
									config[SIRKA_ADDRESS] = *address;
									config[ACC_RES] = received_frame[PARAMETER];
									config[GYRO_RES] = *gyro_res;
									config[MAG_RES] = *mag_res;
									ErasePage(flash_save);
									WriteWord(flash_save,&config,16);
									break;
						/*
						 *  Set Magnetometer number of measurements
						 */
						case 0x12:	setup_Mag(received_frame[PARAMETER]);
									config[SIRKA_ADDRESS] = *address;
									config[ACC_RES] = *acc_res;
									config[GYRO_RES] = *gyro_res;
									config[MAG_RES] = received_frame[PARAMETER];
									ErasePage(flash_save);
									WriteWord(flash_save,&config,16);
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

						case 0x15:	config[BOOT_FLAG] = 0x01;
									ErasePage(flash_save);
									WriteWord(flash_save,&config,16);
									WDOG_Enable(true);
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
			  } // End Preamble-Check
  		  } // End CRC
	  } // End While(1)

  } // End Main
