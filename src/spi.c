/*
 * spi.c
 *
 *  Created on: 24.03.2014
 *      Author: Labor
 */
#include "spi.h"

#define SPI_USART USART1


// This function drives the CS pin low
void CS_Pin_clr(uint8_t CS_PIN) {
	if(CS_PIN == 3)
		GPIO_PinOutClear(gpioPortC, 15);
	else if(CS_PIN == 4)
		GPIO_PinOutClear(gpioPortD, 6);
	else if(CS_PIN == 5)
		GPIO_PinOutClear(gpioPortD, 7);
	//GPIO->P[SPI_COM_PORT].DOUTCLR = (1 << CS_PIN);

}

// This function drives the CS pin high
void CS_Pin_set(uint8_t CS_PIN) {
	if(CS_PIN == 3)
		GPIO_PinOutSet(gpioPortC, 15);
	else if(CS_PIN == 4)
		GPIO_PinOutSet(gpioPortD, 6);
	else if(CS_PIN == 5)
		GPIO_PinOutSet(gpioPortD, 7);
//	GPIO->P[SPI_COM_PORT].DOUTSET = (1 << CS_PIN);
}

// This function writes a byte to the TX buffer (waits for buffer to be cleared)
uint8_t SPI_WriteByte(uint8_t byte) {
  SPI_USART->TXDATA = byte;
  while (!(SPI_USART->STATUS & USART_STATUS_TXC)) ; // Waiting for transmission of last byte
  return(SPI_USART->RXDATA);         // clear RX buffer
}
