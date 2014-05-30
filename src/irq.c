/*
 * irq.c
 *
 *  Created on: 17.03.2014
 *      Author: Labor
 */


#include "irq.h"


extern uint8_t received_frame[30];
extern short frame_position;
void enable_interrupts(void)
{
	/* Enable interrupt for USART0 */
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_RX_IRQn);
	USART0->IEN |= USART_IEN_RXDATAV;
}
/* saves received byte in 'received_frame'
 * increments frame_position after each byte
 * starts timer if transmission gets interrupted
 */
void USART0_RX_IRQHandler(void)
{
	if(frame_position == 0)			// Start Timer when receiving first IRQ
			TIMER_start();
	received_frame[frame_position++] = USART0->RXDATA;
	if(frame_position == received_frame[2])				// if full frame received deactivate timer
	{
		TIMER_stop();
	}
}

/* occurs if data stream isn't fully received by USART0 */
void TIMER0_IRQHandler(void)
{
TIMER_intclear();      			// Clear overflow flag
TIMER_stop();					// disable timer
frame_position = 0;				// error while getting frame, reset frame position to zero
received_frame[2] = 10;			// give PACKET-LENGTH a fixed value
}
