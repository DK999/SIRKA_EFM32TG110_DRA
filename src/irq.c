/*
 * irq.c
 *
 *  Created on: 17.03.2014
 *      Author: Labor
 */


#include "irq.h"


extern uint8_t received_frame[30];
extern short frame_position;
extern uint32_t systime;

void enable_interrupts(void)
{
	/* Enable interrupt for USART0 */
	NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	NVIC_EnableIRQ(USART0_RX_IRQn);
	USART0->IEN |= USART_IEN_RXDATAV;
	/* Enable interrupt for TIMER0 */
	TIMER0->IEN = 0x1;
	NVIC_EnableIRQ(TIMER0_IRQn);              // Enable TIMER0 interrupt vector in NVIC
	/* Enable interrupt for TIMER1 */
	TIMER1->IEN = 0x1;
	NVIC_EnableIRQ(TIMER1_IRQn);              // Enable TIMER0 interrupt vector in NVIC

}
/* saves received byte in 'received_frame'
 * increments frame_position after each byte
 * starts timer, if transmission is interrupted TIMER0_IRQ gets called
 * if transmission succeded, TIMER0 is stopped
 */
void USART0_RX_IRQHandler(void)
{	TIMER0_stop();				// stops timer
	TIMER0_start();
	received_frame[frame_position++] = USART0->RXDATA;
	if(frame_position == received_frame[2])				// if full frame received deactivate timer
	{
		TIMER0_stop();
	}
}

/* occurs if data stream isn't fully received by USART0 */
void TIMER0_IRQHandler(void)
{
TIMER0_intclear();      		// Clear overflow flag
TIMER0_stop();					// disable timer
frame_position = 0;				// error while getting frame, reset frame position to zero
received_frame[2] = 30;			// give PACKET-LENGTH a fixed value
}

void TIMER1_IRQHandler(void)
{
TIMER1_stop();						// stop TIMER
TIMER1_intclear();      			// Clear overflow flag
systime+=5;							// Add 5µs to Systime
TIMER1_start();						// start TIMER again
}

