/*
 * emu.c
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */
#include "emu.h"

void EMU_EnterEM1(void)
{
  /* Just enter Cortex-M3 sleep mode */
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  __WFI();
}
