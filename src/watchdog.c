/*
 * watchdog.c
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */

#include "watchdog.h"

#define bool	_Bool

void WDOG_Enable(bool enable)
{
  if (!enable)
  {
    /* Wait for any pending previous write operation to have been completed in */
    /* low frequency domain */
    while (WDOG->SYNCBUSY & (0x1UL << 0));
  }
  BITBAND_Peripheral(&(WDOG->CTRL), 0, (unsigned int)enable);
}

void WDOG_Feed(void)
{
  /* The watchdog should not be fed while it is disabled */
  if ( !(WDOG->CTRL & (0x1UL << 0)) )
  {
    return;
  }

  /* If a previous clearing is being synchronized to LF domain, then there */
  /* is no point in waiting for it to complete before clearing over again. */
  /* This avoids stalling the core in the typical use case where some idle loop */
  /* keeps clearing the watchdog. */
  if (WDOG->SYNCBUSY & (0x1UL << 1))
  {
    return;
  }
  /* Before writing to the WDOG_CMD register we also need to make sure that
   * any previous write to WDOG_CTRL is complete. */
  while ( WDOG->SYNCBUSY & (0x1UL << 0) );

  WDOG->CMD = (0x1UL << 0);
}

__STATIC_INLINE void BITBAND_Peripheral(volatile uint32_t *addr, uint32_t bit, uint32_t val)
{
#if defined(BITBAND_PER_BASE)
  uint32_t tmp =  BITBAND_PER_BASE + (((uint32_t)addr - PER_MEM_BASE) * 32) + (bit * 4);

  *((volatile uint32_t *)tmp) = (uint32_t)val;
#else
  uint32_t tmp = *addr;
  /* Make sure val is not more than 1, because we only want to set one bit. */
  val &= 0x1;
  *addr = (tmp & ~(1 << bit)) | (val << bit);
#endif /* defined(BITBAND_PER_BASE) */
}
