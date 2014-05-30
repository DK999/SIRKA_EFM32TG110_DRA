/*
 * msc.c
 *
 *  Created on: 27.05.2014
 *      Author: Labor
 */
#include "msc.h"

#define EFM_ASSERT(expr)    ((void)0)
#define MSC_PROGRAM_TIMEOUT 10000000ul
#define WORDS_PER_DATA_PHASE (1)

int8_t ErasePage(uint32_t *startAddress)
{
  int32_t      timeOut  = MSC_PROGRAM_TIMEOUT;

  /* Address must be aligned to pages */
  EFM_ASSERT((((uint32_t) startAddress) & (FLASH_PAGE_SIZE - 1)) == 0);

  /* Enable writing to the MSC */
  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

  /* Load address */
  MSC->ADDRB    = (uint32_t) startAddress;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  /* Check for invalid address */
  if (MSC->STATUS & MSC_STATUS_INVADDR)
  {
    /* Disable writing to the MSC */
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
    return -1;
  }

  /* Check for write protected page */
  if (MSC->STATUS & MSC_STATUS_LOCKED)
  {
    /* Disable writing to the MSC */
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
    return -2;
  }

  /* Send erase page command */
  MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

  /* Wait for the erase to complete */
  while ((MSC->STATUS & MSC_STATUS_BUSY) && (timeOut != 0))
  {
    timeOut--;
  }

  if (timeOut == 0)
  {
    /* Disable writing to the MSC */
    MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
    return -3;
  }

  /* Disable writing to the MSC */
  MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;
  return 0;
}

int8_t WriteWord(uint32_t *address, void const *data, int numBytes)
{
  int wordCount;
  int numWords;
#if defined(_EFM32_TINY_FAMILY) || defined (_EFM32_GIANT_FAMILY) || defined(_EFM32_WONDER_FAMILY)
  int pageWords;
  uint32_t* pData;
#endif
  int8_t retval = 0;

  /* Check alignment (Must be aligned to words) */
  EFM_ASSERT(((uint32_t) address & 0x3) == 0);

  /* Check number of bytes. Must be divisable by four */
  EFM_ASSERT((numBytes & 0x3) == 0);

  /* Enable writing to the MSC */
  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

  /* Convert bytes to words */
  numWords = numBytes >> 2;

#if defined(_EFM32_TINY_FAMILY) || defined (_EFM32_GIANT_FAMILY) || defined(_EFM32_WONDER_FAMILY)

  pData = (uint32_t*) data;

  /* The following loop splits the data into chunks corresponding to flash pages.
     The address is loaded only once per page, because the hardware automatically
     increments the address internally for each data load inside a page. */
  for (wordCount = 0; wordCount < numWords; )
  {
    /* First we load address. The address is auto-incremented within a page.
       Therefore the address phase is only needed once for each page. */
    retval = LoadAddress(address + wordCount);
    if (0 != retval) goto msc_write_word_exit;

    /* Compute the number of words to write to the current page. */
    pageWords =
      (FLASH_PAGE_SIZE - ((uint32_t) (address + wordCount)) % FLASH_PAGE_SIZE) /
      sizeof(uint32_t);
    if (pageWords > numWords-wordCount)
      pageWords = numWords-wordCount;
    wordCount += pageWords;

    /* Now program the data in this page. */
    for (; pageWords; pData+=WORDS_PER_DATA_PHASE, pageWords-=WORDS_PER_DATA_PHASE)
    {
      retval = LoadData(pData, WORDS_PER_DATA_PHASE);
      if (0 != retval)
        goto msc_write_word_exit;
    }
  }
#endif

 msc_write_word_exit:

  /* Disable writing to the MSC */
  MSC->WRITECTRL &= ~MSC_WRITECTRL_WREN;

  return retval;
}

int8_t LoadData(uint32_t* data, int num)
{
  int      timeOut  = MSC_PROGRAM_TIMEOUT;
  int      i;

  /* Wait for the MSC to be ready for a new data word.
   * Due to the timing of this function, the MSC should
   * already by ready */
  timeOut = MSC_PROGRAM_TIMEOUT;
  while (((MSC->STATUS & MSC_STATUS_WDATAREADY) == 0) && (timeOut != 0))
  {
    timeOut--;
  }

  /* Check for timeout */
  if (timeOut == 0)
    return -3;

  /* Load 'num' 32-bit words into write data register. */
  for (i=0; i<num; i++, data++)
    MSC->WDATA = *data;

  /* Trigger write once */
  MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

  /* Wait for the write to complete */
  timeOut = MSC_PROGRAM_TIMEOUT;
  while ((MSC->STATUS & MSC_STATUS_BUSY) && (timeOut != 0))
  {
    timeOut--;
  }

  /* Check for timeout */
  if (timeOut == 0) return -3;

  return 0;
}

int8_t LoadAddress(uint32_t* address)
{
  uint32_t status;

  /* Load address */
  MSC->ADDRB    = (uint32_t) (address);
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  status = MSC->STATUS;
  if (status & (MSC_STATUS_INVADDR | MSC_STATUS_LOCKED))
  {
    /* Check for invalid address */
    if (status & MSC_STATUS_INVADDR)
      return -1;
    /* Check for write protected page */
    if (status & MSC_STATUS_LOCKED)
      return -2;
  }
  return 0;
}
