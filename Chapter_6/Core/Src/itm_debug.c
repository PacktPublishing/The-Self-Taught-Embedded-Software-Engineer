/**
 ******************************************************************************
 * @file    itm_debug.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — SWV/ITM printf-style debug output (implementation)
 *
 * @details Shared verbatim across every Chapter 5 exercise folder so each
 *          project opens and builds standalone in STM32CubeIDE without
 *          referencing files outside its own directory. The intent is
 *          identical to Chapter 2 Exercise 2.5: route printf() to the SWO pin
 *          through ITM stimulus port 0.
 ******************************************************************************
 */

#include "itm_debug.h"
#include "stm32f4xx.h"   /* CMSIS — provides the ITM struct and bit masks */
#include <stddef.h>      /* for NULL */

/* =========================================================================
 * itm_send_char
 *
 * ITM->TCR bit 0 (ITMENA) is the global master enable; ITM->TER bit n enables
 * stimulus port n. Both are set by the debugger when SWV tracing starts, so we
 * check them defensively: with no probe attached the function is a quiet no-op
 * instead of spinning forever on a FIFO that never drains.
 * ========================================================================= */
void itm_send_char(uint32_t port, char ch)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0UL)
    {
        return;   /* ITM master enable not set — no debug probe attached */
    }

    if ((ITM->TER & (1UL << port)) == 0UL)
    {
        return;   /* this stimulus port is not enabled in the debugger */
    }

    while (ITM->PORT[port].u32 == 0UL)
    {
        /* busy-wait for FIFO-not-full; one or two iterations under a probe */
    }

    ITM->PORT[port].u8 = (uint8_t)ch;   /* byte write -> 1-byte SWO packet */
}

/* =========================================================================
 * itm_send_string
 * ========================================================================= */
void itm_send_string(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        itm_send_char(ITM_STDOUT_PORT, *str);
        str++;
    }
}

/* =========================================================================
 * __io_putchar retarget — routes printf() through ITM port 0.
 *
 * The CubeIDE-generated syscalls.c declares __io_putchar() weak; providing a
 * strong definition here sends every printf()/puts()/fprintf() byte to
 * itm_send_char() with no UART and no further project changes.
 * ========================================================================= */
int __io_putchar(int ch)
{
    itm_send_char(ITM_STDOUT_PORT, (char)ch);
    return ch;
}
