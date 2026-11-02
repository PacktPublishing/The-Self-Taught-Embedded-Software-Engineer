/**
 ******************************************************************************
 * @file    itm_debug.c
 * @author  Renato Soriano
 * @brief   Chapter 2 — Exercise 2.5: SWV/ITM printf-style debug output
 *          Implementation
 ******************************************************************************
 */

#include "itm_debug.h"
#include "stm32f4xx.h"   /* CMSIS — provides ITM struct and ITM_TCR_ITMENA_Msk */
#include <stddef.h>      /* for NULL */

/* =========================================================================
 * ITM register access
 *
 * The ITM peripheral is part of the ARM CoreSight debug infrastructure and
 * is memory-mapped at a fixed address in the Private Peripheral Bus (PPB).
 * The CMSIS core_cm4.h header (pulled in by stm32f4xx.h above) provides the
 * ITM struct.  We use it directly here.  Including stm32f4xx.h instead of
 * stm32f4xx_hal.h keeps this module HAL-independent so it can be reused in
 * the bare-metal projects of Chapter 4 onward without modification.
 *
 * ITM->PORT[n].u8  — byte-wide stimulus register for port n.
 *                    Writing here queues a byte for SWO output.
 * ITM->TCR         — Trace Control Register (bit 0 = ITMENA).
 *                    Must be 1 for ITM output to be active.
 * ITM->TER         — Trace Enable Register.
 *                    Bit n must be 1 to enable stimulus port n.
 * ========================================================================= */

/* =========================================================================
 * itm_send_char
 * ========================================================================= */
void itm_send_char(uint32_t port, char ch)
{
    /* Guard: only write if the ITM is enabled globally (ITMENA bit in TCR)
     * and the specific port is enabled in TER.  Without these checks the
     * write would be silently discarded, which can mislead debugging.      */
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0UL)
    {
        return;   /* ITM master enable not set — no output */
    }

    if ((ITM->TER & (1UL << port)) == 0UL)
    {
        return;   /* This stimulus port is disabled */
    }

    /* Poll until the FIFO for this port has space (TXB bit = 1 → ready).
     * Under normal debugger-connected conditions this loop exits in 1–2
     * iterations.                                                          */
    while (ITM->PORT[port].u32 == 0UL)
    {
        /* busy-wait */
    }

    /* Write the character — a single byte write triggers a 1-byte packet  */
    ITM->PORT[port].u8 = (uint8_t)ch;
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
 * __io_putchar retarget — routes printf() through ITM port 0
 *
 * In an STM32CubeIDE project, the auto-generated syscalls.c provides a
 * default _write() that loops over each byte and calls __io_putchar().
 * Both _write() and __io_putchar() are declared as weak symbols, so we
 * override __io_putchar() here with a strong definition.  The linker then
 * picks our version, and every byte produced by printf(), puts(), fprintf(),
 * etc. ends up in itm_send_char() — no UART, no extra hardware, no further
 * project changes.
 *
 * Why __io_putchar() and not fputc():
 *   The newlib-nano build (--specs=nano.specs, used by STM32CubeIDE by
 *   default) implements printf() with an internal buffer and calls _write()
 *   directly, bypassing the public fputc() symbol.  Overriding fputc()
 *   compiles cleanly but is never invoked.  The _write() / __io_putchar()
 *   path, on the other hand, is always called.
 *
 * @param  ch      Character to output (passed as int per newlib convention).
 * @return int     The character written, cast back to int.
 * ========================================================================= */
int __io_putchar(int ch)
{
    itm_send_char(ITM_STDOUT_PORT, (char)ch);

    return ch;
}
