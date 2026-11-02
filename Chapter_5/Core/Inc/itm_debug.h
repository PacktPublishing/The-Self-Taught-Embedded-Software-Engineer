/**
 ******************************************************************************
 * @file    itm_debug.h
 * @author  Renato Soriano
 * @brief   Chapter 2 — Exercise 2.5: SWV/ITM printf-style debug output
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details The Instrumentation Trace Macrocell (ITM) is a hardware unit
 *          built into every ARM Cortex-M3/M4/M7 core.  It provides a set
 *          of 32 stimulus ports that the SWD/SWO pin can stream out to the
 *          debug probe.  STM32CubeIDE shows this data in the
 *          Window → Show View → SWV ITM Data Console.
 *
 *          This module provides two things:
 *
 *          1. itm_send_char() — low-level single-character write to
 *             ITM stimulus port 0.
 *
 *          2. A retarget of the C standard library's fputc() so that
 *             printf() automatically routes through the ITM, with no
 *             UART required.
 *
 *          How to enable SWV in STM32CubeIDE:
 *          1. In Run/Debug Configurations → Debugger tab, tick
 *             "Enable SWV" and enter the core clock (e.g. 16000000 for
 *             16 MHz HSI).
 *          2. Open the SWV ITM Data Console view.
 *          3. Click the wrench icon, enable ITM Stimulus Port 0.
 *          4. Click "Start Trace" (red circle button).
 *          5. Run the program — text appears in the console.
 *
 * @note    SWO output consumes CPU cycles for each character written.
 *          Use it for debugging only, not in production firmware.
 ******************************************************************************
 */

#ifndef ITM_DEBUG_H
#define ITM_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Stimulus port used for printf output.
 * Port 0 is the default for text output and matches the ITM Data Console
 * default channel selection in STM32CubeIDE.
 * ------------------------------------------------------------------------- */
#define ITM_STDOUT_PORT     (0U)

/* -------------------------------------------------------------------------
 * Exported function prototypes
 * ------------------------------------------------------------------------- */

/**
 * @brief  Send a single character to an ITM stimulus port.
 *
 * @param  port   ITM stimulus port number (0–31).  Use ITM_STDOUT_PORT
 *                (port 0) for printf-routed text.
 * @param  ch     Character to transmit.
 *
 * @note   The function polls the port's FIFO-not-full flag (TXB bit) before
 *         writing.  If the debug probe is not connected the FIFO never
 *         drains and this function busy-waits indefinitely.  A debugger
 *         connection is therefore required when ITM output is enabled.
 */
void itm_send_char(uint32_t port, char ch);

/**
 * @brief  Write a null-terminated string to ITM port 0.
 *
 *         Convenience wrapper that calls itm_send_char() for each byte,
 *         including a trailing newline if the string does not already end
 *         with one.  Useful for log messages inside functions.
 *
 * @param  str    Pointer to null-terminated string (must not be NULL).
 */
void itm_send_string(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* ITM_DEBUG_H */
