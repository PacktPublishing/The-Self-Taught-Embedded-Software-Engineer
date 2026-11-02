/**
 ******************************************************************************
 * @file    itm_debug.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Shared SWV/ITM printf-style debug header
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Carried over unchanged from Chapter 2 (Exercise 2.5) onward. It is
 *          the printf transport used by every exercise that needs visibility
 *          while a peripheral is being brought up. In Chapter 5 the UART
 *          becomes a *second* output channel aimed at a PC terminal, but
 *          SWV/ITM stays the always-available debug log that does not depend
 *          on any peripheral the exercise is still trying to configure.
 *
 *          Two entry points are exposed:
 *            1. itm_send_char()   — single character to a chosen stimulus port
 *            2. itm_send_string() — null-terminated string on port 0
 *
 *          A strong __io_putchar() in itm_debug.c routes printf() through ITM.
 *
 * @note    Output requires an active STM32CubeIDE debug session with the SWV
 *          ITM Data Console open, Stimulus Port 0 enabled, and Start Trace
 *          clicked. With no debugger attached, itm_send_char() returns quietly.
 ******************************************************************************
 */

#ifndef ITM_DEBUG_H
#define ITM_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Stimulus port reserved for stdout-style text (the SWV ITM Data Console
 * default channel in STM32CubeIDE). */
#define ITM_STDOUT_PORT     (0U)

/**
 * @brief  Send a single character to an ITM stimulus port.
 * @param  port  Stimulus port number (0-31). Use ITM_STDOUT_PORT for printf.
 * @param  ch    Character byte to transmit.
 */
void itm_send_char(uint32_t port, char ch);

/**
 * @brief  Send a null-terminated string to ITM port 0.
 * @param  str  Pointer to a null-terminated string. Must not be NULL.
 */
void itm_send_string(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* ITM_DEBUG_H */
