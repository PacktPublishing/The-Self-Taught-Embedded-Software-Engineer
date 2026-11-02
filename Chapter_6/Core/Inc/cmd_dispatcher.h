/**
 ******************************************************************************
 * @file    cmd_dispatcher.h
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.3: a line assembler and command dispatcher
 *          fed from the interrupt-driven UART receive path
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details The USART RX interrupt (Chapter 5, Exercise 5.2b) drops each byte
 *          into a lock-free ring buffer and returns. This module is the
 *          consumer side that runs in the main loop: it collects those bytes
 *          into a line, and when a full line arrives (Enter pressed) it splits
 *          off the first word as the command, matches it against a caller-
 *          supplied table, and runs the matching handler.
 *
 *          Nothing here touches a peripheral directly. The dispatcher is given
 *          an output function at init time and calls it to send replies, so the
 *          same command set works over USART2/VCP or USART6 unchanged. Handlers
 *          receive the argument text and a reply buffer to fill.
 ******************************************************************************
 */

#ifndef CMD_DISPATCHER_H
#define CMD_DISPATCHER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Longest single input line the assembler will hold (excess is discarded). */
#define CMD_LINE_MAX     (64U)

/**
 * @brief One command handler.
 * @param args        Text after the command word (may be empty, never NULL).
 * @param reply       Buffer the handler fills with its response text.
 * @param reply_size  Size of @p reply in bytes.
 */
typedef void (*cmd_handler_t)(const char *args, char *reply, uint32_t reply_size);

/** One row of the command table. */
typedef struct
{
    const char   *name;    /* command word, e.g. "led"                    */
    cmd_handler_t handler; /* function to run                             */
    const char   *help;    /* one-line usage shown by a "help" command    */
} cmd_entry_t;

/** Output sink used for all replies (e.g. a UART write-string function). */
typedef void (*cmd_out_fn)(const char *str);

/**
 * @brief  Bind the dispatcher to a command table and an output function.
 * @param  table  Array of commands. Must outlive the dispatcher.
 * @param  count  Number of rows in @p table.
 * @param  out    Function used to emit replies. Must not be NULL.
 */
void cmd_dispatcher_init(const cmd_entry_t *table, uint32_t count, cmd_out_fn out);

/**
 * @brief  Feed one received byte into the line assembler.
 *
 *         Printable bytes are appended to the current line. A carriage return
 *         or newline ends the line and triggers a parse-and-dispatch. Backspace
 *         erases the last character. Call this once per byte drained from the
 *         UART ring buffer.
 */
void cmd_feed_char(uint8_t byte);

/**
 * @brief  Access the bound command table (used by a "help" handler to list
 *         the available commands). @p count_out receives the row count.
 */
const cmd_entry_t *cmd_get_table(uint32_t *count_out);

#ifdef __cplusplus
}
#endif

#endif /* CMD_DISPATCHER_H */
