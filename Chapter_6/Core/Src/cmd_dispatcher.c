/**
 ******************************************************************************
 * @file    cmd_dispatcher.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.3: line assembler and command dispatcher
 *          (implementation)
 *
 * @details Where this sits in the data flow:
 *
 *              USART RX ISR --> ring buffer --> uart_getchar() (main loop)
 *                          --> cmd_feed_char() --> [Enter] --> dispatch_line()
 *                          --> handler --> reply --> out(...)
 *
 *          The interrupt half (byte arrives, gets pushed into the ring buffer)
 *          already happened before we get here. Everything in this file runs in
 *          the main loop, one byte at a time, so the line buffer is a plain
 *          fixed-size array with a length counter: it needs no `volatile` and
 *          no critical section, because the only tricky hand-off (ISR to
 *          thread) was handled safely inside the ring buffer.
 *
 *          Parsing model: bytes accumulate into `s_line` until a carriage
 *          return or newline arrives. At that point the buffer is
 *          null-terminated, the first whitespace-separated word is taken as the
 *          command, and the remainder (with leading spaces trimmed) is the
 *          argument string handed to the matched handler.
 ******************************************************************************
 */

#include "cmd_dispatcher.h"
#include <stddef.h>   /* NULL */

/* -------------------------------------------------------------------------
 * Module state (all private to this file).
 *
 * The dispatcher is bound once, at init, to a command table and an output
 * function; those three pointers are then read on every dispatched line.
 * ------------------------------------------------------------------------- */
static const cmd_entry_t *s_table = NULL;   /* caller's command table       */
static uint32_t           s_count = 0U;     /* number of rows in s_table    */
static cmd_out_fn         s_out   = NULL;   /* where replies are written    */

/* The line being assembled. `s_len` is how many bytes are currently held;
 * one slot is always kept free so a null terminator can be appended.        */
static char     s_line[CMD_LINE_MAX];
static uint32_t s_len = 0U;

/* ---- tiny string helpers (freestanding; no libc string.h dependency) --- */

/**
 * @brief  Compare two null-terminated strings for exact equality.
 *
 *         Equivalent to `strcmp(a, b) == 0`, but written out so the exercise
 *         pulls in no C library string functions - useful on a bare-metal
 *         target where you may not want the extra code, and to see exactly 
 *         what "match" means. Walks both strings in lockstep; the moment two 
 *         characters differ it returns 0 (not equal). If the loop ends, the 
 *         strings matched up to the first terminator, so they are equal only 
 *         if BOTH ended at the same place.
 *
 * @return 1 if the strings are identical, 0 otherwise.
 */
static int str_eq(const char *a, const char *b)
{
    /* Advance while both strings still have characters and they agree. */
    while ((*a != '\0') && (*b != '\0'))
    {
        if (*a != *b)
        {
            return 0;   /* mismatch found - definitely not equal */
        }
        a++;
        b++;
    }
    /* Equal only if we stopped because BOTH hit their terminator together. */
    return (*a == *b) ? 1 : 0;
}

/**
 * @brief  Copy a string into a fixed-size buffer, never overflowing it.
 *
 *         A bounded, always-null-terminated copy (similar in spirit to
 *         strlcpy). At most `dst_size - 1` characters are copied and a '\0' is
 *         always written, so `dst` is a valid C string even if `src` was
 *         longer than the buffer. NULL-safe on both the destination pointer
 *         and a zero size.
 *
 * @param  dst       Destination buffer.
 * @param  src       Source string (may be NULL - treated as empty).
 * @param  dst_size  Total size of @p dst in bytes, including the terminator.
 */
static void copy_str(char *dst, const char *src, uint32_t dst_size)
{
    uint32_t i = 0U;

    /* Nothing to write into - bail out safely. */
    if ((dst == NULL) || (dst_size == 0U))
    {
        return;
    }

    /* Copy until the source ends or we have filled all but the last slot
     * (that last slot is reserved for the '\0' below).                      */
    if (src != NULL)
    {
        while ((src[i] != '\0') && (i < (dst_size - 1U)))
        {
            dst[i] = src[i];
            i++;
        }
    }

    dst[i] = '\0';   /* always terminate, even on truncation or empty src */
}

/**
 * @brief  Bind the dispatcher to a command table and an output function.
 *
 *         Call once during setup. The table and output function must outlive
 *         the dispatcher (they are stored by pointer, not copied). Also clears
 *         any partially assembled line so parsing starts fresh.
 *
 * @param  table  Array of commands (name / handler / help). Must not be freed.
 * @param  count  Number of rows in @p table.
 * @param  out    Function used to emit reply text (e.g. a UART write-string).
 */
void cmd_dispatcher_init(const cmd_entry_t *table, uint32_t count, cmd_out_fn out)
{
    s_table = table;
    s_count = count;
    s_out   = out;
    s_len   = 0U;    /* discard anything half-typed before init */
}

/**
 * @brief  Expose the bound command table (used by a "help" handler to list
 *         the available commands without duplicating the table).
 *
 * @param  count_out  If not NULL, receives the row count.
 * @return Pointer to the command table (may be NULL if not yet bound).
 */
const cmd_entry_t *cmd_get_table(uint32_t *count_out)
{
    if (count_out != NULL)
    {
        *count_out = s_count;
    }
    return s_table;
}

/**
 * @brief  Parse the fully assembled line in @p s_line and run the matching
 *         handler. Called only once a line terminator has arrived.
 *
 *         Steps:
 *           1. Split the line into the command word (up to the first space)
 *              and the argument text (everything after the spaces).
 *           2. Look the command word up in the table.
 *           3. On a match, let the handler fill a reply buffer and send it;
 *              on no match, report an error. Nothing is sent for a blank line.
 */
static void dispatch_line(void)
{
    /* --- 1a. Find the end of the command word: scan up to the first space. */
    uint32_t i = 0U;
    while ((i < s_len) && (s_line[i] != ' '))
    {
        i++;
    }

    /* --- 1b. Copy just that word out as a null-terminated command string.
     * The count passed is (i + 1): i characters plus room for the '\0'.
     * The min-with-sizeof(cmd) guard keeps copy_str safe if the word is
     * somehow longer than the local buffer.                                 */
    char cmd[CMD_LINE_MAX];
    copy_str(cmd, s_line, (i < sizeof(cmd)) ? (i + 1U) : sizeof(cmd));

    /* --- 1c. Skip the run of spaces separating the command from its args. */
    while ((i < s_len) && (s_line[i] == ' '))
    {
        i++;
    }
    /* Whatever remains is the argument text (already null-terminated because
     * cmd_feed_char terminated the whole line before calling us).           */
    const char *args = &s_line[i];

    /* A line that was only spaces (or empty) has no command - do nothing. */
    if (cmd[0] == '\0')
    {
        return;
    }

    /* --- 2. Linear search of the command table for a name match. */
    for (uint32_t r = 0U; r < s_count; r++)
    {
        if ((s_table[r].name != NULL) && str_eq(cmd, s_table[r].name))
        {
            /* --- 3. Match: run the handler, which writes its response into
             * `reply`. We start it empty so a handler that chooses to print
             * nothing (e.g. one that calls s_out directly, like "help")
             * produces no stray output.                                     */
            char reply[96];
            reply[0] = '\0';

            if (s_table[r].handler != NULL)
            {
                s_table[r].handler(args, reply, sizeof(reply));
            }
            /* Only emit if the handler actually filled the reply buffer. */
            if ((s_out != NULL) && (reply[0] != '\0'))
            {
                s_out(reply);
            }
            return;   /* handled - stop searching */
        }
    }

    /* No table row matched the command word. */
    if (s_out != NULL)
    {
        s_out("error: unknown command (try 'help')\r\n");
    }
}

/**
 * @brief  Feed one received byte into the line assembler.
 *
 *         Call this once for every byte you drain from the UART ring buffer.
 *         Most bytes are simply appended to the current line; three are
 *         special:
 *           - CR ('\r') or LF ('\n') ends the line and dispatches it.
 *           - Backspace ('\b') or DEL (0x7F) erases the last character.
 *           - Non-printable control bytes are ignored.
 *
 * @param  byte  The received byte.
 */
void cmd_feed_char(uint8_t byte)
{
    const char c = (char)byte;

    /* --- Line terminator: finish the line and act on it. --------------- */
    if ((c == '\r') || (c == '\n'))
    {
        /* Terminate the assembled line. There is always room for this '\0'
         * because the append branch below never fills the final slot.       */
        s_line[s_len] = '\0';
        dispatch_line();
        s_len = 0U;          /* reset for the next line */
        return;
    }

    /* --- Editing: backspace / DEL removes the most recent character. ---- */
    if ((c == '\b') || (byte == 0x7FU))
    {
        if (s_len > 0U)
        {
            s_len--;         /* just drop the last byte; no redraw needed here */
        }
        return;
    }

    /* --- Printable byte: append it, but keep one slot free for the '\0'.
     * Anything from 0x20 (space) up is treated as text; control codes below
     * that (other than the ones handled above) are silently ignored. If the
     * line is already full we drop the byte rather than overflow the buffer. */
    if ((byte >= 0x20U) && (s_len < (CMD_LINE_MAX - 1U)))
    {
        s_line[s_len] = c;
        s_len++;
    }
}
