/**
 ******************************************************************************
 * @file    main_Ex_5_2b_UART_Interrupt.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2b: Interrupt-driven UART receive
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 over the ST-LINK Virtual COM Port (PA2/PA3) -> USB -> PC
 * @output  PC serial terminal @ 115200 8N1   (status notes on SWV/ITM)
 *
 * @details Where 5.2a *blocked* on every received byte, this version lets the
 *          RX interrupt fill a ring buffer while the main loop stays free. To
 *          prove the CPU is not blocked, the loop prints a periodic "heartbeat"
 *          counter on the SWV/ITM console; that counter keeps advancing whether
 *          or not anything is typed in the terminal.
 *
 *          Received bytes are drained from the ring buffer and assembled into
 *          lines. A complete line (terminated by CR or LF) is handed to a tiny
 *          single-character command dispatcher — the same table-driven pattern
 *          introduced in Chapter 3, now fed by a real UART instead of a
 *          hard-coded array. This is the exact shape Chapter 6 builds on.
 *
 *          This build also registers an **RX callback** with the driver. The
 *          driver calls it from inside the USART2 interrupt for every byte it
 *          receives, which is how a driver hands control back to application
 *          code it was never compiled against. You met the pattern in
 *          *Chapter 3, Section 3.5*; this is the same idea running in interrupt
 *          context. The callback here does the only kind of work an ISR-context
 *          hook should do: it looks at one byte and sets a flag. The reaction
 *          happens later, in the main loop, where it is safe to print.
 ******************************************************************************
 */

#include "main.h"
#include "uart_driver_isr.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define UART_BAUD       (115200UL)
#define LINE_MAX        (32U)
#define HEARTBEAT_LOOPS (2000000UL)

/* ---- single-character command handlers --------------------------------- */
static void cmd_help(void)
{
    uart2_write_string("commands: h=help  p=ping  r=reset-counter\r\n");
}

static void cmd_ping(void)
{
    uart2_write_string("pong\r\n");
}

/* Shared with main via a flag so the handler stays trivial. */
static volatile int s_reset_counter = 0;
static void cmd_reset(void)
{
    s_reset_counter = 1;
    uart2_write_string("counter reset\r\n");
}

static void dispatch(char c)
{
    switch (c)
    {
        case 'h': cmd_help();  break;
        case 'p': cmd_ping();  break;
        case 'r': cmd_reset(); break;
        default:
            uart2_write_string("unknown command (try 'h')\r\n");
            break;
    }
}

/* =========================================================================
 * on_rx_byte — the application's RX callback, called from the USART2 ISR.
 *
 * Registered below with uart2_set_rx_callback(). The driver knows nothing
 * about this function; it only knows it holds a pointer of the right shape.
 *
 * Everything this runs inside is interrupt context, so it obeys the rule the
 * chapter keeps repeating: look, decide, set a flag, return. It does NOT
 * print, does NOT transmit, and does NOT touch the line buffer the main loop
 * owns. The single escape character it watches for is acted on in the loop.
 *
 * s_escape_seen is volatile because it is written here and read there.
 * ========================================================================= */
#define ESCAPE_CHAR   (0x1BU)   /* ESC */

static volatile int s_escape_seen = 0;

static void on_rx_byte(uint8_t byte)
{
    if (byte == ESCAPE_CHAR)
    {
        s_escape_seen = 1;
    }
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    const uart_config_t cfg = { .baud = UART_BAUD };
    if (uart2_init(&cfg) != UART_OK)
    {
        Error_Handler();
    }

    /* Hand the driver a function pointer into this file. From here on, every
     * byte the USART2 interrupt receives is offered to on_rx_byte() before
     * the main loop ever sees it. Passing NULL would unregister it again.  */
    uart2_set_rx_callback(on_rx_byte);

    uart2_write_string("\r\n5.2b interrupt UART ready. Type a command + Enter.\r\n");
    uart2_write_string("Press ESC at any time to see the callback fire.\r\n");
    cmd_help();

    char line[LINE_MAX];
    uint32_t line_len = 0U;
    uint32_t beats    = 0U;
    uint32_t spin     = 0U;

    while (1)
    {
        /* The callback set a flag from interrupt context; the reaction to it
         * belongs out here, where transmitting is safe.                     */
        if (s_escape_seen != 0)
        {
            s_escape_seen = 0;
            line_len = 0U;      /* ESC abandons whatever was half-typed */
            uart2_write_string("\r\n[callback] ESC seen in the ISR; line cleared\r\n");
        }

        /* Drain everything the ISR has queued since the last pass. */
        uint8_t ch;
        while (uart2_getchar(&ch) == 1)
        {
            if ((ch == '\r') || (ch == '\n'))
            {
                if (line_len > 0U)
                {
                    dispatch(line[0]);      /* first character is the command */
                    line_len = 0U;
                }
            }
            else if (ch == ESCAPE_CHAR)
            {
                /* Already handled by the callback path above; do not echo it. */
            }
            else if (line_len < (LINE_MAX - 1U))
            {
                line[line_len] = (char)ch;
                line_len++;
                (void)uart2_write_byte(ch);     /* echo so typing is visible */
            }
            else
            {
                line_len = 0U;      /* overflow — drop the line */
            }
        }

        /* Independent background work: a heartbeat that proves we never block.
         * A real product would do useful periodic work here.                 */
        if (++spin >= HEARTBEAT_LOOPS)
        {
            spin = 0U;
            if (s_reset_counter != 0)
            {
                s_reset_counter = 0;
                beats = 0U;
            }
            beats++;
            printf("[ITM] heartbeat %lu (RX never blocks main, overruns=%lu)\r\n",
                   (unsigned long)beats,
                   (unsigned long)uart2_rx_overruns());
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
