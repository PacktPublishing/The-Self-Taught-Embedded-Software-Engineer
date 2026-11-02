/**
 ******************************************************************************
 * @file    main_Ex_5_2b_LogicAnalyzer_UART_Interrupt.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2b (logic-analyzer variant): interrupt-driven
 *          UART on a header-native port you can actually probe.
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART6 brought out to the ST morpho header:
 *              PC6 = USART6_TX  (AF8)  <- clip the logic-analyzer probe here
 *              PC7 = USART6_RX  (AF8)
 *          No ST-LINK VCP is attached to USART6. To also type into it, connect
 *          a USB-TTL adapter (its TX to PC7, RX to PC6, GND to board GND).
 * @output  Logic analyzer on PC6 (UART decoder, 115200 8N1); status on SWV/ITM.
 *
 * @details Twin of main_Ex_5_2b_UART_Interrupt.c. That version uses USART2 on
 *          the ST-LINK Virtual COM Port (PA2/PA3), which is great for a PC
 *          terminal but, on this board, is NOT routed to a header pin by
 *          default (SB13/SB14 ON, SB62/SB63 OFF — UM1724), so a logic-analyzer
 *          probe on the PA2/PA3 header pin only sees idle-high.
 *
 *          This variant drives USART6 (PC6/PC7), which reaches the morpho
 *          header with no bridges in the way. Receive is interrupt-driven into
 *          a ring buffer exactly as in the USART2 version; the main loop drains
 *          it without ever blocking, dispatches single-character commands, and
 *          periodically transmits a test line so the analyzer always has framed
 *          traffic to capture (including a 'UUUU' run, 0x55, for bit timing).
 *
 * @note    Build ONE main at a time: keep either this file or
 *          main_Ex_5_2b_UART_Interrupt.c in the build and exclude the other
 *          (right-click -> Resource Configurations -> Exclude from Build),
 *          since both define main().
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
#define HEARTBEAT_LOOPS (1500000UL)

/* ---- single-character command handlers (output on USART6) -------------- */
static void cmd_help(void)
{
    usart6_write_string("commands: h=help  p=ping  r=reset-counter\r\n");
}
static void cmd_ping(void)
{
    usart6_write_string("pong\r\n");
}
static volatile int s_reset_counter = 0;
static void cmd_reset(void)
{
    s_reset_counter = 1;
    usart6_write_string("counter reset\r\n");
}
static void dispatch(char c)
{
    switch (c)
    {
        case 'h': cmd_help();  break;
        case 'p': cmd_ping();  break;
        case 'r': cmd_reset(); break;
        default:
            usart6_write_string("unknown command (try 'h')\r\n");
            break;
    }
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    const uart_config_t cfg = { .baud = UART_BAUD };
    if (usart6_init(&cfg) != UART_OK)
    {
        Error_Handler();
    }

    printf("\r\n[ITM] USART6 (PC6/PC7) interrupt RX ready. Probe PC6.\r\n");
    usart6_write_string("\r\n5.2b interrupt UART on USART6/PC6. Type a command + Enter.\r\n");
    cmd_help();

    uint32_t beats = 0U;
    uint32_t spin  = 0U;

    while (1)
    {
        /* Drain everything the ISR queued; first char of a line is the command. */
        uint8_t ch;
        while (usart6_getchar(&ch) == 1)
        {
            if ((ch == '\r') || (ch == '\n'))
            {
                /* command already handled on first char below */
            }
            else
            {
                (void)usart6_write_byte(ch);   /* echo */
                dispatch((char)ch);
            }
        }

        /* Background work + periodic test traffic so the analyzer always has a
         * signal to capture. Proves RX never blocks the main loop.            */
        if (++spin >= HEARTBEAT_LOOPS)
        {
            spin = 0U;
            if (s_reset_counter != 0)
            {
                s_reset_counter = 0;
                beats = 0U;
            }
            beats++;
            usart6_write_string("LA test: UUUUUUUU 0123456789\r\n");
            printf("[ITM] heartbeat %lu (RX never blocks main)\r\n",
                   (unsigned long)beats);
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
