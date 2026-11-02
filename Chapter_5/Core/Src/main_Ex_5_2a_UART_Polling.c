/**
 ******************************************************************************
 * @file    main_Ex_5_2a_UART_Polling.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2a: UART driver in polling mode
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 over the ST-LINK Virtual COM Port (PA2/PA3) -> USB -> PC
 * @output  PC serial terminal @ 115200 8N1 (and SWV/ITM for status notes)
 *
 * @details First contact with the outside world. The program:
 *            1. brings USART2 up at 115200 8N1 (polling),
 *            2. sends a greeting and a short menu to the PC terminal, and
 *            3. enters a blocking echo loop: every character typed in the
 *               terminal is read back and echoed, with two single-key
 *               "commands" ('?' prints the menu, 'c' clears with newlines)
 *               to show that received bytes can drive behaviour.
 *
 *          Everything here is polling: each read/write waits on a status flag.
 *          Open the terminal (e.g. the STM32CubeIDE Serial Terminal, or any
 *          PC terminal on the ST-LINK COM port) at 115200 8N1 to interact.
 *
 *          Verify on the logic analyzer by probing PA2 (TX): at 115200 each
 *          bit is ~8.68 us wide; decode with the UART decoder (8N1).
 ******************************************************************************
 */

#include "main.h"
#include "uart_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define UART_BAUD   (115200UL)

static void print_menu(void)
{
    uart2_write_string("\r\n=== Chapter 5 - Exercise 5.2a: UART polling echo ===\r\n");
    uart2_write_string("Type characters; they are echoed back.\r\n");
    uart2_write_string("Commands:  '?' = show this menu   'c' = clear\r\n\r\n");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);   /* printf -> SWV/ITM, unbuffered */

    const uart_config_t cfg = { .baud = UART_BAUD };
    if (uart2_init(&cfg) != UART_OK)
    {
        Error_Handler();
    }

    /* Status note on the always-available SWV channel. */
    printf("\r\n[ITM] USART2 up at %lu 8N1. Open the VCP terminal.\r\n",
           (unsigned long)UART_BAUD);

    /* Greeting goes to the PC terminal over the wire. */
    uart2_write_string("Hello from the Nucleo-F446RE!\r\n");
    print_menu();

    /* Blocking echo loop. */
    while (1)
    {
        uint8_t ch = 0U;
        if (uart2_read_byte(&ch) != UART_OK)
        {
        	continue;   /* timeout with nothing received — keep waiting */
        }

        switch (ch)
        {
            case '?':
                print_menu();
                break;

            case 'c':
                uart2_write_string("\r\n\r\n\r\n");
                break;

            default:
                /* Plain echo. Translate a bare CR into CR+LF so terminals
                 * that send only '\r' on Enter still advance a line.        */
                (void)uart2_write_byte(ch);
                if (ch == '\r')
                {
                    (void)uart2_write_byte('\n');
                }
                break;
        }
    }

    /* not reached */
}

/* =========================================================================
 * Error_Handler — shared trap (declared in main.h)
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
        /* spin so a debugger can inspect the state */
    }
}
