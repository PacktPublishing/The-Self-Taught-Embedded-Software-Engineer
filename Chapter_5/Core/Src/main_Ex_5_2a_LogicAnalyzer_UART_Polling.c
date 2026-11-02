/**
 ******************************************************************************
 * @file    main_Ex_5_2a_LogicAnalyzer_UART_Polling.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2a (logic-analyzer variant): UART polling
 *          on a header-native port you can actually probe.
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART6 brought out to the ST morpho header:
 *              PC6 = USART6_TX  (AF8)  <- clip the logic-analyzer probe here
 *              PC7 = USART6_RX  (AF8)
 *          No ST-LINK VCP is attached to USART6. To also see text on a PC,
 *          connect a USB-TTL adapter (its RX to PC6, GND to board GND).
 * @output  Logic analyzer on PC6 (UART decoder, 115200 8N1). Status notes also
 *          go to the SWV ITM Data Console.
 *
 * @details This is the twin of main_Ex_5_2a_UART_Polling.c. That version uses
 *          USART2 on the ST-LINK Virtual COM Port (PA2/PA3), which is ideal for
 *          a PC terminal but, on this board, is NOT routed to a header pin by
 *          default: solder bridges SB13/SB14 (ON) tie PA2/PA3 to the ST-LINK,
 *          while SB62/SB63 (OFF) leave the morpho/Arduino PA2/PA3 pins
 *          disconnected (UM1724). Trying to capture USART2 on the header
 *          therefore shows a flat idle-high line.
 *
 *          This variant instead drives USART6 on PC6/PC7, which go straight to
 *          the morpho header with no bridges in the way, so a probe on PC6 sees
 *          the real waveform. To make capture trivial, the loop transmits a
 *          repeating line that includes a run of 'U' (0x55 = 0b01010101) — a
 *          perfect alternating pattern for measuring bit width (8.68 us @
 *          115200). Any bytes received on PC7 are echoed, so a USB-TTL adapter
 *          still works.
 *
 * @note    Build ONE main at a time. Keep either this file or
 *          main_Ex_5_2a_UART_Polling.c in the build and exclude the other
 *          (right-click -> Resource Configurations -> Exclude from Build),
 *          since both define main().
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

static void crude_delay(volatile uint32_t loops)
{
    while (loops > 0UL)
    {
        loops--;
    }
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);   /* printf -> SWV/ITM, unbuffered */

    const uart_config_t cfg = { .baud = UART_BAUD };
    if (usart6_init(&cfg) != UART_OK)
    {
        Error_Handler();
    }

    printf("\r\n[ITM] USART6 up at %lu 8N1 on PC6/PC7.\r\n",
           (unsigned long)UART_BAUD);
    printf("[ITM] Probe PC6 (morpho) with the logic analyzer; UART decoder 8N1.\r\n");

    /* Continuous, decodable traffic so the analyzer always has a signal to
     * capture, plus an easily-measured 'UUUU' run for bit-width checks.       */
    while (1)
    {
    	usart6_write_string("Hello from the Nucleo-F446RE!\r\n");
        usart6_write_string("Chapter 5 - Exercise 5.2a LA test: UUUUUUUU 0123456789\r\n");

        /* Optional: echo anything received (only if a USB-TTL drives PC7). */
        if (usart6_rx_ready() == 1)
        {
            uint8_t ch = 0U;
            if (usart6_read_byte(&ch) == UART_OK)
            {
                (void)usart6_write_byte(ch);
            }
        }

        crude_delay(400000UL);   /* short gap between lines; tune as desired */
    }
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
