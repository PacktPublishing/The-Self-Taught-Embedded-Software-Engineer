/**
 ******************************************************************************
 * @file    main_Ex_7_3b_UART_DMA_DoubleBuffer.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - Exercise 7.3b: double-buffered UART receive with the
 *          DMA half-transfer interrupt, tested against the Chapter 6 stall
 *          [VCP / USART2 build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 PA2/PA3 -> ST-Link VCP | B1 = PC13 (EXTI) cycles the stall
 * @output  Serial terminal at 115200 8N1 for the echo, SWV ITM Data Console
 *          for the once-a-second receiver statistics
 *
 * @details DMA1 Stream 5 runs in circular mode over a 256-byte buffer that
 *          the code treats as two 128-byte halves:
 *
 *              HT  (half transfer)   first half full, DMA now writing the second
 *              TC  (transfer done)   second half full, DMA wrapped to the first
 *              IDLE (USART)          the sender paused part-way through a half
 *
 *          While the DMA fills one half the main loop reads the other. HT and
 *          TC carry the bulk of a long paste; IDLE delivers the short tail of
 *          a typed line, which on its own would never fill a half.
 *
 *          The point of the exercise is the stall. Chapter 6 measured a
 *          blocking BME280 read at 11.2 ms and showed that it overflowed the
 *          63-byte interrupt ring buffer, because about 129 bytes arrive in
 *          that time at 115200 baud. Press B1 to make the main loop stall
 *          for 11 ms on every pass, then paste a long block of text. The
 *          overflow counter stays at zero: the DMA keeps writing while the
 *          CPU is stuck, and 256 bytes of buffer is 22 ms of slack. Press B1
 *          again for a 30 ms stall, which is longer than the buffer, and the
 *          counter climbs. The CPU is just as stuck in both cases; what
 *          changed is who is responsible for catching the bytes.
 *
 *          Typed lines are echoed back by DMA when the transmitter is free.
 *          A single transmit buffer cannot queue, so echoes are skipped and
 *          counted while one is still going out. Statistics go to SWV ITM so
 *          that printing them never competes with the data path under test.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "exti_driver.h"
#include "uart_driver_dma.h"
#include "cycle_counter.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define RX_RING_LEN      (256U)          /* two 128-byte halves, 22 ms at 115200 */
#define LINE_MAX         (80U)
#define STATS_PERIOD_US  (1000000UL)
#define BTN_PORT         GPIOC
#define BTN_PIN          (13U)            /* B1, active low                    */
#define BUTTON_LOCKOUT_US (250000UL)      /* one accepted press per 250 ms     */

/* The stall settings B1 cycles through, in milliseconds. 11 ms is the
 * blocking BME280 read from Chapter 6; 30 ms is longer than the buffer. */
static const uint32_t k_stall_ms[] = { 0U, 11U, 30U };
#define STALL_STEPS      (sizeof(k_stall_ms) / sizeof(k_stall_ms[0]))

static uint8_t  s_rx_ring[RX_RING_LEN];     /* written by DMA, read by the loop */
static char     s_line[LINE_MAX];
static uint32_t s_line_len = 0U;
static uint8_t  s_echo[LINE_MAX + 2U];      /* stays valid while DMA sends it   */
static uint32_t s_lines = 0U;
static uint32_t s_echo_skipped = 0U;
static uint32_t s_last_events = 0U;

static volatile uint8_t s_button_flag = 0U;

static void on_button_press(void)
{
    s_button_flag = 1U;                     /* ISR: one store, nothing else */
}

/* Called from uart2_dma_rx_service(), in thread context. */
static void on_rx_chunk(const uint8_t *data, uint32_t len, uint32_t events)
{
    s_last_events |= events;

    for (uint32_t i = 0U; i < len; i++)
    {
        const char c = (char)data[i];
        if ((c == '\r') || (c == '\n'))
        {
            if (s_line_len == 0U)
            {
                continue;                   /* ignore the LF of a CR LF pair */
            }
            s_lines++;
            if (uart2_dma_tx_busy() == 0)
            {
                (void)memcpy(s_echo, s_line, s_line_len);
                s_echo[s_line_len]      = (uint8_t)'\r';
                s_echo[s_line_len + 1U] = (uint8_t)'\n';
                (void)uart2_dma_send(s_echo, (uint16_t)(s_line_len + 2U));
            }
            else
            {
                s_echo_skipped++;
            }
            s_line_len = 0U;
        }
        else if (s_line_len < LINE_MAX)
        {
            s_line[s_line_len] = c;
            s_line_len++;
        }
        else
        {
            /* line longer than LINE_MAX: the excess is not echoed */
        }
    }
}

static void print_stats(uint32_t stall_ms)
{
    uart_rx_stats_t st;
    uart2_dma_rx_stats(&st);

    printf("[ITM] rx %lu B | HT %lu TC %lu IDLE %lu | overflow %lu | lines %lu "
           "echo skipped %lu | stall %lu ms%s%s%s\r\n",
           (unsigned long)st.bytes, (unsigned long)st.half_events,
           (unsigned long)st.full_events, (unsigned long)st.idle_events,
           (unsigned long)st.overflows, (unsigned long)s_lines,
           (unsigned long)s_echo_skipped, (unsigned long)stall_ms,
           ((s_last_events & UART_RX_EVT_HALF) != 0U) ? " [HT]" : "",
           ((s_last_events & UART_RX_EVT_FULL) != 0U) ? " [TC]" : "",
           ((s_last_events & UART_RX_EVT_IDLE) != 0U) ? " [IDLE]" : "");
    s_last_events = 0U;
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    cycle_counter_init();
    exti_pc13_init(on_button_press);

    const uart_config_t uart = { .baud = 115200U };
    if (uart2_init(&uart) != UART_OK)
    {
        Error_Handler();
    }

    (void)uart2_write_string("\r\nChapter 7 - Exercise 7.3b: double-buffered DMA receive (VCP)\r\n");
    (void)uart2_write_string("Type a line, or paste a long block. B1 cycles the main-loop stall: 0, 11, 30 ms.\r\n");
    printf("[ITM] Exercise 7.3b: %u-byte ring, %u-byte halves\r\n",
           (unsigned)RX_RING_LEN, (unsigned)(RX_RING_LEN / 2U));

    if (uart2_dma_rx_start_circular(s_rx_ring, RX_RING_LEN) != UART_OK)
    {
        Error_Handler();
    }

    uint32_t stall_index = 0U;
    uint32_t last_stats  = cycle_counter_now();
    uint32_t last_press  = cycle_counter_now();

    while (1)
    {
        /* Deliver whatever the DMA wrote since the last pass. */
        (void)uart2_dma_rx_service(on_rx_chunk);

        if (s_button_flag != 0U)
        {
            s_button_flag = 0U;

            /* A contact bounces, and the EXTI driver reports every falling
             * edge, so one press can arrive as two or three. Accept a press
             * only if the lockout has expired AND the pin still reads low:
             * the edge says something happened, the level says it is a real
             * press and not release chatter. Chapter 6's five-state machine
             * is the full answer when short and long presses matter. */
            const uint8_t still_down =
                (gpio_read_pin(BTN_PORT, BTN_PIN) == GP_LOW) ? 1U : 0U;
            if ((still_down != 0U) &&
                (cycle_counter_elapsed_us(last_press) >= BUTTON_LOCKOUT_US))
            {
                last_press  = cycle_counter_now();
                stall_index = (stall_index + 1U) % STALL_STEPS;
                printf("[ITM] main-loop stall now %lu ms\r\n",
                       (unsigned long)k_stall_ms[stall_index]);
            }
        }

        if (cycle_counter_elapsed_us(last_stats) >= STATS_PERIOD_US)
        {
            last_stats = cycle_counter_now();
            print_stats(k_stall_ms[stall_index]);
        }

        /* The deliberate stall: the CPU is unavailable, exactly as it was
         * during the blocking sensor read in Chapter 6. */
        if (k_stall_ms[stall_index] != 0U)
        {
            cycle_counter_delay_us(k_stall_ms[stall_index] * 1000UL);
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
