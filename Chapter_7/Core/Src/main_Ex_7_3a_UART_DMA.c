/**
 ******************************************************************************
 * @file    main_Ex_7_3a_UART_DMA.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - Exercise 7.3a: the Chapter 5 UART driver moved onto
 *          DMA for transmit and receive, plus one memory-to-memory copy
 *          [VCP / USART2 build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 PA2/PA3 -> ST-Link VCP | PA6 = CPU-busy marker (optional
 *          logic-analyzer probe)
 * @output  Serial terminal on the ST-Link VCP at 115200 8N1, plus the SWV
 *          ITM Data Console for a heartbeat while the receive is pending
 *
 * @details Three measurements, in order, each covering one DMA direction:
 *
 *          1. Memory to peripheral. Send the same 512-byte message twice,
 *             first with the polled uart2_write() from Chapter 5, then with
 *             uart2_dma_send(). The polled call holds the CPU for the whole
 *             transmission, about 512 x 86.8 us = 44.4 ms. The DMA call
 *             returns in microseconds and the main loop counts how many
 *             passes it gets through while the DMA does the work. PA6 is high
 *             only while the CPU is stuck inside the send call, so on a logic
 *             analyzer the two marker pulses tell the same story.
 *
 *          2. Memory to memory. Copy 4 KB with memcpy() and then with DMA2.
 *             DMA is not automatically faster here, and the exercise prints
 *             both numbers so you can see it. What DMA buys is that the CPU
 *             is free during the copy.
 *
 *          3. Peripheral to memory. Arm a DMA receive of exactly 8 bytes and
 *             keep the CPU busy counting. The block completes when the eighth
 *             byte arrives, with no per-byte interrupt at all. The fixed
 *             length is also the weakness: a 5-character command never
 *             completes the block. Exercise 7.3b removes that limit.
 *
 *          The DMA streams used (RM0390 Tables 28 and 29):
 *              USART2_TX  DMA1 Stream 6 ch4 | USART2_RX  DMA1 Stream 5 ch4
 *              mem-to-mem DMA2 Stream 7 (only DMA2 can do it)
 *
 *          How the time is measured: every figure comes from the DWT cycle
 *          counter (cycle_counter.c), which counts core clock cycles at
 *          16 MHz, so 16 cycles = 1 us. Reading it is a single register load,
 *          so taking a timestamp does not disturb the transfer being timed.
 *
 *          "Free loop passes" is the exercise's stand-in for useful work: a
 *          counter incremented in the loop that waits for the DMA. The bigger
 *          the number, the more of the transfer the CPU had to itself. The
 *          absolute value depends on what each pass does (some passes also
 *          read a DMA register), so compare passes within one test, not
 *          across tests.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "dma_driver.h"
#include "uart_driver_dma.h"
#include "cycle_counter.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define MARK_PORT        GPIOA
#define MARK_PIN         (6U)
#define TX_MSG_LEN       (512U)
#define TX_LINE_LEN      (64U)
#define RX_BLOCK_LEN     (8U)
#define M2M_WORDS        (1024U)            /* 4 KB */
#define M2M_STREAM       DMA2_Stream7
#define M2M_POLL_BUDGET  (1000000UL)        /* gives up if TCIF never sets */
#define HEARTBEAT_US     (1000000UL)        /* one ITM line per second     */

/* Everything the DMA touches has static storage duration, so it still exists
 * after the function that started the transfer has returned. */
static uint8_t  s_tx_msg[TX_MSG_LEN];       /* source of both 512-byte sends */
static uint8_t  s_rx_block[RX_BLOCK_LEN];   /* destination of the 8-byte receive */
static char     s_reply[96];                /* sent by DMA after receive_block() returns */
static uint32_t s_m2m_src[M2M_WORDS];
static uint32_t s_m2m_dst[M2M_WORDS];

/* =========================================================================
 * marker_init
 *
 * PA6 as a push-pull output, starting low. The code drives it high only
 * while the CPU is blocked inside a send call, so a logic analyzer on PA6
 * shows CPU busy time directly: a 44 ms pulse for the polled send and a
 * pulse tens of microseconds long for the DMA send. Same pin and role as the
 * Chapter 6 marker, so no rewiring is needed.
 * ========================================================================= */
static void marker_init(void)
{
    const gpio_config_t mark = {
        .pin = MARK_PIN, .mode = GP_MODE_OUTPUT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    gpio_init(MARK_PORT, &mark);
    gpio_write_pin(MARK_PORT, MARK_PIN, GP_LOW);
}

/* =========================================================================
 * build_message
 *
 * Fills s_tx_msg with eight 64-byte lines of printable text, each ending in
 * CR LF: 62 letters (A to Z, repeating) plus the two line-end bytes. The
 * content does not matter to the DMA; it is printable only so the terminal
 * shows two identical blocks and you can see both sends arrived intact.
 * ========================================================================= */
static void build_message(void)
{
    for (uint32_t i = 0U; i < TX_MSG_LEN; i++)
    {
        const uint32_t col = i % TX_LINE_LEN;     /* position within the line */
        if (col == (TX_LINE_LEN - 2U))
        {
            s_tx_msg[i] = (uint8_t)'\r';
        }
        else if (col == (TX_LINE_LEN - 1U))
        {
            s_tx_msg[i] = (uint8_t)'\n';
        }
        else
        {
            s_tx_msg[i] = (uint8_t)('A' + (col % 26U));
        }
    }
}

/* =========================================================================
 * report
 *
 * Prints a status line on the serial terminal with the polled Chapter 5
 * call. The driver's polled calls first wait for any DMA transmit still in
 * progress, so a report can follow a DMA send without the two outputs
 * interleaving on the wire.
 * ========================================================================= */
static void report(const char *text)
{
    (void)uart2_write_string(text);
}

/* =========================================================================
 * compare_transmit  (measurement 1: memory to peripheral)
 *
 * Sends s_tx_msg twice and reports, for each, how long the CPU was busy:
 *   polled  uart2_write() feeds DR one byte at a time and returns only when
 *           the last byte has left the shift register, so CPU busy time is
 *           the whole transmission, about 44.4 ms at 115200 baud.
 *   DMA     uart2_dma_send() programs DMA1 Stream 6 and returns. CPU busy
 *           time is just that setup, measured in cycles. The loop that
 *           follows counts free passes until the DMA reports completion.
 *
 * The "transfer took" figure ends when the DMA has written the last byte
 * into DR, about one to two byte times before that byte finishes on the
 * wire, so it reads slightly shorter than the polled figure even though
 * the line speed is identical.
 * ========================================================================= */
static void compare_transmit(void)
{
    char line[96];

    /* Polled send: PA6 high for as long as the CPU is stuck in the call. */
    report("\r\n[1] Sending 512 bytes with the polled Chapter 5 call...\r\n");
    gpio_write_pin(MARK_PORT, MARK_PIN, GP_HIGH);
    const uint32_t t0 = cycle_counter_now();
    (void)uart2_write(s_tx_msg, TX_MSG_LEN);
    const uint32_t blocking_us = cycle_counter_elapsed_us(t0);
    gpio_write_pin(MARK_PORT, MARK_PIN, GP_LOW);

    /* DMA send: the same marker brackets only the setup call, so the pulse
     * shrinks from milliseconds to microseconds. */
    report("\r\n[1] Sending the same 512 bytes with DMA...\r\n");
    gpio_write_pin(MARK_PORT, MARK_PIN, GP_HIGH);
    const uint32_t t1 = cycle_counter_now();
    const uart_status_t st = uart2_dma_send(s_tx_msg, TX_MSG_LEN);
    const uint32_t start_cycles = cycle_counter_now() - t1;   /* setup cost only */
    gpio_write_pin(MARK_PORT, MARK_PIN, GP_LOW);

    /* The transfer is now running in hardware. The DMA completion callback
     * (in uart_driver_dma.c) clears the busy flag this loop polls. */
    uint32_t free_loops = 0U;
    while (uart2_dma_tx_busy() != 0)
    {
        free_loops++;                 /* the CPU's own work would go here */
    }
    const uint32_t dma_total_us = cycle_counter_elapsed_us(t1);

    /* The polled report() calls below are safe: the DMA send has finished. */
    (void)snprintf(line, sizeof(line), "\r\n    polled : CPU busy %lu us\r\n",
                   (unsigned long)blocking_us);
    report(line);
    (void)snprintf(line, sizeof(line), "    DMA    : CPU busy %lu cycles (%lu us), status %d\r\n",
                   (unsigned long)start_cycles,
                   (unsigned long)(start_cycles / CYCLES_PER_US), (int)st);
    report(line);
    (void)snprintf(line, sizeof(line), "             transfer took %lu us, CPU free for %lu loop passes\r\n",
                   (unsigned long)dma_total_us, (unsigned long)free_loops);
    report(line);
}

/* =========================================================================
 * compare_copy  (measurement 2: memory to memory)
 *
 * Copies 4 KB from s_m2m_src to s_m2m_dst twice, first with memcpy() and
 * then with DMA2 Stream 7, and reports the cycles each took. The DMA copy
 * is then checked word for word against the source.
 *
 * The stream is set up without a callback and polled with
 * dma_stream_poll_events(), so no interrupt latency enters the timing.
 * memcpy() keeps the CPU busy for the whole copy; the DMA copy leaves it
 * free, which is what the free-pass count shows. Which one finishes first
 * depends on the library's memcpy() and on bus contention, so read the two
 * cycle counts rather than assuming.
 * ========================================================================= */
static void compare_copy(void)
{
    char line[96];

    /* A pattern that differs in every byte, so a partial or shifted copy
     * cannot pass the memcmp() check by accident. */
    for (uint32_t i = 0U; i < M2M_WORDS; i++)
    {
        s_m2m_src[i] = (i * 0x01010101UL) ^ 0xA5A5A5A5UL;
    }

    /* CPU copy. The destination is cleared first so both copies start from
     * the same state. */
    (void)memset(s_m2m_dst, 0, sizeof(s_m2m_dst));
    const uint32_t t0 = cycle_counter_now();
    (void)memcpy(s_m2m_dst, s_m2m_src, sizeof(s_m2m_dst));
    const uint32_t cpu_cycles = cycle_counter_now() - t0;

    /* DMA copy. 32-bit items on both sides, both addresses increment, one
     * pass (not circular). The driver turns the FIFO on for mem-to-mem,
     * because direct mode is not allowed there. */
    (void)memset(s_m2m_dst, 0, sizeof(s_m2m_dst));
    const dma_config_t m2m = {
        .stream = M2M_STREAM, .channel = 0U,
        .dir = DMA_DIR_MEM_TO_MEM,
        .psize = DMA_SIZE_32BIT, .msize = DMA_SIZE_32BIT,
        .pinc = 1U, .minc = 1U, .circular = 0U,
        .priority = DMA_PRIO_LOW,
        .irq_half = 0U, .irq_complete = 0U,
        .callback = NULL                  /* polled, to keep the timing simple */
    };
    (void)dma_stream_init(&m2m);

    uint32_t free_loops = 0U;
    uint32_t budget = M2M_POLL_BUDGET;
    const uint32_t t1 = cycle_counter_now();
    /* In mem-to-mem mode PAR is the source and M0AR the destination. The
     * count is in 32-bit items: 1024 words = 4096 bytes. Unlike a peripheral
     * transfer, a mem-to-mem stream starts moving data the moment EN is set. */
    (void)dma_stream_start(M2M_STREAM, (uint32_t)s_m2m_src, (uint32_t)s_m2m_dst,
                           (uint16_t)M2M_WORDS);
    /* Poll TCIF without clearing it (clear = 0), so the loop sees it stay
     * set. The budget stops the loop if the transfer never completes. */
    while (((dma_stream_poll_events(M2M_STREAM, 0U) & DMA_EVT_COMPLETE) == 0U) &&
           (budget != 0UL))
    {
        free_loops++;
        budget--;
    }
    const uint32_t dma_cycles = cycle_counter_now() - t1;
    (void)dma_stream_poll_events(M2M_STREAM, 1U);   /* now clear the flags */

    /* Trust, but verify: the copy is only a result if it is correct. */
    const int same = (memcmp(s_m2m_src, s_m2m_dst, sizeof(s_m2m_dst)) == 0) ? 1 : 0;

    report("\r\n[2] Copying 4096 bytes in SRAM\r\n");
    (void)snprintf(line, sizeof(line), "    memcpy : %lu cycles (%lu us), CPU busy throughout\r\n",
                   (unsigned long)cpu_cycles, (unsigned long)(cpu_cycles / CYCLES_PER_US));
    report(line);
    (void)snprintf(line, sizeof(line), "    DMA2   : %lu cycles (%lu us), CPU free for %lu loop passes\r\n",
                   (unsigned long)dma_cycles, (unsigned long)(dma_cycles / CYCLES_PER_US),
                   (unsigned long)free_loops);
    report(line);
    report(same ? "    destination verified against source\r\n"
                : "    COPY MISMATCH\r\n");
}

/* =========================================================================
 * receive_block  (measurement 3: peripheral to memory)
 *
 * Arms a DMA receive of exactly RX_BLOCK_LEN bytes into s_rx_block, then
 * keeps the CPU counting free passes until the block is full. Every byte
 * typed is moved from USART2->DR to the buffer by DMA1 Stream 5 with no
 * CPU involvement; the only interrupt is the one at the end of the block.
 *
 * While waiting, a heartbeat goes to the SWV console once a second, not to
 * the UART, so it can never mix with the bytes being received.
 *
 * When the block is full, the bytes are echoed back (non-printable ones as
 * '.') in a reply sent by DMA. main() calls this function forever, so you
 * can type another 8 characters straight away.
 * ========================================================================= */
static void receive_block(void)
{
    /* Polled report: also waits for the previous pass's DMA reply to finish
     * before the receive is re-armed. */
    report("\r\n[3] Type 8 characters (they are not echoed). The CPU keeps counting.\r\n");
    if (uart2_dma_receive(s_rx_block, RX_BLOCK_LEN) != UART_OK)
    {
        report("    could not start the DMA receive\r\n");
        return;
    }

    /* The receive is armed. Nothing happens on the CPU side per byte: the
     * loop only watches the done flag that the DMA completion callback sets
     * after the eighth byte. */
    uint32_t free_loops = 0U;
    uint32_t last_beat = cycle_counter_now();
    while (uart2_dma_rx_done() == 0)
    {
        free_loops++;
        if (cycle_counter_elapsed_us(last_beat) >= HEARTBEAT_US)
        {
            last_beat = cycle_counter_now();
            printf("[ITM] receive pending, CPU free loop passes so far: %lu\r\n",
                   (unsigned long)free_loops);
        }
    }

    /* Make the received bytes safe to print: keep printable ASCII (0x20 to
     * 0x7E), replace anything else, such as CR from the Enter key, with '.'. */
    char shown[RX_BLOCK_LEN + 1U];
    for (uint32_t i = 0U; i < RX_BLOCK_LEN; i++)
    {
        const uint8_t c = s_rx_block[i];
        shown[i] = ((c >= 0x20U) && (c < 0x7FU)) ? (char)c : '.';
    }
    shown[RX_BLOCK_LEN] = '\0';

    /* The reply is built in s_reply, not in a local array, because the DMA
     * keeps reading it after this function returns. A local buffer would be
     * gone (and reused by the next call's stack frame) before the DMA had
     * sent it. */
    const int n = snprintf(s_reply, sizeof(s_reply),
                           "    DMA filled the block: [%s] after %lu free loop passes\r\n",
                           shown, (unsigned long)free_loops);
    if (n > 0)
    {
        (void)uart2_dma_send((const uint8_t *)s_reply, (uint16_t)n);
    }
}

/* =========================================================================
 * main
 *
 * Brings up the measurement tools (cycle counter, PA6 marker), builds the
 * test message, and initializes USART2 at 115200 baud. uart2_init() also
 * configures the transmit DMA stream, so uart2_dma_send() is ready to use.
 * The two one-shot measurements run once; the receive test then repeats
 * forever.
 * ========================================================================= */
int main(void)
{
    /* Unbuffered stdout: each ITM printf appears at once, and the C library
     * does not allocate a buffer on the heap for it. */
    setvbuf(stdout, NULL, _IONBF, 0);

    cycle_counter_init();     /* start DWT->CYCCNT before anything is timed */
    marker_init();
    build_message();

    const uart_config_t uart = { .baud = 115200U };
    if (uart2_init(&uart) != UART_OK)
    {
        Error_Handler();
    }

    report("\r\nChapter 7 - Exercise 7.3a: USART2 with DMA (VCP)\r\n");
    printf("[ITM] Exercise 7.3a running; results are on the serial terminal\r\n");

    compare_transmit();       /* measurement 1, once */
    compare_copy();           /* measurement 2, once */

    while (1)
    {
        receive_block();      /* measurement 3, every 8 characters typed */
    }
}

/* =========================================================================
 * Error_Handler
 *
 * Reached only if USART2 cannot be initialized. Interrupts are disabled so
 * nothing else runs, and the message goes to the SWV console because the
 * UART is the thing that failed.
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
