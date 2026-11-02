/**
 ******************************************************************************
 * @file    uart_driver_dma.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - USART2 with DMA transmit and receive (implementation)
 *
 * @details The polled half of this file is the Chapter 5 driver, unchanged.
 *          The DMA half follows RM0390 Section 25.4.13 step by step:
 *          program the stream, clear TC (transmit only), enable the stream,
 *          and set DMAT or DMAR so the USART starts raising requests.
 *
 *          How the circular receiver knows where it is. In circular mode the
 *          stream counts NDTR down from the buffer length to zero and then
 *          reloads it. So (length - NDTR) is the index the DMA will write
 *          next: the head. The main loop keeps its own tail, the index it
 *          has delivered up to, and everything between tail and head is new.
 *          HT and TC are not needed to find the data at all. They are wake-up
 *          calls that say a half has filled, and they double as a lap counter:
 *          if the interrupts report two more half-boundaries than the reader
 *          has walked past, the DMA has gone round the whole buffer and
 *          overwritten bytes nobody read.
 *
 *          The circular buffer, with the two indexes (256 bytes in 7.3b):
 *
 *              index 0                 128 (half)               255
 *              +------------------------+------------------------+
 *              | read already | new data | free for the DMA      |
 *              +------------------------+------------------------+
 *                             ^          ^
 *                            tail       head = len - NDTR
 *                     (loop delivered   (DMA writes here next)
 *                      up to here)
 *
 *          HT fires each time head passes 128, TC each time it passes 255
 *          and wraps to 0.
 *
 *          Who calls what in Chapter 7:
 *            7.3a  uart2_init, polled writes, uart2_dma_send,
 *                  uart2_dma_tx_busy, uart2_dma_receive, uart2_dma_rx_done
 *            7.3b  uart2_init, polled writes, uart2_dma_send,
 *                  uart2_dma_tx_busy, uart2_dma_rx_start_circular,
 *                  uart2_dma_rx_service, uart2_dma_rx_stats
 *            7.4a, 7.4b, 7.5  uart2_init and polled writes only. uart2_init
 *                  still configures the transmit stream, which is harmless:
 *                  nothing starts it.
 ******************************************************************************
 */

#include "uart_driver_dma.h"
#include "dma_driver.h"
#include "gpio_driver.h"   /* the Chapter 4 driver, reused for pin setup */
#include "main.h"
#include <stddef.h>

#define UART_TX_PIN             (2U)    /* PA2 */
#define UART_RX_PIN             (3U)    /* PA3 */

/* Bounded waits: a missing or stuck peripheral cannot hang the program.
 *   UART_FLAG_BUDGET      polls of TXE or TC; one byte at 115200 baud takes
 *                         87 us, far fewer polls than this.
 *   UART_DMA_WAIT_BUDGET  polls while a DMA transmit finishes; large enough
 *                         for the longest send the exercises make. */
#define UART_FLAG_BUDGET        (1000000UL)
#define UART_DMA_WAIT_BUDGET    (20000000UL)   /* covers a long DMA transmit */

/* RM0390 Table 28: USART2_TX = DMA1 Stream 6, USART2_RX = DMA1 Stream 5,
 * both on channel 4. */
#define UART_TX_STREAM          DMA1_Stream6
#define UART_RX_STREAM          DMA1_Stream5
#define UART_DMA_CHANNEL        (4U)

/* Shared between thread and interrupt context, so volatile: the main loop
 * must re-read each one every time instead of trusting a register copy.
 *   s_tx_busy        1 from uart2_dma_send() until the DMA reports TC.
 *   s_rx_done        1 when a normal-mode (fixed-length) receive is full.
 *   s_rx_circular    1 while the circular receiver is running.
 *   s_rx_events      UART_RX_EVT_* bits gathered since the last service.
 *   s_rx_isr_halves  half-boundaries the DMA has crossed, counted by the
 *                    HT and TC interrupts: the DMA's side of the lap count.
 *   s_half_count, s_full_count, s_idle_count, s_dma_errors
 *                    running totals for the 7.3b status line. */
static volatile uint8_t  s_tx_busy     = 0U;
static volatile uint8_t  s_rx_done     = 0U;
static volatile uint8_t  s_rx_circular = 0U;
static volatile uint32_t s_rx_events   = 0U;   /* UART_RX_EVT_* since last service */
static volatile uint32_t s_rx_isr_halves = 0U; /* half-boundaries the DMA crossed  */
static volatile uint32_t s_half_count  = 0U;
static volatile uint32_t s_full_count  = 0U;
static volatile uint32_t s_idle_count  = 0U;
static volatile uint32_t s_dma_errors  = 0U;

/* Circular receiver bookkeeping. Touched by the main loop only, so it needs
 * no volatile and no interrupt protection.
 *   s_rx_buf, s_rx_len    the buffer the DMA writes into.
 *   s_rx_tail             index the loop has delivered up to.
 *   s_rx_reader_halves    half-boundaries the reader has walked past: the
 *                         reader's side of the lap count.
 *   s_rx_bytes            total bytes handed to the chunk callback.
 *   s_rx_overflows        times the DMA lapped the reader and data was lost. */
static uint8_t  *s_rx_buf = NULL;
static uint32_t  s_rx_len = 0U;
static uint32_t  s_rx_tail = 0U;
static uint32_t  s_rx_reader_halves = 0U;      /* half-boundaries the reader crossed */
static uint32_t  s_rx_bytes = 0U;
static uint32_t  s_rx_overflows = 0U;

/* =========================================================================
 * DMA callbacks (interrupt context, called from dma_driver.c)
 * ========================================================================= */

/* -------------------------------------------------------------------------
 * tx_dma_event  (DMA1 Stream 6)
 *
 * The transmit is over (or failed): turn off the USART's DMA requests and
 * clear the busy flag. "Over" here means the DMA has handed the last byte
 * to DR; that byte and the one before it are still shifting out on the wire
 * for up to about 170 us more. A polled write that follows simply waits behind
 * them, because it checks TXE before writing.
 * ------------------------------------------------------------------------- */
static void tx_dma_event(uint32_t events)
{
    if ((events & (DMA_EVT_COMPLETE | DMA_EVT_XFER_ERR)) != 0U)
    {
        /* The DMA has written the last byte into DR. Release the request
         * line so polled writes and the next DMA send start clean. */
        USART2->CR3 &= ~USART_CR3_DMAT;
        s_tx_busy = 0U;
    }
}

/* -------------------------------------------------------------------------
 * rx_dma_event  (DMA1 Stream 5)
 *
 * Two jobs depending on the receive mode:
 *   circular  record which half filled (HT or TC), bump the counters, and
 *             count one more half-boundary for the lap detector. The data
 *             itself is collected later by uart2_dma_rx_service().
 *   normal    on TC the fixed-length block is full: stop the USART's DMA
 *             requests so later bytes are not taken, and raise s_rx_done.
 * Any DMA error is counted in both modes.
 * ------------------------------------------------------------------------- */
static void rx_dma_event(uint32_t events)
{
    if (s_rx_circular != 0U)
    {
        if ((events & DMA_EVT_HALF) != 0U)
        {
            s_rx_events |= UART_RX_EVT_HALF;
            s_half_count++;
            s_rx_isr_halves++;
        }
        if ((events & DMA_EVT_COMPLETE) != 0U)
        {
            s_rx_events |= UART_RX_EVT_FULL;
            s_full_count++;
            s_rx_isr_halves++;
        }
    }
    else if ((events & DMA_EVT_COMPLETE) != 0U)
    {
        USART2->CR3 &= ~USART_CR3_DMAR;
        s_rx_done = 1U;
    }
    else
    {
        /* no other event ends a normal-mode block */
    }

    if ((events & (DMA_EVT_XFER_ERR | DMA_EVT_DIRECT_ERR | DMA_EVT_FIFO_ERR)) != 0U)
    {
        s_dma_errors++;
    }
}

/* Drop any byte left in DR and clear ORE and IDLE: both are cleared by
 * reading SR and then DR (RM0390 Section 25.6.1). Called before a receive
 * starts, so the DMA's first byte is a fresh one and not something that
 * arrived while nobody was listening. */
static void rx_flush(void)
{
    (void)USART2->SR;
    (void)USART2->DR;
}

/* =========================================================================
 * uart2_init
 *
 * The Chapter 5 initialization: clock, pins, baud rate, 8N1, transmitter
 * and receiver on. Two Chapter 7 additions: the USART2 interrupt is enabled
 * for the idle-line event, and the transmit DMA stream is configured once,
 * ready for uart2_dma_send(). No receive is started here.
 * ========================================================================= */
uart_status_t uart2_init(const uart_config_t *cfg)
{
    if ((cfg == NULL) || (cfg->baud == 0UL))
    {
        return UART_ERR_PARAM;
    }

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    /* PA2 and PA3 in alternate function 7 (USART2). RX gets a pull-up so
     * the line idles high, as UART expects, even with nothing connected. */
    const gpio_config_t tx_cfg = {
        .pin = UART_TX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF7_USART2, .speed = GP_SPEED_LOW
    };
    const gpio_config_t rx_cfg = {
        .pin = UART_RX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_UP, .af = GP_AF7_USART2, .speed = GP_SPEED_LOW
    };
    gpio_init(GPIOA, &tx_cfg);
    gpio_init(GPIOA, &rx_cfg);

    /* BRR = 16 MHz / baud, rounded to the nearest integer (the "+ baud / 2").
     * 115200 gives 139 (0x8B): 115,108 baud in practice, 0.08% slow, well
     * inside what a UART receiver tolerates. */
    USART2->CR1 = 0U;
    USART2->BRR = (APB1_CLOCK_HZ + (cfg->baud / 2U)) / cfg->baud;
    USART2->CR2 = 0U;                     /* 1 stop bit                     */
    USART2->CR3 = 0U;                     /* DMAT/DMAR are set per transfer */
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    /* The vector is used for the idle-line event of the circular receiver. */
    NVIC_EnableIRQ(USART2_IRQn);
    USART2->CR1 |= USART_CR1_UE;

    /* Byte-wide transfers into DR, one frame per request. RM0390 Section 25.6
     * asks software for 32-bit register accesses; byte-wide DMA into DR is
     * nevertheless the configuration ST's own STM32CubeF4 UART DMA examples
     * use, because a wider PSIZE would pack several bytes into one write.
     *
     * The transmit stream never changes, so it is configured once here.
     * The receive stream is configured when a receive starts, because
     * normal and circular reception need different settings, and because
     * Exercise 7.5 gives DMA1 Stream 5 to the DAC instead. */
    const dma_config_t tx_dma = {
        .stream = UART_TX_STREAM, .channel = UART_DMA_CHANNEL,
        .dir = DMA_DIR_MEM_TO_PERIPH,
        .psize = DMA_SIZE_8BIT, .msize = DMA_SIZE_8BIT,
        .pinc = 0U, .minc = 1U, .circular = 0U,
        .priority = DMA_PRIO_MEDIUM,
        .irq_half = 0U, .irq_complete = 1U,
        .callback = tx_dma_event
    };
    if (dma_stream_init(&tx_dma) != DMA_OK)
    {
        return UART_ERR_TIMEOUT;
    }
    s_tx_busy = 0U;
    return UART_OK;
}

/* =========================================================================
 * Polled transmit (Chapter 5 behavior, made DMA-aware)
 * ========================================================================= */

/* Wait for any DMA transmit to finish before the CPU writes DR itself.
 * Without this, a polled byte could slip in between the DMA's bytes. */
static uart_status_t wait_tx_dma_idle(void)
{
    uint32_t budget = UART_DMA_WAIT_BUDGET;
    while (s_tx_busy != 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    return UART_OK;
}

/* =========================================================================
 * uart2_write_byte
 *
 * One byte, polled: wait until the transmit data register is empty (TXE),
 * then write it. Returns as soon as the byte is in DR, not when it has left
 * the pin.
 * ========================================================================= */
uart_status_t uart2_write_byte(uint8_t byte)
{
    if (wait_tx_dma_idle() != UART_OK)
    {
        return UART_ERR_TIMEOUT;
    }

    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART2->SR & USART_SR_TXE) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    USART2->DR = (uint32_t)byte & 0x1FFUL;   /* DR holds up to 9 data bits */
    return UART_OK;
}

/* =========================================================================
 * uart2_write
 *
 * A buffer, polled, then a wait for TC (transmission complete): the last
 * byte has fully left the shift register. Waiting for TC is what makes the
 * 7.3a polled timing the real time on the wire, and it means the line is
 * truly idle when this returns.
 * ========================================================================= */
uart_status_t uart2_write(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) && (len > 0UL))
    {
        return UART_ERR_PARAM;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        const uart_status_t st = uart2_write_byte(data[i]);
        if (st != UART_OK)
        {
            return st;
        }
    }

    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART2->SR & USART_SR_TC) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    return UART_OK;
}

/* uart2_write() for a null-terminated string: count the characters, send
 * them, and leave the terminator behind. */
uart_status_t uart2_write_string(const char *str)
{
    if (str == NULL)
    {
        return UART_ERR_PARAM;
    }
    uint32_t len = 0U;
    while (str[len] != '\0')
    {
        len++;
    }
    return uart2_write((const uint8_t *)str, len);
}

/* =========================================================================
 * DMA transmit
 * ========================================================================= */

/* =========================================================================
 * uart2_dma_send
 *
 * Starts a DMA transmit of len bytes and returns at once. The order follows
 * RM0390 Section 25.4.13: clear TC, start the stream, then set DMAT so the
 * USART begins requesting bytes. Only one transmit at a time: a second call
 * while one is running returns UART_ERR_BUSY instead of queuing, which is
 * why Exercise 7.3b counts skipped echoes.
 * ========================================================================= */
uart_status_t uart2_dma_send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return UART_ERR_PARAM;
    }
    if (s_tx_busy != 0U)
    {
        return UART_ERR_BUSY;
    }

    s_tx_busy = 1U;
    /* TC is a "write 0 to clear" bit: writing ~TC puts 0 in TC and 1 in
     * every other bit, and the other status bits ignore a written 1. */
    USART2->SR = ~USART_SR_TC;            /* step 6: TC is cleared by writing 0 */

    if (dma_stream_start(UART_TX_STREAM, (uint32_t)&USART2->DR,
                         (uint32_t)data, len) != DMA_OK)
    {
        s_tx_busy = 0U;
        return UART_ERR_TIMEOUT;
    }

    /* TXE is already 1, so the first request is raised the moment DMAT is set. */
    USART2->CR3 |= USART_CR3_DMAT;
    return UART_OK;
}

/* 1 while a DMA transmit is still feeding DR; cleared by tx_dma_event(). */
int uart2_dma_tx_busy(void)
{
    return (s_tx_busy != 0U) ? 1 : 0;
}

/* =========================================================================
 * DMA receive, normal mode: one fixed-length block
 * ========================================================================= */

/* =========================================================================
 * uart2_dma_receive  (Exercise 7.3a)
 *
 * Receives exactly len bytes into buf by DMA and returns at once. Every
 * byte goes from DR to memory with no CPU work; the only interrupt is TC
 * after the last one. Poll uart2_dma_rx_done() to see when it is full.
 *
 * Receive gets high priority and transmit medium: a late transmit byte only
 * delays output, but a late receive byte is overwritten by the next one.
 * ========================================================================= */
uart_status_t uart2_dma_receive(uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0U))
    {
        return UART_ERR_PARAM;
    }
    if (s_rx_circular != 0U)
    {
        return UART_ERR_BUSY;             /* the stream belongs to the circular receiver */
    }

    /* Stop requests while the stream is reprogrammed. */
    USART2->CR3 &= ~USART_CR3_DMAR;
    const dma_config_t rx_dma = {
        .stream = UART_RX_STREAM, .channel = UART_DMA_CHANNEL,
        .dir = DMA_DIR_PERIPH_TO_MEM,
        .psize = DMA_SIZE_8BIT, .msize = DMA_SIZE_8BIT,
        .pinc = 0U, .minc = 1U, .circular = 0U,
        .priority = DMA_PRIO_HIGH,
        .irq_half = 0U, .irq_complete = 1U,
        .callback = rx_dma_event
    };
    if (dma_stream_init(&rx_dma) != DMA_OK)
    {
        return UART_ERR_TIMEOUT;
    }

    rx_flush();                           /* start with an empty DR */
    s_rx_done = 0U;
    if (dma_stream_start(UART_RX_STREAM, (uint32_t)&USART2->DR,
                         (uint32_t)buf, len) != DMA_OK)
    {
        return UART_ERR_TIMEOUT;
    }
    /* From now on every RXNE becomes a DMA request instead of an interrupt. */
    USART2->CR3 |= USART_CR3_DMAR;
    return UART_OK;
}

/* 1 once the block started by uart2_dma_receive() is complete. */
int uart2_dma_rx_done(void)
{
    return (s_rx_done != 0U) ? 1 : 0;
}

/* =========================================================================
 * DMA receive, circular mode: two halves plus idle line
 * ========================================================================= */

/* =========================================================================
 * uart2_dma_rx_start_circular  (Exercise 7.3b)
 *
 * Starts continuous reception into buf. The stream runs in circular mode
 * with HT and TC interrupts, and the USART idle-line interrupt is enabled so
 * a short message that never fills a half is still noticed. All counters
 * restart from zero.
 *
 * len must be even (so the two halves are equal) and at least 4. The time
 * the main loop may go without calling uart2_dma_rx_service() before data
 * is lost is len bytes of line time: 256 bytes at 115200 baud is 22 ms.
 * ========================================================================= */
uart_status_t uart2_dma_rx_start_circular(uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len < 4U) || ((len & 1U) != 0U))
    {
        return UART_ERR_PARAM;
    }

    /* Quiet both sources of receive events while things are rewired. */
    USART2->CR1 &= ~USART_CR1_IDLEIE;
    USART2->CR3 &= ~USART_CR3_DMAR;

    const dma_config_t rx_dma = {
        .stream = UART_RX_STREAM, .channel = UART_DMA_CHANNEL,
        .dir = DMA_DIR_PERIPH_TO_MEM,
        .psize = DMA_SIZE_8BIT, .msize = DMA_SIZE_8BIT,
        .pinc = 0U, .minc = 1U, .circular = 1U,
        .priority = DMA_PRIO_HIGH,
        .irq_half = 1U, .irq_complete = 1U,
        .callback = rx_dma_event
    };
    if (dma_stream_init(&rx_dma) != DMA_OK)
    {
        return UART_ERR_TIMEOUT;
    }

    /* Fresh bookkeeping: tail and head both start at index 0, and both lap
     * counters start level. */
    s_rx_buf = buf;
    s_rx_len = len;
    s_rx_tail = 0U;
    s_rx_reader_halves = 0U;
    s_rx_bytes = 0U;
    s_rx_overflows = 0U;
    s_rx_isr_halves = 0U;
    s_rx_events = 0U;
    s_half_count = 0U;
    s_full_count = 0U;
    s_idle_count = 0U;
    s_dma_errors = 0U;
    s_rx_circular = 1U;

    rx_flush();                           /* also clears a stale IDLE flag */
    if (dma_stream_start(UART_RX_STREAM, (uint32_t)&USART2->DR,
                         (uint32_t)buf, len) != DMA_OK)
    {
        s_rx_circular = 0U;
        return UART_ERR_TIMEOUT;
    }
    /* Requests on, then the idle interrupt: the receiver is live. */
    USART2->CR3 |= USART_CR3_DMAR;
    USART2->CR1 |= USART_CR1_IDLEIE;
    return UART_OK;
}

/* =========================================================================
 * uart2_dma_rx_service  (Exercise 7.3b, called every main-loop pass)
 *
 * Hands every byte written since the last call to cb, in order, and returns
 * how many there were. Steps:
 *   1. Take and clear the event bits atomically.
 *   2. Work out the head from NDTR.
 *   3. Count the half-boundaries between tail and head, and compare with
 *      the count the interrupts kept. Two or more behind means the DMA went
 *      round the whole buffer: data was overwritten, so drop it, count an
 *      overflow, and restart reading from the head.
 *   4. Otherwise deliver tail..head, in two pieces if it wraps past the end.
 *
 * Example of a wrap, 256-byte buffer: tail = 250, head = 6. The callback is
 * called with buf[250..255] (6 bytes), then buf[0..5] (6 bytes).
 * ========================================================================= */
uint32_t uart2_dma_rx_service(uart_rx_chunk_cb_t cb)
{
    if ((s_rx_circular == 0U) || (cb == NULL))
    {
        return 0U;
    }

    /* Take the event bits and clear them in one step the ISR cannot split.
     * PRIMASK is saved and restored, not just re-enabled, so this is also
     * safe if the caller already had interrupts disabled. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint32_t events = s_rx_events;
    s_rx_events = 0U;
    __set_PRIMASK(primask);

    /* Where is the DMA writing now? NDTR reloads to the full length at the
     * wrap, which makes head == length; that is the same place as index 0. */
    const uint32_t len  = s_rx_len;
    const uint32_t half = len / 2U;
    uint32_t head = len - (uint32_t)dma_stream_remaining(UART_RX_STREAM);
    if (head >= len)
    {
        head = 0U;
    }
    /* Reading the interrupt count after NDTR means an HT or TC that fires in
     * between can only make the interrupt count one ahead, never behind. */
    const uint32_t isr_halves = s_rx_isr_halves;   /* read after NDTR */
    const uint32_t tail = s_rx_tail;

    /* Count the half-boundaries the reader walks past going tail -> head. */
    uint32_t crossed;
    if (head >= tail)
    {
        /* No wrap: only the middle boundary can lie between them. */
        crossed = ((tail < half) && (head >= half)) ? 1U : 0U;
    }
    else
    {
        /* Wrap: the end boundary for certain, plus the middle one on the
         * way to the end (if tail started in the first half), plus the
         * middle one again after the wrap (if head is in the second half). */
        crossed = 1U + ((tail < half) ? 1U : 0U) + ((head >= half) ? 1U : 0U);
    }
    s_rx_reader_halves += crossed;

    /* Two or more boundaries ahead means the DMA lapped the reader. A lag of
     * one, or a negative lag, is only the ISR and this read racing. */
    const int32_t lag = (int32_t)(isr_halves - s_rx_reader_halves);
    if (lag >= 2)
    {
        /* Some bytes between tail and head were overwritten, and there is no
         * way to tell which, so none of them is delivered. Resynchronize:
         * both counters level again, reading resumes at the head. */
        s_rx_overflows++;
        s_rx_reader_halves = isr_halves;
        s_rx_tail = head;
        return 0U;
    }

    /* Deliver the new bytes straight from the DMA buffer, no copy. */
    uint32_t delivered = 0U;
    if (head > tail)
    {
        cb(&s_rx_buf[tail], head - tail, events);
        delivered = head - tail;
    }
    else if (head < tail)
    {
        cb(&s_rx_buf[tail], len - tail, events);        /* up to the end   */
        delivered = len - tail;
        if (head > 0U)
        {
            cb(&s_rx_buf[0], head, events);             /* then the wrap   */
            delivered += head;
        }
    }
    else
    {
        /* head == tail: nothing new */
    }

    s_rx_tail = head;
    s_rx_bytes += delivered;
    return delivered;
}

/* =========================================================================
 * uart2_dma_rx_stats
 *
 * Copies the receiver's counters for the 7.3b status line. The fields are
 * read one after another, not atomically, so an interrupt in the middle can
 * make them disagree by one event. That is fine for a display.
 * ========================================================================= */
void uart2_dma_rx_stats(uart_rx_stats_t *out)
{
    if (out == NULL)
    {
        return;
    }
    out->half_events = s_half_count;
    out->full_events = s_full_count;
    out->idle_events = s_idle_count;
    out->bytes       = s_rx_bytes;
    out->overflows   = s_rx_overflows;
    out->dma_errors  = s_dma_errors;
}

/* =========================================================================
 * USART2_IRQHandler - now only the idle-line event.
 *
 * IDLE sets when the line has stayed high for one whole frame after at
 * least one byte arrived: the sender has paused. It is cleared by reading SR
 * then DR. With DMAR set, the DMA empties DR within a few bus cycles of each
 * byte arriving, so by the time the line has been idle for a whole frame DR
 * holds nothing new. The window in which this read could take a byte from
 * the DMA is those few cycles, right as a new byte lands after a pause.
 *
 * A typed line raises IDLE after every keystroke, because people pause far
 * longer than one frame between keys. A paste raises it only where the
 * stream pauses, which on the board was a few times per paste.
 * ========================================================================= */
void USART2_IRQHandler(void)
{
    const uint32_t sr = USART2->SR;       /* first half of the clear sequence */

    if (((sr & USART_SR_IDLE) != 0U) && ((USART2->CR1 & USART_CR1_IDLEIE) != 0U))
    {
        (void)USART2->DR;                 /* second half: IDLE is now clear   */
        s_rx_events |= UART_RX_EVT_IDLE;
        s_idle_count++;
    }
}
