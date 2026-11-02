/**
 ******************************************************************************
 * @file    dma_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - DMA stream driver (implementation)
 *
 * @details Two pieces of address arithmetic carry this file, and both are the
 *          same trick the Chapter 4 GPIO driver used to turn a port handle
 *          into a clock-enable bit.
 *
 *          1. Which stream is this? The eight stream register blocks start
 *             0x10 bytes into each controller and are 0x18 bytes apart
 *             (RM0390 Section 9.5.11). Subtracting and dividing turns a
 *             DMA_Stream_TypeDef pointer into a stream index, 0 to 7.
 *
 *          2. Where are its flags? Streams 0 to 3 report in DMA_LISR and
 *             streams 4 to 7 in DMA_HISR, each in a six-bit group. The four
 *             groups do not sit at evenly spaced offsets: they start at bits
 *             0, 6, 16, and 22. A four-entry table is clearer than a formula.
 *
 *          Inside each six-bit group the layout is the same for every stream:
 *
 *              bit 0  FEIF   FIFO error
 *              bit 1  (reserved)
 *              bit 2  DMEIF  direct-mode error
 *              bit 3  TEIF   transfer error (a bus error on either port)
 *              bit 4  HTIF   half transfer: NDTR has counted down halfway
 *              bit 5  TCIF   transfer complete: NDTR has reached 0
 *
 *          The DMA_EVT_* bits in dma_driver.h use exactly these positions,
 *          so a group shifted down to bit 0 is already an event mask.
 *
 *          Who uses this driver in Chapter 7: uart_driver_dma.c (DMA transmit
 *          and receive in 7.3a and 7.3b; its init also sets up the transmit
 *          stream in 7.4 and 7.5), adc_driver.c (7.4b, 7.5), dac_driver.c
 *          (7.5), and main_Ex_7_3a_UART_DMA.c directly for the
 *          memory-to-memory copy.
 ******************************************************************************
 */

#include "dma_driver.h"
#include <stddef.h>

#define DMA_STREAM_OFFSET    (0x10UL)   /* first stream block in a controller */
#define DMA_STREAM_STRIDE    (0x18UL)   /* distance between stream blocks     */
#define DMA_STREAMS          (8U)
#define DMA_EN_BUDGET        (100000UL) /* bounded wait for EN to read back 0 */

/* Bit position of each stream's flag group inside LISR/HISR (and the
 * matching clear register LIFCR/HIFCR). Indexed by stream number modulo 4:
 * streams 0 and 4 use bit 0, streams 1 and 5 bit 6, and so on. */
static const uint8_t k_flag_shift[4] = { 0U, 6U, 16U, 22U };

/* NVIC line of every stream, indexed [controller][stream]. Streams 7 of
 * DMA1 and 5 to 7 of DMA2 sit apart from their neighbours in the vector
 * table, which is exactly why this is a table and not arithmetic. */
static const IRQn_Type k_irqn[2][DMA_STREAMS] =
{
    { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
      DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn },
    { DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
      DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn }
};

/* One callback per stream. Written in thread context, read in the ISR.
 * The volatile qualifier applies to the pointers themselves, so the ISR
 * always fetches the current one instead of a value cached in a register. */
static dma_callback_t volatile s_callbacks[2][DMA_STREAMS];

/* Interrupt-enable bits in DMA_SxCR, saved and restored around a stop. */
#define DMA_CR_IRQ_MASK  (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE)

/* =========================================================================
 * stream_locate  (file-private)
 *
 * Splits a stream pointer into its controller (0 = DMA1, 1 = DMA2) and its
 * stream index. Returns 0 on success, -1 if the pointer is not a stream.
 *
 * Every public function calls this first, so a wrong pointer is rejected
 * with DMA_ERR_PARAM before any register is touched.
 * ========================================================================= */
static int stream_locate(const DMA_Stream_TypeDef *stream,
                         uint32_t *ctrl, uint32_t *index)
{
    const uint32_t addr = (uint32_t)stream;
    uint32_t base;

    /* The two controllers sit at different base addresses, so the range the
     * pointer falls in says which controller it belongs to. */
    if ((addr >= DMA1_Stream0_BASE) && (addr <= DMA1_Stream7_BASE))
    {
        base  = DMA1_BASE;
        *ctrl = 0U;
    }
    else if ((addr >= DMA2_Stream0_BASE) && (addr <= DMA2_Stream7_BASE))
    {
        base  = DMA2_BASE;
        *ctrl = 1U;
    }
    else
    {
        return -1;
    }

    /* Example: DMA1_Stream6 sits at DMA1_BASE + 0x10 + 6 * 0x18 = 0xA0.
     * (0xA0 - 0x10) / 0x18 = 6. */
    *index = (addr - (base + DMA_STREAM_OFFSET)) / DMA_STREAM_STRIDE;
    return 0;
}

/* Controller number (0 or 1) back to its CMSIS register block. */
static DMA_TypeDef *controller_of(uint32_t ctrl)
{
    return (ctrl == 0U) ? DMA1 : DMA2;
}

/* =========================================================================
 * stream_flags  (file-private)
 *
 * Reads one stream's six flag bits and returns them shifted down to bit 0,
 * so the result is directly a mask of DMA_EVT_* bits. With clear != 0 it
 * also clears them.
 *
 * Only the flags that were read are cleared. The clear registers are
 * write-1-to-clear and ignore zeros, so an event that arrives between the
 * read and the write stays set and is not lost: it will be seen next time.
 * ========================================================================= */
static uint32_t stream_flags(uint32_t ctrl, uint32_t index, uint8_t clear)
{
    DMA_TypeDef *dma = controller_of(ctrl);
    const uint32_t shift = k_flag_shift[index & 0x3U];
    uint32_t flags;

    if (index < 4U)
    {
        /* Streams 0 to 3: low status register. The DMA_EVT_ALL mask drops
         * the reserved bit and the neighbouring stream's flags. */
        flags = (dma->LISR >> shift) & DMA_EVT_ALL;
        if (clear != 0U)
        {
            dma->LIFCR = flags << shift;   /* write 1 to clear, 0 is ignored */
        }
    }
    else
    {
        /* Streams 4 to 7: high status register, same layout. */
        flags = (dma->HISR >> shift) & DMA_EVT_ALL;
        if (clear != 0U)
        {
            dma->HIFCR = flags << shift;
        }
    }
    return flags;
}

/* =========================================================================
 * stream_clear_all  (file-private)
 *
 * Clears every flag of one stream whether it is set or not. Used before a
 * stream is enabled, because the hardware refuses to start a stream whose
 * flags from the previous transfer are still set.
 * ========================================================================= */
static void stream_clear_all(uint32_t ctrl, uint32_t index)
{
    DMA_TypeDef *dma = controller_of(ctrl);
    const uint32_t mask = DMA_EVT_ALL << k_flag_shift[index & 0x3U];

    if (index < 4U)
    {
        dma->LIFCR = mask;
    }
    else
    {
        dma->HIFCR = mask;
    }
}

/* =========================================================================
 * dma_stream_stop
 *
 * Stops a stream and leaves it ready to be reconfigured.
 *
 * Clearing EN does not stop the stream instantly: the hardware finishes the
 * item in flight and only then reads back EN = 0 (RM0390 Section 9.3.18).
 * Disabling a running stream also raises its transfer-complete flag (RM0390
 * Section 9.3.15), so the interrupt enables are parked while that happens
 * and restored afterwards. Without that, every stop would look like a
 * finished transfer to the peripheral driver's callback.
 *
 * Calling it on a stream that is already stopped is harmless: EN already
 * reads 0, so the wait loop is skipped.
 *
 * Returns DMA_ERR_TIMEOUT if EN never reads back 0 within DMA_EN_BUDGET
 * polls. That should not happen on working hardware; the bound is there so
 * a fault cannot hang the caller forever.
 * ========================================================================= */
dma_status_t dma_stream_stop(DMA_Stream_TypeDef *stream)
{
    uint32_t ctrl;
    uint32_t index;

    if ((stream == NULL) || (stream_locate(stream, &ctrl, &index) != 0))
    {
        return DMA_ERR_PARAM;
    }

    /* Remember which interrupts were enabled, then clear them together with
     * EN in a single write, so no interrupt can fire for the stop itself. */
    const uint32_t irq_bits = stream->CR & DMA_CR_IRQ_MASK;
    stream->CR &= ~(irq_bits | DMA_SxCR_EN);

    /* Wait for the hardware to confirm: EN reads 0 only once the item in
     * flight has been written. */
    uint32_t budget = DMA_EN_BUDGET;
    while ((stream->CR & DMA_SxCR_EN) != 0U)
    {
        if (--budget == 0UL)
        {
            return DMA_ERR_TIMEOUT;
        }
    }

    /* Throw away the TCIF the stop raised (and anything older), then put
     * the interrupt enables back for the next transfer. */
    stream_clear_all(ctrl, index);
    stream->CR |= irq_bits;
    return DMA_OK;
}

/* =========================================================================
 * dma_stream_init
 *
 * Programs everything about a stream that stays the same from one transfer
 * to the next: which request drives it (channel), direction, item widths,
 * address increments, circular mode, priority, FIFO or direct mode, and
 * which events interrupt. It also records the callback and enables or
 * disables the stream's NVIC line to match.
 *
 * It does not load addresses or a length and does not start anything; that
 * is dma_stream_start()'s job. A peripheral driver calls this once from its
 * own init function and then calls dma_stream_start() for every transfer.
 *
 * Returns DMA_ERR_PARAM for a NULL config, an unknown stream, a channel
 * above 7, or a memory-to-memory request the hardware cannot do.
 * ========================================================================= */
dma_status_t dma_stream_init(const dma_config_t *cfg)
{
    uint32_t ctrl;
    uint32_t index;

    if ((cfg == NULL) || (cfg->stream == NULL) || (cfg->channel > 7U) ||
        (stream_locate(cfg->stream, &ctrl, &index) != 0))
    {
        return DMA_ERR_PARAM;
    }

    /* RM0390 Section 9.3.7: only DMA2 can move memory to memory, and a
     * memory-to-memory stream can be neither circular nor in direct mode. */
    if ((cfg->dir == DMA_DIR_MEM_TO_MEM) && ((ctrl == 0U) || (cfg->circular != 0U)))
    {
        return DMA_ERR_PARAM;
    }

    /* Clock the controller before touching its registers. The read-back
     * makes sure the enable has taken effect before the next access, the
     * same pattern every driver since Chapter 4 uses. */
    RCC->AHB1ENR |= (ctrl == 0U) ? RCC_AHB1ENR_DMA1EN : RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;

    /* Configuration bits can only be written while EN = 0. */
    if (dma_stream_stop(cfg->stream) != DMA_OK)
    {
        return DMA_ERR_TIMEOUT;
    }

    /* Build the whole control word in a local variable and write it once,
     * so the stream never sees a half-configured state. Each field below is
     * the enum value placed at its DMA_SxCR position:
     *   CHSEL  which of the eight requests on this stream's multiplexer
     *   PL     priority against other streams of the same controller
     *   MSIZE  memory-side item width   (8, 16, or 32 bits)
     *   PSIZE  peripheral-side item width
     *   DIR    peripheral-to-memory, memory-to-peripheral, memory-to-memory */
    uint32_t cr = 0U;
    cr |= ((uint32_t)cfg->channel  << DMA_SxCR_CHSEL_Pos);
    cr |= ((uint32_t)cfg->priority << DMA_SxCR_PL_Pos);
    cr |= ((uint32_t)cfg->msize    << DMA_SxCR_MSIZE_Pos);
    cr |= ((uint32_t)cfg->psize    << DMA_SxCR_PSIZE_Pos);
    cr |= ((uint32_t)cfg->dir      << DMA_SxCR_DIR_Pos);
    /* MINC: step through a buffer. PINC: rarely set, because a peripheral
     * data register stays at one address. CIRC: when NDTR reaches 0 the
     * hardware reloads it and starts the block again, with no software. */
    if (cfg->minc != 0U)     { cr |= DMA_SxCR_MINC; }
    if (cfg->pinc != 0U)     { cr |= DMA_SxCR_PINC; }
    if (cfg->circular != 0U) { cr |= DMA_SxCR_CIRC; }

    /* Store the callback before the NVIC line is enabled below, so the
     * first interrupt can never find a stale pointer. */
    s_callbacks[ctrl][index] = cfg->callback;
    if (cfg->callback != NULL)
    {
        /* Errors are always reported when someone is listening. */
        cr |= (DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
        if (cfg->irq_half != 0U)     { cr |= DMA_SxCR_HTIE; }
        if (cfg->irq_complete != 0U) { cr |= DMA_SxCR_TCIE; }
    }
    cfg->stream->CR = cr;

    /* Peripheral transfers run in direct mode: each request moves one item
     * straight through. In direct mode the hardware uses PSIZE for both
     * sides once the stream is enabled, so the two widths should match.
     * Memory-to-memory needs the FIFO, so it gets the FIFO enabled
     * (DMDIS = 1) with a full threshold (FTH = 3): the stream reads until
     * the 16-byte FIFO is full, then writes it out. */
    if (cfg->dir == DMA_DIR_MEM_TO_MEM)
    {
        cfg->stream->FCR = DMA_SxFCR_DMDIS | (0x3UL << DMA_SxFCR_FTH_Pos);
    }
    else
    {
        cfg->stream->FCR = 0U;
    }

    if (cfg->callback != NULL)
    {
        /* Drop any interrupt left pending by an earlier run before enabling
         * the line, so the callback does not fire for something stale. */
        NVIC_ClearPendingIRQ(k_irqn[ctrl][index]);
        NVIC_EnableIRQ(k_irqn[ctrl][index]);
    }
    else
    {
        /* No callback: the caller polls with dma_stream_poll_events(). */
        NVIC_DisableIRQ(k_irqn[ctrl][index]);
    }
    return DMA_OK;
}

/* =========================================================================
 * dma_stream_start
 *
 * Starts one transfer on a stream configured by dma_stream_init(): stops it
 * if it is still running, loads the two addresses and the item count, clears
 * old flags, and sets EN. From that moment the hardware moves data on its
 * own and the function returns immediately; completion is reported through
 * the callback or found by polling.
 *
 * count is in data items of the peripheral-side width (PSIZE), not bytes:
 * 512 half-words is count = 512. NDTR is a 16-bit register, hence uint16_t.
 * ========================================================================= */
dma_status_t dma_stream_start(DMA_Stream_TypeDef *stream, uint32_t periph_addr,
                              uint32_t mem_addr, uint16_t count)
{
    uint32_t ctrl;
    uint32_t index;

    if ((stream == NULL) || (count == 0U) ||
        (stream_locate(stream, &ctrl, &index) != 0))
    {
        return DMA_ERR_PARAM;
    }

    /* PAR, M0AR, and NDTR are also writable only while EN = 0. */
    if (dma_stream_stop(stream) != DMA_OK)
    {
        return DMA_ERR_TIMEOUT;
    }

    stream->PAR  = periph_addr;   /* peripheral register, or source in mem-to-mem */
    stream->M0AR = mem_addr;      /* memory buffer, or destination in mem-to-mem  */
    stream->NDTR = (uint32_t)count;

    /* A stream will not start while any of its old flags are still set. */
    stream_clear_all(ctrl, index);

    /* Go. In a peripheral transfer nothing moves until the peripheral raises
     * its request (for example USART2 with DMAT set); in memory-to-memory
     * the transfer begins at once. */
    stream->CR |= DMA_SxCR_EN;
    return DMA_OK;
}

/* =========================================================================
 * dma_stream_remaining
 *
 * Returns DMA_SxNDTR, the number of items still to move in the current
 * block. It counts down as the transfer runs and reads 0 when a normal-mode
 * transfer is done. In circular mode it reloads instead of stopping, so
 * (block length - remaining) is the position the hardware writes next: the
 * circular UART receiver in Exercise 7.3b uses exactly that to find new data.
 * ========================================================================= */
uint16_t dma_stream_remaining(const DMA_Stream_TypeDef *stream)
{
    if (stream == NULL)
    {
        return 0U;
    }
    return (uint16_t)(stream->NDTR & 0xFFFFUL);   /* NDT field is bits 15:0 */
}

/* =========================================================================
 * dma_stream_poll_events
 *
 * The no-interrupt way to follow a transfer: returns the stream's current
 * DMA_EVT_* flags and, with clear != 0, clears the ones it returned. Used
 * by streams configured without a callback, such as the memory-to-memory
 * copy in Exercise 7.3a, which polls for DMA_EVT_COMPLETE.
 * ========================================================================= */
uint32_t dma_stream_poll_events(const DMA_Stream_TypeDef *stream, uint8_t clear)
{
    uint32_t ctrl;
    uint32_t index;

    if ((stream == NULL) || (stream_locate(stream, &ctrl, &index) != 0))
    {
        return 0U;
    }
    return stream_flags(ctrl, index, clear);
}

/* =========================================================================
 * Interrupt handlers
 *
 * Each handler reads its stream's flags, clears exactly those, and hands
 * them to whoever registered for the stream. Only the streams Chapter 7
 * routes have handlers. To use another stream with a callback, add a
 * one-line handler for it here, so the vector stays defined exactly once.
 *
 * The names must match the vector table in the startup file exactly. A
 * misspelled handler still compiles, but the vector keeps pointing at the
 * default handler, and the first interrupt hangs the program there.
 * ========================================================================= */
static void dma_irq_dispatch(uint32_t ctrl, uint32_t index)
{
    /* Read and clear first, then call. Clearing before the callback means
     * an event that happens while the callback runs sets its flag again and
     * re-pends this interrupt, instead of being wiped out on the way out. */
    const uint32_t events = stream_flags(ctrl, index, 1U);
    const dma_callback_t cb = s_callbacks[ctrl][index];

    if ((cb != NULL) && (events != 0U))
    {
        cb(events);
    }
}

void DMA1_Stream5_IRQHandler(void)   /* USART2_RX (ch4) or DAC1 (ch7) */
{
    /* Shared by two drivers: whichever of uart_driver_dma.c or dac_driver.c
     * initialized Stream 5 last owns the callback. */
    dma_irq_dispatch(0U, 5U);
}

void DMA1_Stream6_IRQHandler(void)   /* USART2_TX (ch4) */
{
    dma_irq_dispatch(0U, 6U);
}

void DMA2_Stream0_IRQHandler(void)   /* ADC1 (ch0) */
{
    dma_irq_dispatch(1U, 0U);
}
