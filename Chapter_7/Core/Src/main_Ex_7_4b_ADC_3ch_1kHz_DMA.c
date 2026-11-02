/**
 ******************************************************************************
 * @file    main_Ex_7_4b_ADC_3ch_1kHz_DMA.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - Exercise 7.4b: sample three analog channels at 1 kHz
 *          with timer-triggered ADC and circular DMA   [VCP build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   Potentiometer wiper on PA0 (A0). The other two channels are
 *          inside the chip, so no extra wiring is needed.
 *          PA6 toggles once per finished half-buffer (optional analyzer probe)
 * @output  Serial terminal on the ST-Link VCP at 115200 8N1
 *
 * @details The three channels, scanned in this order on every trigger:
 *
 *              ADC1_IN0   potentiometer        84 cycles  (10.5 us)
 *              ADC1_IN17  VREFINT, about 1.21 V 480 cycles (60 us)
 *              ADC1_IN18  temperature sensor   480 cycles  (60 us)
 *
 *          One scan takes (84 + 12) + 2 x (480 + 12) = 1080 ADCCLK cycles,
 *          135 us at 8 MHz, comfortably inside the 1 ms trigger period.
 *
 *          TIM2 overflows 1000 times a second and its TRGO output starts each
 *          scan. Nothing in software decides when a sample is taken, so the
 *          sample rate cannot jitter with what the main loop happens to be
 *          doing. DMA2 Stream 0 copies each result into a 600-entry buffer
 *          that holds two blocks of 100 scans. The half-transfer interrupt
 *          says block A is complete while the DMA fills block B; the
 *          transfer-complete interrupt says the reverse. The main loop
 *          averages whichever block just finished, prints one line, and is
 *          done long before the other block fills 100 ms later.
 *
 *          Three things come out of the averages:
 *            - VDDA, measured from VREFINT against ST's factory calibration,
 *              instead of assumed to be 3.3 V.
 *            - The potentiometer voltage, scaled by that measured VDDA.
 *            - The die temperature from the factory-calibrated sensor.
 *          And one thing comes out of the interrupt timestamps: the sample
 *          rate, from the time between consecutive blocks. Read that number
 *          for what it is. The timestamps come from the cycle counter, and
 *          the cycle counter and TIM2 both run from the same HSI oscillator,
 *          so if the HSI is 1.5 percent fast (as Chapter 6 measured) the
 *          trigger and the ruler speed up together and the line still says
 *          1000.0 Hz. The figure proves the configuration: prescaler, ARR,
 *          sequence, and DMA length. Only an instrument with its own clock,
 *          a logic analyzer on PA6, can check the time itself.
 *
 *          The buffer, as the DMA fills it (p = pot, r = VREFINT, t = temp):
 *
 *              index  0   1   2   3   4   5  ...  297 298 299 | 300 ... 599
 *                     p0  r0  t0  p1  r1  t1 ...  p99 r99 t99 | block B
 *                     <-------------- block A --------------> |
 *                                               HT fires here ^   TC at 599
 *
 *          After TC the DMA wraps to index 0 by itself (circular mode) and
 *          overwrites block A while the loop reads block B, and so on.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "adc_driver.h"
#include "uart_driver_dma.h"
#include "cycle_counter.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define POT_PORT          GPIOA
#define POT_PIN           (0U)
#define MARK_PORT         GPIOA
#define MARK_PIN          (6U)

/* Buffer geometry. Everything below follows from three choices: the scan
 * rate, the number of channels, and how many scans to average per line.
 *   SAMPLE_RATE_HZ   TIM2 update rate = scans per second (one scan converts
 *                    all three channels).
 *   CHANNELS         length of the ADC sequence; must match seq[] in main().
 *   SCANS_PER_BLOCK  scans averaged into one printed line. 100 scans at
 *                    1 kHz = one line every 100 ms.
 *   BLOCK_SAMPLES    results in one half of the buffer: 100 x 3 = 300.
 *   BUFFER_SAMPLES   the whole circular buffer, two blocks: 600 half-words
 *                    (1200 bytes). This is the DMA count, so it must stay
 *                    below 65536, the limit of the 16-bit NDTR register. */
#define SAMPLE_RATE_HZ    (1000UL)          /* scans per second            */
#define CHANNELS          (3U)
#define SCANS_PER_BLOCK   (100U)            /* one block = 100 ms of data  */
#define BLOCK_SAMPLES     (SCANS_PER_BLOCK * CHANNELS)
#define BUFFER_SAMPLES    (2U * BLOCK_SAMPLES)

/* Written by the DMA, read by the main loop one block at a time. uint16_t
 * because each ADC result is a right-aligned 12-bit value in a 16-bit DR,
 * and the DMA is set up for half-word transfers on both sides. Static, so
 * it exists for the whole program: the DMA writes it forever. */
static uint16_t s_samples[BUFFER_SAMPLES];

/* Interrupt-to-main hand-off. All volatile: the DMA callback writes them in
 * interrupt context, and the main loop must re-read them every pass rather
 * than reuse a value cached in a register.
 *   s_block_ready    1 = a finished block is waiting; the loop clears it.
 *   s_block_index    which half finished: 0 = block A, 1 = block B.
 *   s_block_cycles   cycle counter value when the block finished, for the
 *                    rate calculation.
 *   s_block_count    total blocks finished since start, printed as "blk".
 *   s_blocks_missed  blocks that finished while the previous one was still
 *                    unread: the main loop fell more than 100 ms behind.
 *   s_dma_errors     DMA transfer or direct-mode errors, printed as
 *                    "dmaerr". It should stay at 0; anything else points at
 *                    a bad address or a bus fault in the DMA setup. */
static volatile uint8_t  s_block_ready   = 0U;
static volatile uint8_t  s_block_index   = 0U;   /* 0: first half, 1: second */
static volatile uint32_t s_block_cycles  = 0U;   /* CYCCNT at the event      */
static volatile uint32_t s_block_count   = 0U;
static volatile uint32_t s_blocks_missed = 0U;   /* main loop too slow       */
static volatile uint32_t s_dma_errors    = 0U;

/* =========================================================================
 * on_adc_dma  (interrupt context, called from DMA2_Stream0_IRQHandler)
 *
 * Runs twice per buffer pass: on half transfer (block A full) and on
 * transfer complete (block B full). It does the minimum, as every callback
 * in this book does: record which block is ready and when, raise the flag,
 * and toggle PA6. All the averaging and printing happens in the main loop.
 * ========================================================================= */
static void on_adc_dma(uint32_t events)
{
    if ((events & (DMA_EVT_HALF | DMA_EVT_COMPLETE)) != 0U)
    {
        /* The flag is still set: the loop never got to the last block before
         * this one finished. That block's data is now being overwritten. */
        if (s_block_ready != 0U)
        {
            s_blocks_missed++;               /* previous block never consumed */
        }
        /* HT means the first half (block A) is complete, TC the second. */
        s_block_index  = ((events & DMA_EVT_COMPLETE) != 0U) ? 1U : 0U;
        s_block_cycles = cycle_counter_now();
        s_block_count++;
        s_block_ready  = 1U;                 /* set last: the data above is ready */
        /* One edge per block: on a logic analyzer PA6 is a square wave,
         * 100 ms high and 100 ms low at a 1 kHz scan rate. */
        gpio_toggle_pin(MARK_PORT, MARK_PIN);
    }
    if ((events & (DMA_EVT_XFER_ERR | DMA_EVT_DIRECT_ERR)) != 0U)
    {
        s_dma_errors++;
    }
}

/* =========================================================================
 * pins_init
 *
 * PA0 in analog mode for the potentiometer (no digital input stage, no
 * pulls), and PA6 as a push-pull output for the block marker. The two
 * internal channels need no pins.
 * ========================================================================= */
static void pins_init(void)
{
    const gpio_config_t pot = {
        .pin = POT_PIN, .mode = GP_MODE_ANALOG, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    const gpio_config_t mark = {
        .pin = MARK_PIN, .mode = GP_MODE_OUTPUT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    gpio_init(POT_PORT, &pot);
    gpio_init(MARK_PORT, &mark);
}

/* =========================================================================
 * main
 *
 * Setup, then hand the sampling to hardware: after adc1_start_timer_dma()
 * the timer, ADC, and DMA run by themselves. The loop below only wakes up
 * when a block is ready, ten times a second, and turns 100 scans into one
 * line of averaged, calibrated values.
 * ========================================================================= */
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    cycle_counter_init();                /* timestamps for the rate figure    */
    pins_init();

    const uart_config_t uart = { .baud = 115200U };
    if (uart2_init(&uart) != UART_OK)
    {
        Error_Handler();
    }

    /* Power the ADC, wake the internal sensor and reference, and program
     * the three-channel scan. The internal channels need at least 10 us of
     * sampling (DS10693); 480 cycles at 8 MHz is 60 us. The pot is a low
     * enough impedance for 84 cycles, which keeps the scan short. */
    (void)adc1_init();
    adc1_enable_internal_channels();
    const adc_channel_cfg_t seq[CHANNELS] = {
        { ADC_CH_POT,     ADC_SMP_84  },
        { ADC_CH_VREFINT, ADC_SMP_480 },
        { ADC_CH_TEMP,    ADC_SMP_480 }
    };
    (void)adc1_set_sequence(seq, CHANNELS);

    (void)uart2_write_string("\r\nChapter 7 - Exercise 7.4b: 3 channels at 1 kHz, timer + DMA (VCP)\r\n");

    /* From here on the hardware samples on its own: TIM2 at 1 kHz, a scan
     * per trigger, circular DMA into s_samples[], and on_adc_dma() every
     * 300 results. circular = 1 also sets DDS, so the ADC keeps requesting
     * DMA after the buffer wraps. */
    if (adc1_start_timer_dma(s_samples, (uint16_t)BUFFER_SAMPLES, SAMPLE_RATE_HZ,
                             1U, on_adc_dma) != ADC_OK)
    {
        Error_Handler();
    }

    /* Print the configuration once: the trigger rate TIM2 was really set to
     * (16 MHz / (ARR + 1)), the block size, and the buffer size. */
    char line[128];
    (void)snprintf(line, sizeof(line), "TIM2 trigger %lu Hz, %u scans per block, buffer %u samples\r\n",
                   (unsigned long)adc1_trigger_rate_hz(), (unsigned)SCANS_PER_BLOCK,
                   (unsigned)BUFFER_SAMPLES);
    (void)uart2_write_string(line);

    /* The rate needs two timestamps, so the first block prints 0.0 Hz. */
    uint32_t prev_cycles = 0U;
    uint8_t  have_prev   = 0U;

    while (1)
    {
        if (s_block_ready == 0U)
        {
            continue;                        /* nothing new: the DMA is busy */
        }

        /* s_samples[] is written by the DMA, not by code the compiler can
         * see. The barrier stops it from moving any buffer read above the
         * flag test, so every value read belongs to the finished block. */
        __COMPILER_BARRIER();

        /* Take a copy of the hand-off values, then clear the flag. The next
         * block cannot finish for another 100 ms, so these three reads and
         * the clear cannot be split by a new interrupt in practice. */
        const uint32_t base       = (uint32_t)s_block_index * BLOCK_SAMPLES;
        const uint32_t now_cycles = s_block_cycles;
        const uint32_t count      = s_block_count;
        s_block_ready = 0U;

        /* Average this block while the DMA fills the other one. The samples
         * are interleaved (pot, ref, temp, pot, ref, temp, ...), so scan s
         * starts at base + 3s. Averaging 100 readings cuts random noise by
         * about ten times (the square root of 100). The largest sum is
         * 100 x 4095 = 409,500, far inside a uint32_t. */
        uint32_t sum_pot = 0U;
        uint32_t sum_ref = 0U;
        uint32_t sum_ts  = 0U;
        for (uint32_t s = 0U; s < SCANS_PER_BLOCK; s++)
        {
            const uint32_t i = base + (s * CHANNELS);
            sum_pot += s_samples[i];
            sum_ref += s_samples[i + 1U];
            sum_ts  += s_samples[i + 2U];
        }
        const uint16_t pot = (uint16_t)(sum_pot / SCANS_PER_BLOCK);
        const uint16_t ref = (uint16_t)(sum_ref / SCANS_PER_BLOCK);
        const uint16_t ts  = (uint16_t)(sum_ts  / SCANS_PER_BLOCK);

        /* Measure VDDA first, then use it for the other two conversions, so
         * the pot voltage and the temperature do not depend on assuming
         * exactly 3.3 V. */
        const uint32_t vdda_mv = adc1_vdda_mv(ref);
        const uint32_t pot_mv  = adc1_raw_to_mv(pot, vdda_mv);
        const int32_t  temp_cc = adc1_temp_centi_c(ts, vdda_mv);
        /* Print the temperature as sign + magnitude, so a reading below 0 C
         * (a freezer test) prints as "-1.25" instead of a garbled value. */
        const uint32_t temp_mag = (uint32_t)((temp_cc < 0) ? -temp_cc : temp_cc);

        /* Sample rate from the time between two consecutive blocks, in tenths
         * of a hertz. Measured in HSI cycles, so it cannot see an HSI error. */
        uint32_t rate_x10 = 0U;
        if (have_prev != 0U)
        {
            /* 100 scans took period_us, so scans per second x 10 is
             * 100 x 10 x 1,000,000 / period_us. At exactly 100,000 us that
             * is 10000, printed as 1000.0 Hz. */
            const uint32_t period_us = (now_cycles - prev_cycles) / CYCLES_PER_US;
            if (period_us != 0U)
            {
                rate_x10 = (SCANS_PER_BLOCK * 10UL * 1000000UL) / period_us;
            }
        }
        prev_cycles = now_cycles;
        have_prev   = 1U;

        /* One line per block, about 97 characters: roughly 8.4 ms of polled
         * transmit, well inside the 100 ms before the next block. "missed",
         * "ovr", and "dmaerr" should all stay at 0. "missed" rising means
         * the main loop fell behind, "ovr" that the ADC lost a result, and
         * "dmaerr" that the DMA itself reported an error. */
        (void)snprintf(line, sizeof(line),
                       "blk %5lu  pot %4lu mV  VDDA %4lu mV  T %s%lu.%02lu C  "
                       "rate(HSI) %lu.%lu Hz  missed %lu  ovr %d  dmaerr %lu\r\n",
                       (unsigned long)count, (unsigned long)pot_mv,
                       (unsigned long)vdda_mv, (temp_cc < 0) ? "-" : "",
                       (unsigned long)(temp_mag / 100U), (unsigned long)(temp_mag % 100U),
                       (unsigned long)(rate_x10 / 10U), (unsigned long)(rate_x10 % 10U),
                       (unsigned long)s_blocks_missed, adc1_overrun(),
                       (unsigned long)s_dma_errors);
        (void)uart2_write_string(line);
    }
}

/* =========================================================================
 * Error_Handler
 *
 * Reached if USART2 or the ADC/DMA start-up fails. Interrupts are disabled
 * so nothing else runs, and the message goes to the SWV console because the
 * UART may be the thing that failed.
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
