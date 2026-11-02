/**
 ******************************************************************************
 * @file    main_Ex_7_5_DAC_Sine_DMA.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - Exercise 7.5: a 1 kHz sine wave on the DAC, fed by
 *          DMA from a table, measured back with the ADC   [VCP build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   DAC_OUT1 = PA4 (A2). No wiring: PA4 is also ADC1_IN4, so the ADC
 *          can measure the DAC on the same pin.
 *          PA6 toggles at every half pass of the table, giving a square wave
 *          at the sine frequency for an optional logic analyzer.
 * @output  Serial terminal on the ST-Link VCP at 115200 8N1
 *
 * @details Three steps:
 *
 *          1. Static levels. Write five codes with dac1_write() and read each
 *             back with the ADC. With the output buffer on, DS10693 Table 85
 *             guarantees the output only between 0.2 V and VDDA - 0.2 V. A
 *             lightly loaded board usually gets much closer to the rails than
 *             that, but a design cannot count on one board, so the sine table
 *             below spans 258 to 3838 rather than 0 to 4095.
 *
 *          2. The waveform. A 64-entry table in Flash holds one period. TIM6
 *             triggers the DAC 64,000 times a second, and each trigger makes
 *             the DAC request the next entry from DMA1 Stream 5 in circular
 *             mode. 64,000 / 64 = 1000 Hz. After the start call, the CPU does
 *             nothing at all to keep the wave going.
 *
 *          3. Measurement without an oscilloscope. The ADC samples PA4 at
 *             32 kHz into a 96-entry buffer, three periods of the wave. The
 *             program prints one period as a text plot, then finds the rising
 *             crossings of the mean to compute the frequency. Every two
 *             seconds it measures again and also counts DMA passes of the
 *             table, a second route to the same 1 kHz through different
 *             hardware. Different hardware, same clock: TIM6, TIM2, and the
 *             cycle counter all run from the HSI oscillator, so the figures
 *             agree with each other even if the HSI is a few percent off.
 *             They check the configuration. PA6 on a logic analyzer checks
 *             the clock.
 *
 *          Do not combine this exercise with a USART2 DMA receive: DAC1 and
 *          USART2_RX both live on DMA1 Stream 5 (RM0390 Table 28). The UART
 *          here only transmits, by polling.
 *
 *          Two DMA controllers work at the same time in step 3: DMA1 keeps
 *          feeding the DAC while DMA2 collects the ADC results. They are
 *          separate controllers, so neither waits for the other.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "dac_driver.h"
#include "adc_driver.h"
#include "uart_driver_dma.h"
#include "cycle_counter.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define MARK_PORT          GPIOA
#define MARK_PIN           (6U)

/* Waveform and measurement settings, and how they relate:
 *   SINE_POINTS        table entries per sine period.
 *   DAC_RATE_HZ        DAC updates per second; DAC_RATE_HZ / SINE_POINTS is
 *                      the sine frequency: 64,000 / 64 = 1 kHz.
 *   CAPTURE_RATE_HZ    ADC samples per second while measuring. 32 kHz gives
 *                      32 samples per 1 kHz period, far above the 2 kHz
 *                      minimum that sampling theory asks for.
 *   CAPTURE_POINTS     samples per snapshot: 96 = three periods, so at least
 *                      two rising crossings are found whatever the phase.
 *   CAPTURE_TIMEOUT_US how long to wait for a 3 ms snapshot before giving up.
 *   LEVEL_AVERAGE      ADC readings averaged for each static level and for
 *                      VDDA, to smooth out noise.
 *   PLOT_WIDTH         characters across the text plot, 0 V to VDDA.
 *   PLOT_ROWS          rows in the plot, one per sample: 32 = one period.
 *   REPORT_PERIOD_US   time between status lines in the final loop. */
#define SINE_POINTS        (64U)
#define DAC_RATE_HZ        (64000UL)        /* 64 points x 1 kHz            */
#define CAPTURE_POINTS     (96U)            /* three periods at 32 kHz      */
#define CAPTURE_RATE_HZ    (32000UL)
#define CAPTURE_TIMEOUT_US (50000UL)
#define LEVEL_AVERAGE      (16U)
#define PLOT_WIDTH         (48U)
#define PLOT_ROWS          (32U)            /* one period: 32 kHz / 1 kHz   */
#define REPORT_PERIOD_US   (2000000UL)

/* One period of 2048 + 1790 sin(2 pi i / 64), rounded. const puts it in
 * Flash (.rodata), and the DMA reads it from there directly. Each entry is a
 * 32-bit word because the DAC registers only accept word accesses.
 *
 * Why 2048 and 1790: 2048 is the middle of the 12-bit range. With the buffer
 * on, the datasheet guarantees the output only from 0.2 V to VDDA - 0.2 V,
 * about 248 codes from each end at 3.3 V, so the swing is kept to
 * 2048 +/- 1790 (258 to 3838), just inside that window. Step 1 may show
 * your board reaching closer to the rails; the table is sized to the
 * guaranteed range, not to what one board happens to do. */
static const uint32_t k_sine[SINE_POINTS] =
{
    2048UL, 2223UL, 2397UL, 2568UL, 2733UL, 2892UL, 3042UL, 3184UL,
    3314UL, 3432UL, 3536UL, 3627UL, 3702UL, 3761UL, 3804UL, 3829UL,
    3838UL, 3829UL, 3804UL, 3761UL, 3702UL, 3627UL, 3536UL, 3432UL,
    3314UL, 3184UL, 3042UL, 2892UL, 2733UL, 2568UL, 2397UL, 2223UL,
    2048UL, 1873UL, 1699UL, 1528UL, 1363UL, 1204UL, 1054UL,  912UL,
     782UL,  664UL,  560UL,  469UL,  394UL,  335UL,  292UL,  267UL,
     258UL,  267UL,  292UL,  335UL,  394UL,  469UL,  560UL,  664UL,
     782UL,  912UL, 1054UL, 1204UL, 1363UL, 1528UL, 1699UL, 1873UL
};

/* ADC snapshot of PA4, written by DMA2 Stream 0, 16-bit results. */
static uint16_t s_capture[CAPTURE_POINTS];

/* Interrupt-to-main hand-off, volatile because the DMA callbacks write them
 * and the main loop reads them.
 *   s_table_passes  complete passes of k_sine (one per TC), 1000 per second.
 *   s_capture_done  1 once the ADC snapshot is complete. */
static volatile uint32_t s_table_passes = 0U;   /* DMA TC count            */
static volatile uint8_t  s_capture_done = 0U;

/* =========================================================================
 * on_dac_dma  (interrupt context, DMA1 Stream 5)
 *
 * Called at the half and the end of every pass of the table. PA6 toggles on
 * both, so it is high for one half of the sine period and low for the
 * other: a 1 kHz square wave a logic analyzer can time with its own clock.
 * The pass counter gives the firmware's own 1 kHz figure.
 * ========================================================================= */
static void on_dac_dma(uint32_t events)
{
    if ((events & (DMA_EVT_HALF | DMA_EVT_COMPLETE)) != 0U)
    {
        gpio_toggle_pin(MARK_PORT, MARK_PIN);
    }
    if ((events & DMA_EVT_COMPLETE) != 0U)
    {
        s_table_passes++;
    }
}

/* =========================================================================
 * on_capture_dma  (interrupt context, DMA2 Stream 0)
 *
 * The ADC capture runs in normal (one block) mode, so TC arrives once, when
 * all 96 samples are in s_capture[]. It only raises the flag.
 * ========================================================================= */
static void on_capture_dma(uint32_t events)
{
    if ((events & DMA_EVT_COMPLETE) != 0U)
    {
        s_capture_done = 1U;
    }
}

/* Short name for the polled UART print used throughout. Polled transmit is
 * fine here: the DAC keeps playing by DMA while the CPU waits on the UART. */
static void say(const char *text)
{
    (void)uart2_write_string(text);
}

/* PA6 as a push-pull output for the 1 kHz marker. */
static void marker_init(void)
{
    const gpio_config_t mark = {
        .pin = MARK_PIN, .mode = GP_MODE_OUTPUT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    gpio_init(MARK_PORT, &mark);
}

/* =========================================================================
 * measure_vdda_mv
 *
 * VDDA from VREFINT, so every millivolt below is measured, not assumed.
 * Sixteen readings of the internal reference (480-cycle sampling, as the
 * datasheet's 10 us minimum requires) are averaged, then converted with
 * ST's factory calibration value. The result is the full scale used for
 * every millivolt printed afterwards.
 * ========================================================================= */
static uint32_t measure_vdda_mv(void)
{
    const adc_channel_cfg_t ref[] = { { ADC_CH_VREFINT, ADC_SMP_480 } };
    uint16_t raw = 0U;
    uint32_t sum = 0U;

    (void)adc1_set_sequence(ref, 1U);
    for (uint32_t i = 0U; i < LEVEL_AVERAGE; i++)
    {
        (void)adc1_read_single(&raw);
        sum += raw;
    }
    return adc1_vdda_mv((uint16_t)(sum / LEVEL_AVERAGE));
}

/* =========================================================================
 * static_levels  (step 1)
 *
 * Writes five codes across the range and reads each one back on PA4. For
 * each code the line shows the ideal voltage (code / 4095 of VDDA) next to
 * the measured one. The middle three should agree within about 20 mV: the
 * reading includes the gain and offset errors of both the DAC and the ADC,
 * and this loopback cannot tell them apart. Code 0 and code 4095 show how
 * close this board gets to the rails; the datasheet only guarantees 0.2 V
 * above 0 and 0.2 V below VDDA.
 * ========================================================================= */
static void static_levels(uint32_t vdda_mv)
{
    static const uint16_t codes[] = { 0U, 1024U, 2048U, 3072U, 4095U };
    /* 84 cycles is plenty for the DAC's low-impedance buffered output. */
    const adc_channel_cfg_t pin[] = { { ADC_CH_DAC_PIN, ADC_SMP_84 } };
    char line[96];

    (void)adc1_set_sequence(pin, 1U);
    say("\r\n[1] Static levels, read back on PA4 by the ADC\r\n");
    say("    code   ideal mV   measured mV\r\n");

    for (uint32_t c = 0U; c < (sizeof(codes) / sizeof(codes[0])); c++)
    {
        dac1_write(codes[c]);
        cycle_counter_delay_us(100U);           /* settling is 6 us max */

        /* Average sixteen readings so ADC noise does not hide the result. */
        uint32_t sum = 0U;
        for (uint32_t i = 0U; i < LEVEL_AVERAGE; i++)
        {
            uint16_t raw = 0U;
            (void)adc1_read_single(&raw);
            sum += raw;
        }
        const uint32_t measured = adc1_raw_to_mv((uint16_t)(sum / LEVEL_AVERAGE), vdda_mv);
        /* What a perfect DAC with rail-to-rail output would give. */
        const uint32_t ideal    = ((uint32_t)codes[c] * vdda_mv) / DAC_MAX_CODE;

        (void)snprintf(line, sizeof(line), "    %4u   %8lu   %11lu\r\n",
                       (unsigned)codes[c], (unsigned long)ideal, (unsigned long)measured);
        say(line);
    }
}

/* =========================================================================
 * capture_waveform  (step 3)
 *
 * Takes one 96-sample snapshot of PA4 at 32 kHz with the timer-triggered
 * ADC and DMA from Exercise 7.4b, this time in one-block (non-circular)
 * mode: 3 ms of sampling, then TC. Returns 0 on success, -1 if the ADC
 * could not start or the snapshot did not finish within 50 ms.
 *
 * Sampling time is only 15 cycles: (15 + 12) / 8 MHz = 3.4 us per
 * conversion, well inside the 31.25 us between samples, and the buffered
 * DAC output is a strong enough source to charge the ADC that quickly.
 * ========================================================================= */
static int capture_waveform(void)
{
    const adc_channel_cfg_t pin[] = { { ADC_CH_DAC_PIN, ADC_SMP_15 } };

    (void)adc1_set_sequence(pin, 1U);
    s_capture_done = 0U;
    if (adc1_start_timer_dma(s_capture, (uint16_t)CAPTURE_POINTS, CAPTURE_RATE_HZ,
                             0U, on_capture_dma) != ADC_OK)
    {
        return -1;
    }

    /* Wait for the snapshot, with a timeout so a fault cannot hang the
     * program. The DAC keeps playing throughout. */
    const uint32_t t0 = cycle_counter_now();
    while ((s_capture_done == 0U) && (cycle_counter_elapsed_us(t0) < CAPTURE_TIMEOUT_US))
    {
        /* 3 ms of sampling */
    }
    adc1_stop_timer_dma();              /* stop TIM2 and free DMA2 Stream 0 */

    /* The DMA wrote s_capture[] behind the compiler's back; keep every later
     * read of it below the completion test. */
    __COMPILER_BARRIER();
    return (s_capture_done != 0U) ? 0 : -1;
}

/* =========================================================================
 * estimate_freq_x10
 *
 * Frequency from rising crossings of the mean, interpolated between samples.
 * Returns tenths of a hertz, or 0 if fewer than two crossings were found.
 * Also returns the lowest and highest raw sample, for the swing on the
 * status line.
 *
 * Method:
 *   1. Mean of the 96 samples = the sine's midline.
 *   2. Find every place the signal goes from below the mean to at or above
 *      it between samples i and i + 1.
 *   3. Place each crossing between the two samples by straight-line
 *      interpolation, in hundredths of a sample, so the result is not
 *      limited to whole-sample steps.
 *   4. Average period = (last crossing - first crossing) / (crossings - 1).
 *   5. Frequency = sample rate / period. At exactly 1 kHz the period is
 *      32.00 samples (3200 in hundredths), and 32000 x 1000 / 3200 = 10000,
 *      printed as 1000.0 Hz.
 * ========================================================================= */
static uint32_t estimate_freq_x10(uint32_t *min_raw, uint32_t *max_raw)
{
    uint32_t sum = 0U;
    uint32_t lo = 0xFFFFU;
    uint32_t hi = 0U;

    /* One pass for the sum (for the mean), the minimum, and the maximum. */
    for (uint32_t i = 0U; i < CAPTURE_POINTS; i++)
    {
        sum += s_capture[i];
        lo = (s_capture[i] < lo) ? s_capture[i] : lo;
        hi = (s_capture[i] > hi) ? s_capture[i] : hi;
    }
    *min_raw = lo;
    *max_raw = hi;

    const int32_t mean = (int32_t)(sum / CAPTURE_POINTS);
    int32_t first_x100 = -1;            /* -1 = no crossing found yet */
    int32_t last_x100  = -1;
    uint32_t crossings = 0U;

    for (uint32_t i = 0U; (i + 1U) < CAPTURE_POINTS; i++)
    {
        const int32_t a = (int32_t)s_capture[i];
        const int32_t b = (int32_t)s_capture[i + 1U];
        if ((a < mean) && (b >= mean))  /* rising through the midline */
        {
            /* Fraction of the way from sample i to i + 1 where the line
             * a -> b meets the mean, in hundredths. b > a here, so the
             * division is safe. */
            const int32_t pos_x100 = ((int32_t)i * 100) + (((mean - a) * 100) / (b - a));
            if (first_x100 < 0)
            {
                first_x100 = pos_x100;
            }
            last_x100 = pos_x100;
            crossings++;
        }
    }

    if (crossings < 2U)
    {
        return 0U;                      /* need two crossings for one period */
    }
    const uint32_t period_x100 = (uint32_t)(last_x100 - first_x100) / (crossings - 1U);
    if (period_x100 == 0U)
    {
        return 0U;
    }
    /* trigger rate (32000) x 1000 / period in hundredths = Hz x 10. The rate
     * is the nominal TIM2 rate, so this figure shares the HSI blind spot. */
    return (adc1_trigger_rate_hz() * 1000UL) / period_x100;
}

/* =========================================================================
 * plot_one_period
 *
 * Prints the first 32 samples (one period) as a sideways text plot: one row
 * per sample, with its millivolt value and a '*' whose column is the
 * voltage as a fraction of VDDA across PLOT_WIDTH characters. Read top to
 * bottom, the stars trace one cycle of the sine. Where the plot starts in
 * the cycle depends on when the snapshot began, so it differs run to run.
 * ========================================================================= */
static void plot_one_period(uint32_t vdda_mv)
{
    char line[PLOT_WIDTH + 24U];

    say("\r\n    One period on PA4, sampled at 32 kHz (one row per sample)\r\n");
    for (uint32_t i = 0U; i < PLOT_ROWS; i++)
    {
        /* Column of the star: 0 at 0 V, PLOT_WIDTH - 1 at VDDA. */
        const uint32_t mv  = adc1_raw_to_mv(s_capture[i], vdda_mv);
        uint32_t col = (mv * PLOT_WIDTH) / vdda_mv;
        if (col >= PLOT_WIDTH)
        {
            col = PLOT_WIDTH - 1U;
        }

        /* Row label first ("  7 2905 mV |"), then spaces up to the star.
         * The (pos + 3) guard leaves room for "\r\n" and the terminator. */
        const int n = snprintf(line, sizeof(line), "    %2lu %4lu mV |",
                               (unsigned long)i, (unsigned long)mv);
        uint32_t pos = (n > 0) ? (uint32_t)n : 0U;
        for (uint32_t x = 0U; (x <= col) && ((pos + 3U) < sizeof(line)); x++)
        {
            line[pos] = (x == col) ? '*' : ' ';
            pos++;
        }
        line[pos]      = '\r';
        line[pos + 1U] = '\n';
        line[pos + 2U] = '\0';
        say(line);
    }
}

/* =========================================================================
 * main
 *
 * Runs the three steps once, then reports every two seconds forever. The
 * ADC is brought up before the DAC because step 1 needs it to measure VDDA
 * first; VDDA then scales every millivolt in the program.
 * ========================================================================= */
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    cycle_counter_init();
    marker_init();

    const uart_config_t uart = { .baud = 115200U };
    if (uart2_init(&uart) != UART_OK)
    {
        Error_Handler();
    }

    (void)adc1_init();
    adc1_enable_internal_channels();     /* VREFINT, for the VDDA measurement */
    (void)dac1_init();                   /* PA4 analog, channel 1 on, code 0  */

    say("\r\nChapter 7 - Exercise 7.5: DAC sine wave by DMA (VCP)\r\n");
    const uint32_t vdda_mv = measure_vdda_mv();
    char line[112];
    (void)snprintf(line, sizeof(line), "VDDA measured from VREFINT: %lu mV\r\n",
                   (unsigned long)vdda_mv);
    say(line);

    /* Step 1: five static levels. */
    static_levels(vdda_mv);

    /* Step 2: hand the waveform to TIM6 + DAC + DMA1. From here the sine
     * plays with no CPU involvement. */
    say("\r\n[2] Starting the waveform: 64-point table, TIM6 at 64 kHz, DMA1 Stream 5 circular\r\n");
    if (dac1_start_waveform(k_sine, (uint16_t)SINE_POINTS, DAC_RATE_HZ, on_dac_dma) != DAC_OK)
    {
        Error_Handler();
    }
    (void)snprintf(line, sizeof(line), "    DAC update rate %lu Hz, expected sine %lu Hz\r\n",
                   (unsigned long)dac1_sample_rate_hz(),
                   (unsigned long)(dac1_sample_rate_hz() / SINE_POINTS));
    say(line);

    /* Step 3: one snapshot, shown as a plot. */
    say("\r\n[3] Measuring it with the ADC\r\n");
    if (capture_waveform() == 0)
    {
        plot_one_period(vdda_mv);
    }
    else
    {
        say("    capture timed out\r\n");
    }

    uint32_t last_report = cycle_counter_now();
    uint32_t last_passes = s_table_passes;

    while (1)
    {
        if (cycle_counter_elapsed_us(last_report) < REPORT_PERIOD_US)
        {
            continue;                        /* the DMA keeps the wave going */
        }
        /* Two independent firmware figures for the sine frequency:
         *   DMA passes  table passes counted in the last ~2 s, divided by
         *               the exact elapsed time. 2000 passes in 2,000,000 us
         *               gives 10000, printed as 1000.0 Hz. The window always
         *               runs a few microseconds over 2 s, so the division
         *               is rounded (+ elapsed_us / 2) rather than truncated,
         *               or 9999.97 would print as 999.9. The product is
         *               about 2 x 10^10, too big for 32 bits, hence uint64_t.
         *   ADC         a fresh snapshot and estimate_freq_x10().
         * Both are timed by the HSI, like TIM6 itself, so they confirm the
         * configuration; PA6 on a logic analyzer is the external check. */
        const uint32_t elapsed_us = cycle_counter_elapsed_us(last_report);
        last_report = cycle_counter_now();
        const uint32_t passes = s_table_passes;
        const uint32_t pass_hz_x10 = (uint32_t)((((uint64_t)(passes - last_passes) * 10000000ULL)
                                                 + (elapsed_us / 2U)) / elapsed_us);
        last_passes = passes;

        uint32_t lo = 0U;
        uint32_t hi = 0U;
        uint32_t f_x10 = 0U;
        if (capture_waveform() == 0)
        {
            f_x10 = estimate_freq_x10(&lo, &hi);
        }

        /* One line: ADC frequency and swing, DMA-pass frequency, underrun.
         * The ideal swing is codes 258 and 3838 scaled by the measured VDDA,
         * about 209 to 3109 mV at 3317 mV. Expect the readings within a few
         * tens of millivolts of that, on either side: the step 1 errors
         * (DAC and ADC gain and offset) apply here too, and they outweigh
         * the few millivolts lost if a sample misses a peak. underrun should
         * stay 0. */
        (void)snprintf(line, sizeof(line),
                       "    ADC: %lu.%lu Hz(HSI), %lu to %lu mV | DMA passes: %lu.%lu Hz(HSI) | underrun %d\r\n",
                       (unsigned long)(f_x10 / 10U), (unsigned long)(f_x10 % 10U),
                       (unsigned long)adc1_raw_to_mv((uint16_t)lo, vdda_mv),
                       (unsigned long)adc1_raw_to_mv((uint16_t)hi, vdda_mv),
                       (unsigned long)(pass_hz_x10 / 10U), (unsigned long)(pass_hz_x10 % 10U),
                       dac1_underrun());
        say(line);
    }
}

/* =========================================================================
 * Error_Handler
 *
 * Reached if USART2 cannot be initialized or the waveform cannot start.
 * Interrupts are disabled so nothing else runs, and the message goes to the
 * SWV console because the UART may be the thing that failed.
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
