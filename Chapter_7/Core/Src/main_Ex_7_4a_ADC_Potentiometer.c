/**
 ******************************************************************************
 * @file    main_Ex_7_4a_ADC_Potentiometer.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - Exercise 7.4a: read a potentiometer with the ADC in
 *          single and continuous mode, and show it over UART   [VCP build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   10 kOhm potentiometer: ends to 3V3 and GND, wiper to PA0 (A0)
 *          B1 = PC13 (EXTI) switches between the two modes
 * @output  Serial terminal on the ST-Link VCP at 115200 8N1
 *
 * @details Ten times a second the loop prints the wiper position as a raw
 *          12-bit count, as millivolts, and as a bar. B1 switches mode:
 *
 *          Single mode. The loop starts one conversion, waits for EOC
 *          (12 us at 84 + 12 cycles of an 8 MHz ADCCLK), and reads DR. The
 *          ADC is idle the rest of the time. Simple, and good enough for a
 *          knob.
 *
 *          Continuous mode. One software start, and the ADC converts over and
 *          over by itself, about 83,000 times a second. To show what that
 *          means, the loop spends 10 ms catching every result it can and
 *          reports the rate. Then it goes back to printing ten lines a
 *          second, during which tens of thousands of results per second are
 *          overwritten unread. Continuous conversion produces data far faster
 *          than a CPU loop wants to consume it. Exercise 7.4b hands the
 *          collecting to the DMA and paces the conversions with a timer.
 *
 *          The millivolt figure assumes VDDA = 3300 mV, the Nucleo's nominal
 *          supply. Exercise 7.4b measures VDDA instead of assuming it.
 *
 *          Where the numbers come from:
 *            raw    0 to 4095, the fraction of VDDA on the wiper in 4096 steps.
 *            mV     raw x 3300 / 4095, so one step is about 0.8 mV.
 *            rate   (84 + 12) cycles / 8 MHz = 12 us per conversion, so the
 *                   expected figure is 1 / 12 us = 83,333 conversions per
 *                   second, printed to the nearest 100.
 *
 *          The rate is timed with the cycle counter, and the cycle counter,
 *          the ADC clock, and the UART all run from the same HSI oscillator.
 *          If the HSI runs 1% fast, the ADC converts 1% faster and the 10 ms
 *          window is also 1% shorter, so the printed rate does not move. It
 *          confirms the configuration, not the clock.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "exti_driver.h"
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
#define POT_PIN           (0U)              /* PA0 = ADC1_IN0 */
#define NOMINAL_VDDA_MV   (3300UL)
#define PRINT_PERIOD_US   (100000UL)        /* 10 lines per second */
#define RATE_WINDOW_US    (10000UL)         /* continuous-mode rate window */
#define BAR_WIDTH         (32U)
#define BTN_PORT          GPIOC
#define BTN_PIN           (13U)             /* B1, active low */
#define BUTTON_LOCKOUT_US (250000UL)        /* one accepted press per 250 ms */

typedef enum
{
    MODE_SINGLE = 0,
    MODE_CONTINUOUS
} adc_mode_t;

/* Set by the EXTI interrupt, cleared by the main loop. volatile because the
 * two run in different contexts and the loop must re-read it every pass. */
static volatile uint8_t s_button_flag = 0U;

/* =========================================================================
 * on_button_press  (interrupt context)
 *
 * Called by the Chapter 4 EXTI driver on every falling edge of PC13. It
 * only records that something happened; deciding whether it was a real
 * press, and switching mode, happens in the main loop, where it is safe to
 * take time and print.
 * ========================================================================= */
static void on_button_press(void)
{
    s_button_flag = 1U;
}

/* =========================================================================
 * pot_pin_init
 *
 * Puts PA0 in analog mode so the ADC can read the wiper voltage.
 * ========================================================================= */
static void pot_pin_init(void)
{
    /* Analog mode disconnects the digital input stage and the pulls. A pin
     * left in digital input mode would still convert, but the Schmitt
     * trigger behind it draws extra current at mid-rail voltages, and any
     * pull-up or pull-down would load the potentiometer. */
    const gpio_config_t pot = {
        .pin = POT_PIN, .mode = GP_MODE_ANALOG, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    gpio_init(POT_PORT, &pot);
}

/* =========================================================================
 * print_reading
 *
 * Prints one status line: the mode, the raw count, the millivolts, a
 * 32-character bar, and, in continuous mode, the measured conversion rate.
 * A rate of 0 means single mode, where a rate would mean nothing.
 *
 * Example lines:
 *   single     raw 2047  1649 mV  [###############-----------------]
 *   continuous raw 2051  1652 mV  [################----------------]  83300 conv/s
 * ========================================================================= */
static void print_reading(const char *mode, uint16_t raw, uint32_t rate)
{
    /* Bar length in proportion to the reading: 0 gives no '#', 4095 gives
     * all 32. Integer arithmetic, so the bar steps every 128 counts. */
    char bar[BAR_WIDTH + 1U];
    const uint32_t filled = ((uint32_t)raw * BAR_WIDTH) / ADC_FULL_SCALE;
    for (uint32_t i = 0U; i < BAR_WIDTH; i++)
    {
        bar[i] = (i < filled) ? '#' : '-';
    }
    bar[BAR_WIDTH] = '\0';

    /* %-10s pads the mode name to ten characters, so the numbers line up in
     * the same columns whichever mode is printing. */
    char line[112];
    if (rate == 0U)
    {
        (void)snprintf(line, sizeof(line), "%-10s raw %4u  %4lu mV  [%s]\r\n",
                       mode, (unsigned)raw,
                       (unsigned long)adc1_raw_to_mv(raw, NOMINAL_VDDA_MV), bar);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "%-10s raw %4u  %4lu mV  [%s]  %lu conv/s\r\n",
                       mode, (unsigned)raw,
                       (unsigned long)adc1_raw_to_mv(raw, NOMINAL_VDDA_MV), bar,
                       (unsigned long)rate);
    }
    /* Polled transmit: about 80 characters take roughly 7 ms at 115200
     * baud. In continuous mode several hundred results are overwritten
     * while this line goes out, which is the point the exercise makes. */
    (void)uart2_write_string(line);
}

/* =========================================================================
 * measure_continuous_rate
 *
 * Catches every continuous-mode result for RATE_WINDOW_US (10 ms) and scales
 * the count to one second (x 100). The last result caught is returned in
 * *last, so the status line shows a fresh reading.
 *
 * This only counts every result if each pass of the polling loop takes less
 * than one conversion time (12 us). If a pass took longer, results would be
 * overwritten between polls and the rate would read low. In continuous mode
 * without DMA the ADC does not flag an overwritten result, so such a miss
 * would be invisible; the expected 83,333 is the check.
 * ========================================================================= */
static uint32_t measure_continuous_rate(uint16_t *last)
{
    uint32_t count = 0U;
    const uint32_t t0 = cycle_counter_now();
    while (cycle_counter_elapsed_us(t0) < RATE_WINDOW_US)
    {
        /* Non-blocking: returns 1 only when EOC says a new result is ready,
         * and reading it clears EOC for the next one. */
        if (adc1_poll_result(last) != 0)
        {
            count++;
        }
    }
    /* About 833 results in 10 ms; x 100 gives conversions per second with a
     * resolution of 100. */
    return count * (1000000UL / RATE_WINDOW_US);
}

/* =========================================================================
 * main
 *
 * Initializes the tools (cycle counter, B1 interrupt, PA0 analog pin,
 * USART2), configures ADC1 for one channel, then loops forever doing two
 * jobs: handle B1 presses (switch mode) and, every 100 ms, take a reading
 * and print it.
 * ========================================================================= */
int main(void)
{
    /* Unbuffered stdout, as in every Chapter 7 exercise: ITM output appears
     * at once and the C library allocates no heap buffer for it. */
    setvbuf(stdout, NULL, _IONBF, 0);

    cycle_counter_init();                /* timing for the print period, the
                                            button lockout, and the rate     */
    exti_pc13_init(on_button_press);     /* B1 falling edge -> on_button_press */
    pot_pin_init();

    const uart_config_t uart = { .baud = 115200U };
    if (uart2_init(&uart) != UART_OK)
    {
        Error_Handler();
    }

    /* One-channel sequence: the potentiometer on ADC1_IN0, sampled for 84
     * ADC clock cycles (10.5 us). The wiper of a 10 kOhm pot has up to
     * about 2.5 kOhm of source resistance at mid travel, and a longer
     * sampling time gives the ADC's sampling capacitor time to charge
     * fully through it. 84 cycles is comfortably long for that. */
    (void)adc1_init();
    const adc_channel_cfg_t seq[] = { { ADC_CH_POT, ADC_SMP_84 } };
    (void)adc1_set_sequence(seq, 1U);

    (void)uart2_write_string("\r\nChapter 7 - Exercise 7.4a: potentiometer on PA0 (VCP)\r\n");
    (void)uart2_write_string("Turn the knob. Press B1 to switch single / continuous.\r\n");

    adc_mode_t mode = MODE_SINGLE;       /* start in the simple mode          */
    uint32_t last_print = cycle_counter_now();
    uint32_t last_press = cycle_counter_now();

    while (1)
    {
        /* Bounce filter: the EXTI driver reports every edge, so accept a press
         * only after the lockout and only while the pin still reads low. */
        if (s_button_flag != 0U)
        {
            s_button_flag = 0U;
            const uint8_t accept =
                ((gpio_read_pin(BTN_PORT, BTN_PIN) == GP_LOW) &&
                 (cycle_counter_elapsed_us(last_press) >= BUTTON_LOCKOUT_US)) ? 1U : 0U;
            if (accept == 0U)
            {
                /* bounce or release chatter: ignore */
            }
            else if (mode == MODE_SINGLE)
            {
                /* Single -> continuous: one software start and the ADC runs
                 * on its own from now on. */
                last_press = cycle_counter_now();
                mode = MODE_CONTINUOUS;
                (void)adc1_start_continuous();
                (void)uart2_write_string("-- continuous mode: the ADC restarts itself --\r\n");
            }
            else
            {
                /* Continuous -> single: clear CONT. The conversion in flight
                 * finishes; adc1_read_single() then clears CONT again anyway
                 * and starts each conversion itself. */
                last_press = cycle_counter_now();
                mode = MODE_SINGLE;
                adc1_stop_continuous();
                (void)uart2_write_string("-- single mode: one conversion per request --\r\n");
            }
        }

        /* Every 100 ms: one reading and one printed line. */
        if (cycle_counter_elapsed_us(last_print) >= PRINT_PERIOD_US)
        {
            last_print = cycle_counter_now();
            uint16_t raw = 0U;

            if (mode == MODE_SINGLE)
            {
                /* Start, wait about 12 us, read: the ADC works only when
                 * asked. */
                if (adc1_read_single(&raw) == ADC_OK)
                {
                    print_reading("single", raw, 0U);
                }
            }
            else
            {
                /* The ADC is already converting nonstop; spend 10 ms
                 * collecting results to show how fast it goes. */
                const uint32_t rate = measure_continuous_rate(&raw);
                print_reading("continuous", raw, rate);
            }
        }
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
