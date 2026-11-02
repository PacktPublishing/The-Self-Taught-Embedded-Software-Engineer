/**
 ******************************************************************************
 * @file    adc_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - ADC1 driver (implementation)
 *
 * @details The order of the timer-triggered start matters, and it is the
 *          order RM0390 Section 13.8.1 implies: the DMA stream must be armed
 *          before the ADC is allowed to request, and the ADC must be armed
 *          before the timer produces its first trigger. Start them the other
 *          way round and the first result lands in DR with nobody to collect
 *          it, which is an overrun before the program has done anything.
 *
 *          DDS (DMA disable selection) decides what happens when NDTR hits
 *          zero. DDS = 0: the ADC stops requesting, which is right for one
 *          block. DDS = 1: requests keep coming, which is what a circular
 *          stream needs to loop forever.
 *
 *          Who calls what in Chapter 7:
 *            7.4a  adc1_init, adc1_set_sequence, adc1_read_single,
 *                  adc1_start_continuous, adc1_poll_result,
 *                  adc1_stop_continuous, adc1_raw_to_mv
 *            7.4b  adc1_init, adc1_enable_internal_channels,
 *                  adc1_set_sequence, adc1_start_timer_dma (circular),
 *                  adc1_trigger_rate_hz, adc1_overrun, adc1_vdda_mv,
 *                  adc1_raw_to_mv, adc1_temp_centi_c
 *            7.5   adc1_init, adc1_enable_internal_channels,
 *                  adc1_set_sequence, adc1_read_single,
 *                  adc1_start_timer_dma (one block), adc1_stop_timer_dma,
 *                  adc1_trigger_rate_hz, adc1_vdda_mv, adc1_raw_to_mv
 *
 *          Two status-register habits run through the file. ADC_SR flags are
 *          cleared by writing 0 to them, so "ADC1->SR = 0" clears every stale
 *          flag before a new start. EOC is also cleared by reading DR, which
 *          is why a read of the result is all a polled caller needs.
 ******************************************************************************
 */

#include "adc_driver.h"
#include "main.h"
#include <stddef.h>

#define ADC_EOC_BUDGET          (100000UL)
#define ADC_STARTUP_LOOPS       (200UL)    /* > 10 us at 16 MHz: covers tSTAB (3 us)
                                              and the sensor tSTART (10 us max)   */
#define ADC_EXTSEL_TIM2_TRGO    (0x6UL)    /* RM0390 13.13.3: 0110 = Timer 2 TRGO   */
#define ADC_EXTEN_RISING        (0x1UL)
#define TIM_MMS_UPDATE          (0x2UL)    /* TRGO = update event                   */
#define ADC_DMA_STREAM          DMA2_Stream0
#define ADC_DMA_CHANNEL         (0U)       /* RM0390 Table 29: ADC1 = DMA2 S0 ch0   */

/* The rate TIM2 was actually programmed for, kept so adc1_trigger_rate_hz()
 * can report it. 16 MHz does not divide evenly by every requested rate. */
static uint32_t s_trigger_rate_hz = 0U;

/* =========================================================================
 * startup_delay  (file-private)
 *
 * A short busy wait after powering something analog. The ADC needs tSTAB
 * (about 3 us) after ADON before its first conversion is valid, and the
 * temperature sensor and VREFINT need up to 10 us after TSVREFE. The loop
 * count gives more than that at 16 MHz with any optimization level, and it
 * runs only at initialization, so precision does not matter.
 * ========================================================================= */
static void startup_delay(void)
{
    for (volatile uint32_t i = 0U; i < ADC_STARTUP_LOOPS; i++)
    {
        /* short settling wait, see ADC_STARTUP_LOOPS */
    }
}

/* =========================================================================
 * adc1_init
 *
 * Clocks ADC1, sets the common prescaler, puts the converter in a known
 * state (12-bit, right-aligned, single channel, software start), and powers
 * it on. No conversion is started. The caller still has to put each analog
 * pin in analog mode with the GPIO driver.
 * ========================================================================= */
adc_status_t adc1_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;                 /* read-back: clock on before access  */

    /* ADCPRE = 00: ADCCLK = PCLK2 / 2 = 8 MHz. Common to ADC1-3. */
    ADC->CCR &= ~ADC_CCR_ADCPRE;

    /* Writing whole registers, not individual bits, clears anything a
     * previous exercise or a debugger session left behind. */
    ADC1->CR1 = 0U;                     /* RES = 00: 12-bit, no SCAN yet      */
    ADC1->CR2 = 0U;                     /* ALIGN = 0: right-aligned in DR     */
    ADC1->SR  = 0U;                     /* clear any stale EOC or OVR         */
    ADC1->CR2 |= ADC_CR2_ADON;          /* power the converter on             */
    startup_delay();                    /* wait out tSTAB before converting   */
    return ADC_OK;
}

/* =========================================================================
 * adc1_enable_internal_channels
 *
 * Powers the temperature sensor (channel 18) and the internal reference
 * VREFINT (channel 17) through the common TSVREFE bit. Both are off after
 * reset to save power, and both read garbage until they have settled, hence
 * the delay. Exercise 7.4b reads both; Exercise 7.5 reads VREFINT to work
 * out the real VDDA before it converts the DAC levels to millivolts.
 * ========================================================================= */
void adc1_enable_internal_channels(void)
{
    /* VBATE shares channel 18 with the sensor and wins if both are set. */
    ADC->CCR &= ~ADC_CCR_VBATE;
    ADC->CCR |= ADC_CCR_TSVREFE;
    startup_delay();                    /* sensor tSTART, up to 10 us         */
}

/* =========================================================================
 * adc1_set_sequence
 *
 * Programs the regular sequence: which channels, in what order, and the
 * sampling time of each. One entry is a single-channel conversion; two or
 * more turn on SCAN so one start converts the whole list in order.
 *
 * Example, Exercise 7.4b: { POT, 84 }, { VREFINT, 480 }, { TEMP, 480 }
 * gives L = 2 (three conversions), SQ1 = 0, SQ2 = 17, SQ3 = 18.
 *
 * The sampling time belongs to the channel, not to the slot: a channel
 * listed twice uses the same sampling time both times.
 * ========================================================================= */
adc_status_t adc1_set_sequence(const adc_channel_cfg_t *list, uint8_t count)
{
    if ((list == NULL) || (count == 0U) || (count > 16U))
    {
        return ADC_ERR_PARAM;
    }

    /* L holds the sequence length minus one: L = 0 means one conversion. */
    uint32_t sqr1 = ((uint32_t)(count - 1U) << ADC_SQR1_L_Pos);
    uint32_t sqr2 = 0U;
    uint32_t sqr3 = 0U;

    for (uint32_t i = 0U; i < count; i++)
    {
        const uint32_t ch = list[i].channel;
        if (ch > 18U)                   /* ADC1 has channels 0 to 18 */
        {
            return ADC_ERR_PARAM;
        }

        /* Sequence slot: SQ1-SQ6 in SQR3, SQ7-SQ12 in SQR2, SQ13-SQ16 in
         * SQR1, five bits each (RM0390 13.13.9 to 13.13.11). */
        if (i < 6U)
        {
            sqr3 |= (ch << (5U * i));
        }
        else if (i < 12U)
        {
            sqr2 |= (ch << (5U * (i - 6U)));
        }
        else
        {
            sqr1 |= (ch << (5U * (i - 12U)));
        }

        /* Sampling time: channels 0-9 in SMPR2, 10-18 in SMPR1, three bits
         * each (RM0390 13.13.4 and 13.13.5). Clear the old code first, then
         * write the new one, so a shorter code cannot leave stale bits. */
        const uint32_t smp = (uint32_t)list[i].smp & 0x7UL;
        if (ch < 10U)
        {
            ADC1->SMPR2 &= ~(0x7UL << (3U * ch));
            ADC1->SMPR2 |=  (smp   << (3U * ch));
        }
        else
        {
            ADC1->SMPR1 &= ~(0x7UL << (3U * (ch - 10U)));
            ADC1->SMPR1 |=  (smp   << (3U * (ch - 10U)));
        }
    }

    /* Write the three sequence registers only after the whole list has been
     * validated, so a bad entry never leaves a half-written sequence. */
    ADC1->SQR1 = sqr1;
    ADC1->SQR2 = sqr2;
    ADC1->SQR3 = sqr3;

    if (count > 1U)
    {
        ADC1->CR1 |= ADC_CR1_SCAN;      /* one start converts every slot      */
    }
    else
    {
        ADC1->CR1 &= ~ADC_CR1_SCAN;
    }
    return ADC_OK;
}

/* =========================================================================
 * adc1_read_single
 *
 * The simplest way to read the ADC: start one conversion by software, wait
 * for end of conversion (EOC), and read the result. The CPU waits the whole
 * conversion time, which with 480-cycle sampling at 8 MHz is about 62 us.
 *
 * Any continuous, DMA, or hardware-trigger setting left by an earlier mode
 * is switched off first, so this call always behaves the same way.
 * ========================================================================= */
adc_status_t adc1_read_single(uint16_t *out)
{
    if (out == NULL)
    {
        return ADC_ERR_PARAM;
    }

    ADC1->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_EXTEN);
    ADC1->SR = 0U;                      /* clear a stale EOC or OVR            */
    ADC1->CR2 |= ADC_CR2_SWSTART;       /* hardware clears SWSTART as it starts */

    /* Bounded wait: a conversion takes microseconds, so running out of
     * budget means the ADC is not powered or not clocked. */
    uint32_t budget = ADC_EOC_BUDGET;
    while ((ADC1->SR & ADC_SR_EOC) == 0U)
    {
        if (--budget == 0UL)
        {
            return ADC_ERR_TIMEOUT;
        }
    }
    *out = (uint16_t)(ADC1->DR & 0xFFFFUL);   /* reading DR clears EOC */
    return ADC_OK;
}

/* =========================================================================
 * adc1_start_continuous
 *
 * Continuous mode: one software start, then the ADC begins the next
 * conversion as soon as the previous one ends, forever. The CPU's only job
 * is to read DR before the next result overwrites it, which is the race
 * Exercise 7.4a measures.
 *
 * EOCS = 0 sets EOC at the end of each sequence; with a one-channel
 * sequence that is every conversion. With DMA and EOCS both 0, the ADC
 * does not raise OVR when a result is overwritten (RM0390 Section 13.8.2),
 * so a missed result in this mode is lost silently. That is one of the
 * reasons Exercise 7.4b moves to DMA.
 * ========================================================================= */
adc_status_t adc1_start_continuous(void)
{
    ADC1->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_EXTEN | ADC_CR2_EOCS);
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_SWSTART;       /* one start, then it never stops     */
    return ADC_OK;
}

/* =========================================================================
 * adc1_poll_result
 *
 * Non-blocking companion to adc1_start_continuous(): returns 1 and the new
 * result if EOC is set, or 0 at once if no new result is ready. The caller
 * can therefore check the ADC between other jobs instead of waiting on it.
 * ========================================================================= */
int adc1_poll_result(uint16_t *out)
{
    if ((out == NULL) || ((ADC1->SR & ADC_SR_EOC) == 0U))
    {
        return 0;
    }
    *out = (uint16_t)(ADC1->DR & 0xFFFFUL);   /* also clears EOC */
    return 1;
}

/* =========================================================================
 * adc1_stop_continuous
 *
 * Clears CONT. The ADC does not abort: it finishes the conversion already in
 * progress and then stops, so one more result may still appear in DR.
 * ========================================================================= */
void adc1_stop_continuous(void)
{
    ADC1->CR2 &= ~ADC_CR2_CONT;         /* the conversion in flight finishes  */
}

/* =========================================================================
 * trigger_timer_start  (file-private)
 *
 * Runs TIM2 as a pure rate generator. It has no output pin and no interrupt;
 * its only job is to produce a trigger output (TRGO) pulse on every update
 * event, and the ADC is wired internally to start a scan on each pulse.
 *
 * TIM2 is a 32-bit timer clocked at 16 MHz (APB1 prescaler = 1), so one
 * prescaler setting covers every rate from 1 Hz to 1 MHz: PSC = 0 and
 * ARR = 16 MHz / rate - 1. A 1 kHz trigger is ARR = 15999.
 * ========================================================================= */
static void trigger_timer_start(uint32_t rate_hz)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    /* The counter runs 0..ARR, so the period is ARR + 1 timer ticks. */
    const uint32_t arr = (APB1_CLOCK_HZ / rate_hz) - 1U;

    TIM2->CR1 = 0U;                     /* stopped, up-counting, no one-pulse */
    TIM2->PSC = 0U;                     /* timer tick = 16 MHz                */
    TIM2->ARR = arr;
    /* MMS = 010: send the update event out as TRGO, the signal the ADC's
     * EXTSEL = 0110 setting listens to. */
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | (TIM_MMS_UPDATE << TIM_CR2_MMS_Pos);
    /* PSC and ARR are buffered and take effect at the next update; UG forces
     * that update now. Because MMS selects the update event, this also sends
     * one TRGO, so the first scan starts at once rather than one period
     * later. */
    TIM2->EGR = TIM_EGR_UG;             /* load PSC and ARR now               */
    TIM2->SR  = 0U;                     /* discard the UIF flag UG just set   */
    TIM2->CR1 |= TIM_CR1_CEN;           /* start counting                     */

    /* Integer division: report the rate the hardware really produces, which
     * can differ from the request when 16 MHz is not an exact multiple. */
    s_trigger_rate_hz = APB1_CLOCK_HZ / (arr + 1U);
}

/* =========================================================================
 * adc1_start_timer_dma
 *
 * Hands the whole sampling job to hardware: TIM2 sets the pace, the ADC
 * converts the sequence once per trigger, and DMA2 Stream 0 copies every
 * result into buf. The CPU is involved only when the DMA reports a half
 * or a full buffer through cb.
 *
 *   circular = 1  (Exercise 7.4b) the buffer is refilled forever. HT says
 *                 the first half is ready to process, TC the second half,
 *                 while the DMA keeps filling the other one.
 *   circular = 0  (Exercise 7.5)  one block of count results, then TC.
 *
 * buf holds results in sequence order, scan after scan: for a three-channel
 * sequence, buf[0], buf[1], buf[2] are the first scan, buf[3] starts the
 * second. That is why count should be a multiple of the sequence length.
 *
 * The rate check only keeps the ARR arithmetic sane (ARR >= 15). It does
 * not check that the sequence can be converted in one trigger period; the
 * caller must keep rate x (sum of sampling time + 12 cycles, per channel)
 * below the 8,000,000 ADC clock cycles available each second, or the ADC
 * falls behind and overruns. Exercise 7.4b: 1000 x (96 + 492 + 492)
 * = 1,080,000 cycles per second, well inside the limit.
 * ========================================================================= */
adc_status_t adc1_start_timer_dma(uint16_t *buf, uint16_t count, uint32_t rate_hz,
                                  uint8_t circular, dma_callback_t cb)
{
    if ((buf == NULL) || (count == 0U) || (rate_hz == 0U) ||
        (rate_hz > (APB1_CLOCK_HZ / 16U)))
    {
        return ADC_ERR_PARAM;
    }

    /* Start from a clean state even if a previous run is still going. */
    adc1_stop_timer_dma();

    /* 1. Arm the DMA stream. Half-word transfers match the 16-bit DR and
     *    the uint16_t buffer; the peripheral address stays on DR while the
     *    memory address walks through buf. HT is only useful in circular
     *    mode, where it marks the half that can be processed. */
    const dma_config_t dma = {
        .stream = ADC_DMA_STREAM, .channel = ADC_DMA_CHANNEL,
        .dir = DMA_DIR_PERIPH_TO_MEM,
        .psize = DMA_SIZE_16BIT, .msize = DMA_SIZE_16BIT,
        .pinc = 0U, .minc = 1U,
        .circular = (circular != 0U) ? 1U : 0U,
        .priority = DMA_PRIO_HIGH,
        .irq_half = (circular != 0U) ? 1U : 0U,
        .irq_complete = 1U,
        .callback = cb
    };
    if (dma_stream_init(&dma) != DMA_OK)
    {
        return ADC_ERR_TIMEOUT;
    }
    if (dma_stream_start(ADC_DMA_STREAM, (uint32_t)&ADC1->DR,
                         (uint32_t)buf, count) != DMA_OK)
    {
        return ADC_ERR_TIMEOUT;
    }

    /* 2. Arm the ADC: DMA requests on, hardware trigger on TIM2 TRGO.
     *    The new CR2 value is built in a local copy and written once, so the
     *    ADC never sees a mix of the old and new trigger settings. */
    ADC1->SR = 0U;
    uint32_t cr2 = ADC1->CR2;
    cr2 &= ~(ADC_CR2_CONT | ADC_CR2_EXTSEL | ADC_CR2_EXTEN | ADC_CR2_DDS);
    cr2 |= ADC_CR2_DMA;                 /* a DMA request after each conversion */
    if (circular != 0U)
    {
        cr2 |= ADC_CR2_DDS;             /* keep requesting after NDTR wraps   */
    }
    cr2 |= (ADC_EXTSEL_TIM2_TRGO << ADC_CR2_EXTSEL_Pos);  /* which trigger   */
    cr2 |= (ADC_EXTEN_RISING << ADC_CR2_EXTEN_Pos);       /* on which edge   */
    ADC1->CR2 = cr2;

    /* 3. Start the clock that paces everything. From here on the CPU does
     *    nothing per sample. */
    trigger_timer_start(rate_hz);
    return ADC_OK;
}

/* =========================================================================
 * adc1_stop_timer_dma
 *
 * Undoes adc1_start_timer_dma() in the reverse order: stop the triggers
 * first, so no new scan starts, then stop the ADC's DMA requests, then the
 * stream itself, and finally clear any flag left behind.
 *
 * The timer and the DMA stream are touched only if their clocks are on. On
 * the first call, before anything was started, neither has been clocked,
 * and an unclocked peripheral ignores writes, so the steps would be
 * pointless.
 * ========================================================================= */
void adc1_stop_timer_dma(void)
{
    if ((RCC->APB1ENR & RCC_APB1ENR_TIM2EN) != 0U)
    {
        TIM2->CR1 &= ~TIM_CR1_CEN;      /* no more triggers                   */
    }
    ADC1->CR2 &= ~(ADC_CR2_EXTEN | ADC_CR2_DMA | ADC_CR2_DDS);
    if ((RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN) != 0U)
    {
        (void)dma_stream_stop(ADC_DMA_STREAM);
    }
    ADC1->SR = 0U;                      /* clear EOC or OVR from the last scan */
}

/* =========================================================================
 * adc1_trigger_rate_hz
 *
 * The scan rate TIM2 was really set to, after integer division. Note what
 * this is and is not: it is the rate the configuration asks for, computed
 * from the nominal 16 MHz. If the HSI oscillator runs a little fast or slow,
 * the real rate moves with it and this number cannot show that; only an
 * instrument with its own clock, such as a logic analyzer, can.
 * ========================================================================= */
uint32_t adc1_trigger_rate_hz(void)
{
    return s_trigger_rate_hz;
}

/* =========================================================================
 * adc1_overrun
 *
 * Returns 1 if OVR is set: a conversion finished while the previous result
 * was still unread in DR, so a result was lost. In DMA mode that means the
 * DMA did not keep up, which in a working Exercise 7.4b should never happen.
 * The flag stays set until software clears it.
 * ========================================================================= */
int adc1_overrun(void)
{
    return ((ADC1->SR & ADC_SR_OVR) != 0U) ? 1 : 0;
}

/* =========================================================================
 * Conversions to physical units
 *
 * VREFINT is a fixed internal voltage (about 1.21 V). Its raw reading
 * changes only because VDDA, the ADC's reference, changes. ST measured it at
 * exactly 3.3 V in production and stored the result, so
 *     VDDA = 3300 mV * VREFINT_CAL / VREFINT_raw.
 * The temperature calibration points were also taken at 3.3 V, so a sensor
 * reading is first rescaled to what it would have been at 3.3 V, then placed
 * on the straight line through (TS_CAL1, 30 C) and (TS_CAL2, 110 C).
 *
 * All the arithmetic is integer, in millivolts and hundredths of a degree,
 * so no floating point is needed. Every intermediate product fits easily in
 * 32 bits; the largest, about 4095 x 8000 in the temperature formula, is
 * far below the 2^31 limit of the signed arithmetic used there.
 * ========================================================================= */

/* VDDA in millivolts from one VREFINT reading. Example: VREFINT_CAL = 1500
 * and a reading of 1500 give 3300 mV; a reading of 1550 means VDDA is lower,
 * about 3193 mV, because the same 1.21 V is now a larger fraction of it. */
uint32_t adc1_vdda_mv(uint16_t vrefint_raw)
{
    /* The calibration word lives in read-only system memory, written once by
     * ST. volatile makes the compiler really read it rather than assume. */
    const uint32_t cal = (uint32_t)(*(const volatile uint16_t *)ADC_VREFINT_CAL_ADDR);

    if (vrefint_raw == 0U)
    {
        return 0U;                      /* avoid dividing by zero              */
    }
    return (ADC_CAL_VDDA_MV * cal) / (uint32_t)vrefint_raw;
}

/* Raw 12-bit result to millivolts: raw / 4095 of the reference VDDA.
 * Passing the measured VDDA, rather than assuming 3300, removes the error a
 * slightly off supply would otherwise add to every reading. */
uint32_t adc1_raw_to_mv(uint16_t raw, uint32_t vdda_mv)
{
    return ((uint32_t)raw * vdda_mv) / ADC_FULL_SCALE;
}

/* Die temperature in hundredths of a degree Celsius (2537 = 25.37 C).
 * This is the junction temperature of the chip, a few degrees above the
 * room, not the air temperature. */
int32_t adc1_temp_centi_c(uint16_t ts_raw, uint32_t vdda_mv)
{
    const int32_t cal1 = (int32_t)(*(const volatile uint16_t *)ADC_TS_CAL1_ADDR);
    const int32_t cal2 = (int32_t)(*(const volatile uint16_t *)ADC_TS_CAL2_ADDR);

    if (cal2 == cal1)
    {
        return 0;                       /* blank calibration: no line to use   */
    }

    /* Step 1: what this reading would have been at VDDA = 3.3 V, the supply
     * the calibration points were measured at. */
    const int32_t raw_at_3v3 = (int32_t)(((uint32_t)ts_raw * vdda_mv) / ADC_CAL_VDDA_MV);
    /* Step 2: linear interpolation between (cal1, 30.00 C) and
     * (cal2, 110.00 C), in hundredths: 3000 + (raw - cal1) * 8000 / span. */
    return 3000 + (((raw_at_3v3 - cal1) * (11000 - 3000)) / (cal2 - cal1));
}
