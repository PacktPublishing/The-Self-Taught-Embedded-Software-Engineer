/**
 ******************************************************************************
 * @file    dac_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - DAC channel 1 driver (implementation)
 *
 * @details One rule shapes dac1_start_waveform(): TSEL1 cannot be changed
 *          while EN1 is set (RM0390 Section 14.3.6). So the channel is
 *          switched off, the trigger and DMA bits are written, and only then
 *          is it switched back on. The same section explains why a DMA
 *          request appears at all: with DMAEN1 set, every external trigger
 *          copies DHR to DOR and asks the DMA for the next value.
 *
 *          The two registers that matter, and the one way data flows:
 *
 *              software or DMA --> DHR12R1 --(trigger)--> DOR --> PA4
 *                                  holding                output
 *
 *          DHR is where new codes are written. DOR is what the converter is
 *          actually putting out. With no trigger enabled, DHR is copied to
 *          DOR one APB1 cycle after each write (dac1_write). With a trigger
 *          enabled, the copy waits for the trigger (dac1_start_waveform),
 *          which is what makes the sample timing exact.
 *
 *          Used by Exercise 7.5 only: dac1_init and dac1_write for the static
 *          levels, dac1_start_waveform for the sine, dac1_sample_rate_hz and
 *          dac1_underrun for the status lines.
 ******************************************************************************
 */

#include "dac_driver.h"
#include "gpio_driver.h"
#include "main.h"
#include <stddef.h>

#define DAC_PIN              (4U)       /* PA4 = DAC_OUT1                        */
#define DAC_DMA_STREAM       DMA1_Stream5
#define DAC_DMA_CHANNEL      (7U)       /* RM0390 Table 28: DAC1 = DMA1 S5 ch7   */
#define DAC_TSEL_TIM6_TRGO   (0x0UL)    /* RM0390 Table 94: 000 = Timer 6 TRGO   */
#define TIM_MMS_UPDATE       (0x2UL)    /* TRGO = update event                   */
/* TIM6 is a 16-bit counter running at 16 MHz with PSC = 0, so its slowest
 * update rate is 16 MHz / 65536 = 244.1 Hz. Rounding up gives the lowest
 * rate this driver accepts without needing a prescaler. */
#define TIM6_MIN_RATE_HZ     ((APB1_CLOCK_HZ / 65536UL) + 1UL)   /* 245 Hz */
/* The fastest the DAC itself can follow: DS10693 Table 85 gives a maximum
 * update rate of 1 MS/s, and that only for small (1 LSB) steps. Larger steps
 * need up to 6 us to settle with the buffer on, so practical rates for a
 * full-swing waveform are lower still. Capping here also keeps ARR >= 15,
 * well away from ARR = 0, where TIM6 would never count. */
#define DAC_MAX_RATE_HZ      (1000000UL)                        /* 1 MS/s */

/* The rate TIM6 was actually programmed for, after integer division. */
static uint32_t s_sample_rate_hz = 0U;

/* =========================================================================
 * dac1_init
 *
 * Gets channel 1 ready for static output: PA4 in analog mode, DAC clock on,
 * channel 1 reset to plain software-written mode with the output buffer
 * enabled, code 0 loaded, and the channel switched on. After this,
 * dac1_write() moves the pin.
 * ========================================================================= */
dac_status_t dac1_init(void)
{
    /* The pin must be analog before the channel is enabled, so the digital
     * input stage does not draw current at mid-rail (RM0390 Section 14.2,
     * note under the DAC pin table). */
    const gpio_config_t pin = {
        .pin = DAC_PIN, .mode = GP_MODE_ANALOG, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    gpio_init(GPIOA, &pin);

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    (void)RCC->APB1ENR;                 /* read-back: clock on before access  */

    /* Channel 1 only: buffer on (BOFF1 = 0), no trigger, no DMA, no wave.
     * DAC_CR holds channel 1 in bits 15:0 and channel 2 in bits 31:16, so
     * clearing the low half resets channel 1 and leaves channel 2 alone. */
    DAC->CR &= ~(0xFFFFUL);
    DAC->DHR12R1 = 0U;                  /* start at the bottom of the range   */
    DAC->CR |= DAC_CR_EN1;              /* power channel 1 on                 */
    return DAC_OK;
}

/* =========================================================================
 * dac1_write
 *
 * Software-paced output: write one 12-bit code and the pin follows. With no
 * trigger enabled, DHR is copied to DOR one APB1 cycle later, and the
 * buffered output then needs a few microseconds to settle (DS10693 Table 85
 * gives the settling time). Codes above 4095 are clamped rather than
 * wrapped, so a bad value saturates instead of jumping to the bottom.
 * ========================================================================= */
void dac1_write(uint16_t code)
{
    if (code > DAC_MAX_CODE)
    {
        code = DAC_MAX_CODE;
    }
    /* DHR12R1: 12-bit, right-aligned holding register. 32-bit write, as
     * RM0390 Section 14.5 requires for every DAC register. */
    DAC->DHR12R1 = (uint32_t)code;
}

/* =========================================================================
 * trigger_timer_start  (file-private)
 *
 * Runs TIM6 as the DAC's sample clock. TIM6 is a basic timer: no pins and
 * no capture channels, only a counter and an update event, which makes it
 * exactly right for this job. Each update produces one TRGO pulse, and with
 * TSEL1 = 000 the DAC loads its next sample on that pulse.
 *
 * PSC = 0 and ARR = 16 MHz / rate - 1. Exercise 7.5 asks for 64 kHz, which
 * is ARR = 249, exactly 64,000 samples per second.
 *
 * Accepted range: 245 Hz (TIM6_MIN_RATE_HZ, the slowest a 16-bit counter
 * reaches with no prescaler) to 1 MHz (DAC_MAX_RATE_HZ, the DAC's own update
 * limit). Without the upper cap, a request near 16 MHz would give ARR = 0,
 * and a timer with ARR = 0 does not count, so no trigger would ever come.
 * ========================================================================= */
static dac_status_t trigger_timer_start(uint32_t rate_hz)
{
    if ((rate_hz < TIM6_MIN_RATE_HZ) || (rate_hz > DAC_MAX_RATE_HZ))
    {
        return DAC_ERR_PARAM;
    }

    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    (void)RCC->APB1ENR;

    /* The counter runs 0..ARR, so one period is ARR + 1 ticks. */
    const uint32_t arr = (APB1_CLOCK_HZ / rate_hz) - 1U;

    TIM6->CR1 = 0U;                     /* stopped, auto-reload not buffered  */
    TIM6->PSC = 0U;                     /* timer tick = 16 MHz                */
    TIM6->ARR = arr;                    /* 64 kHz -> 249                       */
    /* MMS = 010: the update event goes out as TRGO, the signal TSEL1 = 000
     * selects. */
    TIM6->CR2 = (TIM6->CR2 & ~TIM_CR2_MMS) | (TIM_MMS_UPDATE << TIM_CR2_MMS_Pos);
    TIM6->EGR = TIM_EGR_UG;             /* load PSC and ARR now               */
    TIM6->SR  = 0U;                     /* discard the UIF flag UG just set   */
    TIM6->CR1 |= TIM_CR1_CEN;           /* start counting: samples begin      */

    /* Integer division: the rate the hardware really produces from a
     * nominal 16 MHz. Like the ADC trigger rate, it cannot reveal an HSI
     * error, because it is computed from the same clock it describes. */
    s_sample_rate_hz = APB1_CLOCK_HZ / (arr + 1U);
    return DAC_OK;
}

/* =========================================================================
 * dac1_start_waveform
 *
 * Plays table[] on PA4 forever: TIM6 sets the pace, each trigger moves the
 * waiting sample from DHR to DOR, and DMA1 Stream 5 immediately refills DHR
 * with the next table entry. Circular DMA wraps from the last entry to the
 * first by itself, so after this call the CPU does nothing per sample.
 *
 * Output frequency = sample_rate / count. Exercise 7.5: 64 entries at
 * 64 kHz gives a 1 kHz sine.
 *
 * One detail of the hand-off: a trigger outputs the value already waiting
 * in DHR and only then requests the next one. The very first trigger
 * therefore outputs whatever DHR held before the waveform started (the last
 * static level), and table[0] appears one sample period later. After that
 * the table plays in order.
 *
 * Order matters, as in the ADC driver: arm the DMA first, then the DAC, and
 * start the timer last, so the first trigger finds everything ready.
 * ========================================================================= */
dac_status_t dac1_start_waveform(const uint32_t *table, uint16_t count,
                                 uint32_t sample_rate, dma_callback_t cb)
{
    if ((table == NULL) || (count < 2U))
    {
        return DAC_ERR_PARAM;
    }

    /* Start from a clean state even if a waveform is already playing. */
    dac1_stop_waveform();

    /* 1. Arm the DMA: table (memory, incrementing) to DHR12R1 (peripheral,
     *    fixed), 32-bit items, circular. HT and TC are reported so the
     *    caller can count passes of the table. Priority high, because a late
     *    DMA write is exactly what an underrun is. */
    const dma_config_t dma = {
        .stream = DAC_DMA_STREAM, .channel = DAC_DMA_CHANNEL,
        .dir = DMA_DIR_MEM_TO_PERIPH,
        .psize = DMA_SIZE_32BIT, .msize = DMA_SIZE_32BIT,   /* word access only */
        .pinc = 0U, .minc = 1U, .circular = 1U,
        .priority = DMA_PRIO_HIGH,
        .irq_half = 1U, .irq_complete = 1U,
        .callback = cb
    };
    if (dma_stream_init(&dma) != DMA_OK)
    {
        return DAC_ERR_TIMEOUT;
    }
    if (dma_stream_start(DAC_DMA_STREAM, (uint32_t)&DAC->DHR12R1,
                         (uint32_t)table, count) != DMA_OK)
    {
        return DAC_ERR_TIMEOUT;
    }

    /* 2. Arm the DAC. TSEL1 is only writable with EN1 = 0. */
    DAC->CR &= ~DAC_CR_EN1;
    /* No built-in noise or triangle generator (WAVE1 = 00, MAMP1 cleared):
     * the table is the waveform. */
    DAC->CR &= ~(DAC_CR_TSEL1 | DAC_CR_WAVE1 | DAC_CR_MAMP1);
    DAC->CR |= (DAC_TSEL_TIM6_TRGO << DAC_CR_TSEL1_Pos);
    /* TEN1: wait for the trigger before copying DHR to DOR.
     * DMAEN1: after each trigger, request the next sample from the DMA. */
    DAC->CR |= (DAC_CR_TEN1 | DAC_CR_DMAEN1);
    DAC->SR  = DAC_SR_DMAUDR1;          /* write 1 to clear a stale underrun   */
    DAC->CR |= DAC_CR_EN1;

    /* 3. Start the sample clock. */
    return trigger_timer_start(sample_rate);
}

/* =========================================================================
 * dac1_stop_waveform
 *
 * Stops the waveform in the order RM0390 asks for: triggers off first, then
 * the DMA stream, then the DAC's trigger and DMA modes. The channel is
 * switched back on afterwards in plain mode, so dac1_write() works again
 * straight away and DOR keeps the last sample that was played.
 *
 * EN1 is cleared for a few instructions while TEN1 and DMAEN1 change, so the
 * pin can leave the driven level for that moment. That matters only if
 * something is measuring the output exactly when the waveform stops.
 *
 * The timer and the stream are touched only if their clocks are on, so this
 * is safe to call before the first waveform was ever started.
 * ========================================================================= */
void dac1_stop_waveform(void)
{
    if ((RCC->APB1ENR & RCC_APB1ENR_TIM6EN) != 0U)
    {
        TIM6->CR1 &= ~TIM_CR1_CEN;      /* no more triggers                   */
    }
    if ((RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN) != 0U)
    {
        /* RM0390 9.3.18: stop the stream before the peripheral it serves. */
        (void)dma_stream_stop(DAC_DMA_STREAM);
    }
    DAC->CR &= ~DAC_CR_EN1;
    DAC->CR &= ~(DAC_CR_TEN1 | DAC_CR_DMAEN1);   /* back to software mode */
    DAC->CR |= DAC_CR_EN1;
}

/* =========================================================================
 * dac1_sample_rate_hz
 *
 * The sample rate TIM6 was programmed for, computed from the nominal 16 MHz.
 * It confirms the configuration (ARR), not the real rate: if the HSI runs
 * fast, the real rate runs fast with it and this number does not change.
 * ========================================================================= */
uint32_t dac1_sample_rate_hz(void)
{
    return s_sample_rate_hz;
}

/* =========================================================================
 * dac1_underrun
 *
 * Returns 1 if DMAUDR1 is set: a trigger arrived before the DMA had answered
 * the previous request. The DAC then keeps converting the old value and
 * issues no new DMA request (RM0390 Section 14.3.7), so the waveform stalls
 * until the DMA and the DAC are set up again, which dac1_start_waveform()
 * does. It should stay 0 at 64 kHz; a 1 means the DMA could not keep up, for
 * example because another stream on DMA1 was hogging the bus. The flag stays
 * set until software clears it.
 * ========================================================================= */
int dac1_underrun(void)
{
    return ((DAC->SR & DAC_SR_DMAUDR1) != 0U) ? 1 : 0;
}
