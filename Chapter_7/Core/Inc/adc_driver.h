/**
 ******************************************************************************
 * @file    adc_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 7 - ADC1 driver: single, continuous, and timer-triggered
 *          scan conversions with DMA
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   PA0 = ADC1_IN0 (Arduino A0, potentiometer wiper)
 *          PA4 = ADC1_IN4 (shared with DAC_OUT1, used for loopback in 7.5)
 *          Internal: ADC1_IN17 = VREFINT, ADC1_IN18 = temperature sensor
 *
 * @details One driver, three ways to start a conversion, matching the three
 *          modes Section 7.4 walks through:
 *
 *            adc1_read_single()       software start, wait for EOC, read DR.
 *            adc1_start_continuous()  CONT = 1: the ADC restarts itself after
 *                                     every conversion and the CPU must read
 *                                     DR before the next result lands.
 *            adc1_start_timer_dma()   TIM2 TRGO starts one scan of the whole
 *                                     sequence per timer period, and DMA2
 *                                     Stream 0 copies each result to memory.
 *                                     The sample rate is set by the timer,
 *                                     not by the code that reads the data.
 *
 *          Clocking. ADC1 sits on APB2 (16 MHz at reset) and the common
 *          prescaler divides it by 2, so ADCCLK is 8 MHz, inside the 36 MHz
 *          limit in DS10693 Table 74. One conversion takes the programmed
 *          sampling time plus 12 ADCCLK cycles for a 12-bit result
 *          (RM0390 Section 13.5).
 *
 *          The internal channels need long sampling times. DS10693 Tables 80
 *          and 83 ask for at least 10 us on the temperature sensor and on
 *          VREFINT. At 8 MHz that rules out everything below 84 cycles
 *          (10.5 us); this chapter uses 480 cycles (60 us) for both.
 *
 *          Registers touched (RM0390 Section 13.13):
 *              RCC_APB2ENR     ADC1EN
 *              ADC_CCR         ADCPRE, TSVREFE          (common registers)
 *              ADC_CR1         SCAN
 *              ADC_CR2         ADON, CONT, DMA, DDS, SWSTART, EXTSEL, EXTEN
 *              ADC_SMPR1/2     sampling time per channel
 *              ADC_SQR1/2/3    sequence length and order
 *              ADC_SR / ADC_DR status and result
 *              TIM2 PSC, ARR, CR2.MMS = update, the conversion trigger
 ******************************************************************************
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "dma_driver.h"
#include <stdint.h>

typedef enum
{
    ADC_OK = 0,
    ADC_ERR_PARAM,
    ADC_ERR_TIMEOUT
} adc_status_t;

/** Sampling time in ADCCLK cycles, the 3-bit SMPx code (RM0390 13.13.4). */
typedef enum
{
    ADC_SMP_3   = 0U,
    ADC_SMP_15  = 1U,
    ADC_SMP_28  = 2U,
    ADC_SMP_56  = 3U,
    ADC_SMP_84  = 4U,
    ADC_SMP_112 = 5U,
    ADC_SMP_144 = 6U,
    ADC_SMP_480 = 7U
} adc_sample_time_t;

#define ADC_CH_POT         (0U)    /* PA0  */
#define ADC_CH_DAC_PIN     (4U)    /* PA4  */
#define ADC_CH_VREFINT     (17U)   /* internal reference, needs TSVREFE */
#define ADC_CH_TEMP        (18U)   /* temperature sensor, needs TSVREFE */

#define ADC_FULL_SCALE     (4095U) /* 12-bit right-aligned result */

/* Factory calibration values, written by ST at VDDA = 3.3 V
 * (DS10693 Tables 81 and 84). Read-only system memory. */
#define ADC_VREFINT_CAL_ADDR   (0x1FFF7A2AUL)   /* VREFINT raw at 30 C        */
#define ADC_TS_CAL1_ADDR       (0x1FFF7A2CUL)   /* temp sensor raw at 30 C    */
#define ADC_TS_CAL2_ADDR       (0x1FFF7A2EUL)   /* temp sensor raw at 110 C   */
#define ADC_CAL_VDDA_MV        (3300UL)

/** One entry of the regular conversion sequence. */
typedef struct
{
    uint8_t           channel;   /* 0-18 */
    adc_sample_time_t smp;
} adc_channel_cfg_t;

/**
 * @brief  Clock ADC1, set ADCCLK = PCLK2 / 2, 12-bit right-aligned results,
 *         and power the converter on. Leaves no conversion running.
 *         The caller configures the analog pins (GP_MODE_ANALOG).
 */
adc_status_t adc1_init(void);

/** @brief Wake the temperature sensor and VREFINT (ADC_CCR.TSVREFE). */
void adc1_enable_internal_channels(void);

/**
 * @brief  Program the regular sequence: order, length, and the sampling time
 *         of each channel. SCAN is enabled when there is more than one entry.
 * @param  count  1 to 16.
 */
adc_status_t adc1_set_sequence(const adc_channel_cfg_t *list, uint8_t count);

/**
 * @brief  Software-start one conversion of the sequence and wait for it.
 *         Intended for a one-channel sequence: with several channels and no
 *         DMA, only the last result of the scan survives in DR.
 */
adc_status_t adc1_read_single(uint16_t *out);

/** @brief Start continuous conversion (CONT = 1) of the sequence. */
adc_status_t adc1_start_continuous(void);

/**
 * @brief  Non-blocking check for a new continuous-mode result.
 * @return 1 and the value in @p out if EOC was set, otherwise 0.
 */
int adc1_poll_result(uint16_t *out);

/** @brief Stop continuous conversion. */
void adc1_stop_continuous(void);

/**
 * @brief  Start timer-triggered sampling of the whole sequence into memory.
 *
 * @param  buf       Destination, @p count half-words. Must outlive the run.
 * @param  count     Number of results, a multiple of the sequence length.
 * @param  rate_hz   Sequence scans per second (TIM2 update rate).
 * @param  circular  1: DMA loops forever, HT and TC both reported (double
 *                   buffering). 0: one block, TC reported, then the ADC
 *                   stops raising DMA requests.
 * @param  cb        DMA callback, interrupt context. May be NULL to poll.
 */
adc_status_t adc1_start_timer_dma(uint16_t *buf, uint16_t count, uint32_t rate_hz,
                                  uint8_t circular, dma_callback_t cb);

/** @brief Stop the trigger timer, the DMA stream, and ADC DMA requests. */
void adc1_stop_timer_dma(void);

/** @return The rate TIM2 actually runs at (integer division of 16 MHz). */
uint32_t adc1_trigger_rate_hz(void);

/** @return 1 if the ADC reported an overrun (a result was lost), else 0. */
int adc1_overrun(void);

/** @brief Analog supply in mV, from a VREFINT reading and its calibration. */
uint32_t adc1_vdda_mv(uint16_t vrefint_raw);

/** @brief Convert a raw result to millivolts for a given VDDA. */
uint32_t adc1_raw_to_mv(uint16_t raw, uint32_t vdda_mv);

/** @brief Junction temperature in hundredths of a degree Celsius. */
int32_t adc1_temp_centi_c(uint16_t ts_raw, uint32_t vdda_mv);

#ifdef __cplusplus
}
#endif

#endif /* ADC_DRIVER_H */
