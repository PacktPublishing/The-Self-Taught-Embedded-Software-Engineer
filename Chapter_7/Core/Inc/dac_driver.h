/**
 ******************************************************************************
 * @file    dac_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 7 - DAC channel 1 driver: static level and DMA waveform
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   DAC_OUT1 = PA4 (Arduino A2). DAC_OUT2 would be PA5, which is LD2,
 *          so this book uses channel 1 only.
 *
 * @details Two ways to drive the DAC, matching the two ideas in Section 7.5:
 *
 *            dac1_write()            write a 12-bit code and the output moves
 *                                    one APB1 cycle later. One level at a time,
 *                                    paced by software.
 *            dac1_start_waveform()   TIM6 TRGO paces the DAC. Each trigger
 *                                    moves DHR to DOR and raises a DMA request,
 *                                    and DMA1 Stream 5 refills DHR from a table
 *                                    in memory. In circular mode the table
 *                                    plays forever with the CPU doing nothing.
 *
 *          Output range. With the output buffer on (the default here),
 *          DS10693 Table 85 guarantees the output only from 0.2 V to
 *          VDDA - 0.2 V. Codes closer to 0 or 4095 than that are outside the
 *          guaranteed range, even if a lightly loaded board gets closer to
 *          the rails. The waveform table in Exercise 7.5 is scaled to stay
 *          inside the guaranteed window.
 *
 *          Shared stream. DAC1 uses DMA1 Stream 5 channel 7, the same stream
 *          USART2_RX uses on channel 4 (RM0390 Table 28). Do not run this
 *          waveform and a USART2 DMA receive in the same program.
 *
 *          Registers touched (RM0390 Section 14.5):
 *              RCC_APB1ENR   DACEN, TIM6EN
 *              DAC_CR        EN1, BOFF1, TEN1, TSEL1, DMAEN1
 *              DAC_DHR12R1   12-bit right-aligned data holding register
 *              DAC_SR        DMAUDR1, set if a trigger finds no new data
 *              TIM6 PSC, ARR, CR2.MMS = update, the conversion trigger
 ******************************************************************************
 */

#ifndef DAC_DRIVER_H
#define DAC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "dma_driver.h"
#include <stdint.h>

typedef enum
{
    DAC_OK = 0,
    DAC_ERR_PARAM,
    DAC_ERR_TIMEOUT
} dac_status_t;

#define DAC_MAX_CODE    (4095U)

/**
 * @brief  Configure PA4 as analog, clock the DAC, enable channel 1 with its
 *         output buffer, and output code 0.
 */
dac_status_t dac1_init(void);

/** @brief Output one level now (no trigger). Codes above 4095 are clamped. */
void dac1_write(uint16_t code);

/**
 * @brief  Play a table of codes forever, one entry per TIM6 period.
 *
 * @param  table        12-bit codes, one per 32-bit word. RM0390 Section 14.5
 *                      says the DAC registers must be accessed as 32-bit
 *                      words, so the DMA moves words and the table is
 *                      uint32_t even though every value fits in 12 bits.
 *                      It may live in Flash: the DMA memory port can read
 *                      internal Flash (RM0390 Section 2.1.4).
 * @param  count        Entries in one period of the waveform.
 * @param  sample_rate  Entries per second. The output frequency is
 *                      sample_rate / count. TIM6 is 16-bit at 16 MHz, so the
 *                      rate must be at least 245 Hz; the DAC updates at most
 *                      1 MS/s (DS10693 Table 85), so it must be at most
 *                      1 MHz. Outside that range DAC_ERR_PARAM is returned.
 * @param  cb           Optional DMA callback (interrupt context), told about
 *                      each half and each full pass of the table.
 */
dac_status_t dac1_start_waveform(const uint32_t *table, uint16_t count,
                                 uint32_t sample_rate, dma_callback_t cb);

/** @brief Stop the timer and the DMA; the output holds its last level. */
void dac1_stop_waveform(void);

/** @return The rate TIM6 actually runs at. */
uint32_t dac1_sample_rate_hz(void);

/** @return 1 if a trigger found no fresh sample (DMA underrun), else 0. */
int dac1_underrun(void);

#ifdef __cplusplus
}
#endif

#endif /* DAC_DRIVER_H */
