/**
 ******************************************************************************
 * @file    cycle_counter.h
 * @author  Renato Soriano
 * @brief   Chapter 7 - DWT cycle counter helpers for timing measurements
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Chapter 6 timed its pipeline with the Cortex-M4 cycle counter
 *          (DWT->CYCCNT) inline in pipeline.c. Chapter 7 measures something
 *          in almost every exercise (a blocking transmit against a DMA
 *          transmit, a memcpy against a DMA copy, the period between two ADC
 *          blocks), so the same three register writes move into one small
 *          module that every exercise shares.
 *
 *          The counter increments once per core clock. At the reset-default
 *          16 MHz HSI that is 16 counts per microsecond, and the 32-bit
 *          counter wraps after about 268 seconds. Every elapsed-time helper
 *          below subtracts with unsigned arithmetic, so a single wrap between
 *          the two timestamps still produces the right answer.
 *
 *          Registers touched (ARMv7-M Architecture Reference Manual, DWT and
 *          CoreDebug, exposed by core_cm4.h):
 *              CoreDebug->DEMCR  TRCENA     enable the trace/DWT block
 *              DWT->CYCCNT                  the free-running cycle count
 *              DWT->CTRL         CYCCNTENA  start the counter
 ******************************************************************************
 */

#ifndef CYCLE_COUNTER_H
#define CYCLE_COUNTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Core clock cycles per microsecond at the book's 16 MHz HSI baseline. */
#define CYCLES_PER_US       (16UL)

/**
 * @brief  Enable the DWT block and start CYCCNT from zero.
 *         Safe to call more than once; later calls restart the count.
 */
void cycle_counter_init(void);

/**
 * @brief  Read the current cycle count.
 * @return Raw 32-bit CYCCNT value.
 */
uint32_t cycle_counter_now(void);

/**
 * @brief  Microseconds elapsed since a timestamp taken with
 *         cycle_counter_now(). Wrap-safe for spans under about 268 s.
 * @param  start  Earlier CYCCNT value.
 * @return Elapsed time in microseconds.
 */
uint32_t cycle_counter_elapsed_us(uint32_t start);

/**
 * @brief  Busy-wait for a number of microseconds.
 *
 *         This is a deliberate blocking delay. Exercise 7.3b uses it to
 *         recreate the 11 ms stall that the blocking BME280 read caused in
 *         Chapter 6, so the DMA receive path can be tested against it.
 *
 * @param  us  Microseconds to wait (keep below about 268,000,000).
 */
void cycle_counter_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* CYCLE_COUNTER_H */
