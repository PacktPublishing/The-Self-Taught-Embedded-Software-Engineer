/**
 ******************************************************************************
 * @file    tim3_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.5: TIM3 periodic update-interrupt driver
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Public interface for a periodic interrupt source built on TIM3, a
 *          general-purpose timer. Unlike the polling delay of Exercise 4.4,
 *          here the timer raises an interrupt every N milliseconds and the CPU
 *          is free to sleep between events. The application supplies the work
 *          to run on each tick as a callback, invoked from the TIM3 ISR.
 *
 *          Timing basis: HSI 16 MHz, APB1 undivided, so the TIM3 counter clock
 *          is 16 MHz. The driver pre-scales that to a 1 kHz (1 ms) tick and
 *          uses the auto-reload value to set the interrupt period in ms.
 ******************************************************************************
 */

#ifndef TIM3_DRIVER_H
#define TIM3_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/** Signature of the per-tick callback run from the TIM3 interrupt handler. */
typedef void (*tim3_tick_cb_t)(void);

/**
 * @brief  Start TIM3 raising an update interrupt every @p period_ms.
 *
 * @param  period_ms  Interrupt period, 1..65536 ms.
 * @param  cb         Work to run each period (interrupt context, keep short).
 *                    May be NULL.
 */
void tim3_periodic_init(uint32_t period_ms, tim3_tick_cb_t cb);

/**
 * @brief  Change the interrupt period at run time without re-initialising.
 *
 * @param  period_ms  New period, 1..65536 ms.
 */
void tim3_set_period_ms(uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* TIM3_DRIVER_H */
