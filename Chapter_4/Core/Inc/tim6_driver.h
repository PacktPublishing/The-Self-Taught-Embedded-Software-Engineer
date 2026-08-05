/**
 ******************************************************************************
 * @file    tim6_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.4: TIM6 basic-timer blocking delay
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Public interface for a precise millisecond delay built on TIM6,
 *          one of the STM32F4 basic timers. TIM6 has only a prescaler, a
 *          counter and an auto-reload register — no output channels — which
 *          makes it the clearest peripheral for learning how a hardware timer
 *          turns a clock frequency into a measured interval.
 *
 *          This driver is polling-only: tim6_delay_ms() starts the timer and
 *          spins on the update flag. Interrupt-driven timing is Exercise 4.5.
 *
 *          Timing basis: the MCU runs on HSI at 16 MHz (see main.h), and with
 *          APB1 undivided the TIM6 counter clock is also 16 MHz.
 ******************************************************************************
 */

#ifndef TIM6_DRIVER_H
#define TIM6_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/**
 * @brief  Enable TIM6 and pre-scale its 16 MHz counter clock down to 1 kHz,
 *         so one counter tick is exactly one millisecond.
 *
 *         Call once before the first tim6_delay_ms().
 */
void tim6_init(void);

/**
 * @brief  Block for an exact number of milliseconds using TIM6.
 *
 * @param  ms  Delay length, 1..65536 ms. Values outside that range are
 *             clamped to the nearest valid auto-reload.
 */
void tim6_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* TIM6_DRIVER_H */
