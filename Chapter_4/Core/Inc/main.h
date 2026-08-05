/**
 ******************************************************************************
 * @file    main.h
 * @author  Renato Soriano
 * @brief   Chapter 4 — Shared minimal main.h header
 *
 * @details The Chapter 4 driver exercises are register-level: no STM32 HAL
 *          drivers and no CubeMX clock configuration are used. The MCU runs
 *          on its reset-default clock — the internal HSI oscillator at
 *          16 MHz, which is already running and selected as SYSCLK when
 *          main() begins. Every timer calculation in this chapter is derived
 *          from that 16 MHz figure.
 *
 *          Because no HAL is linked, this header pulls in only the CMSIS
 *          device header (stm32f4xx.h), which gives us the typed peripheral
 *          structs (RCC, GPIOA, TIM6, IWDG, ...) and their bit masks.
 ******************************************************************************
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

/** Reset-default core/bus clock used by every Chapter 4 timing calculation. */
#define SYSTEM_CLOCK_HZ     (16000000UL)   /* HSI, 16 MHz */

/**
 * @brief  Trap on an unrecoverable error. Disables interrupts and spins so a
 *         debugger can inspect the call stack.
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
