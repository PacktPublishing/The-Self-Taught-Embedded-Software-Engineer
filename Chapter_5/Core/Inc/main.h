/**
 ******************************************************************************
 * @file    main.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Shared minimal main.h header
 *
 * @details The Chapter 5 communication exercises are register-level: no STM32
 *          HAL drivers and no CubeMX clock configuration are used. The MCU runs
 *          on its reset-default clock — the internal HSI oscillator at 16 MHz,
 *          already running and selected as SYSCLK when main() begins.
 *
 *          At reset the AHB, APB1 and APB2 prescalers are all 1, so every bus
 *          clock equals SYSCLK = 16 MHz. USART2, SPI2 and I2C1 all live on
 *          APB1, so their baud/clock maths in this chapter is derived from
 *          APB1_CLOCK_HZ below.
 *
 *          Only the CMSIS device header is pulled in (no HAL), giving the typed
 *          peripheral structs (RCC, GPIOA/B, USART2, SPI2, I2C1, ...) and bit
 *          masks used by the drivers.
 ******************************************************************************
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

/** Reset-default core/bus clock (HSI). */
#define SYSTEM_CLOCK_HZ     (16000000UL)
/** APB1 peripheral clock — bus for USART2, SPI2 and I2C1 (PPRE1 = /1 at reset). */
#define APB1_CLOCK_HZ       (16000000UL)
/** APB2 peripheral clock — kept for reference (SYSCFG, USART1/6, SPI1). */
#define APB2_CLOCK_HZ       (16000000UL)

/**
 * @brief  Trap on an unrecoverable error. Disables interrupts and spins so a
 *         debugger can inspect the call stack.
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
