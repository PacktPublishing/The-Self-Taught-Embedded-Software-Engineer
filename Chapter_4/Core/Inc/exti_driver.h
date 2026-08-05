/**
 ******************************************************************************
 * @file    exti_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.3: External-interrupt (EXTI) driver for the
 *          B1 user button on PC13
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Public interface for an event-driven button. Instead of polling
 *          PC13 in the main loop (Exercise 4.2), the pin is wired to EXTI
 *          line 13, which raises an interrupt on the falling edge produced
 *          when B1 is pressed.
 *
 *          The driver keeps the wiring details — SYSCFG line mapping, edge
 *          selection, NVIC enable — behind a single init call, and lets the
 *          application supply the work to run on each press as a callback.
 *          That callback is invoked from interrupt context, so it must be
 *          short and must only touch variables qualified `volatile`.
 ******************************************************************************
 */

#ifndef EXTI_DRIVER_H
#define EXTI_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

/** Signature of a user callback run from the EXTI13 interrupt handler. */
typedef void (*exti_callback_t)(void);

/**
 * @brief  Configure PC13 as a falling-edge external interrupt source.
 *
 *         Steps performed (all register-level, RM0390):
 *           1. Configure PC13 as a pulled-up input, via the Exercise 4.2
 *              GPIO driver, which also enables the GPIOC clock.
 *           2. Enable the SYSCFG clock on APB2.
 *           3. Route EXTI13 to port C via SYSCFG_EXTICR4.
 *           4. Unmask EXTI13 and select the falling-edge trigger.
 *           5. Enable the shared EXTI15_10 vector in the NVIC.
 *
 * @param  cb  Function to run on each press. May be NULL (interrupt still
 *             fires and clears, but does no application work).
 */
void exti_pc13_init(exti_callback_t cb);

#ifdef __cplusplus
}
#endif

#endif /* EXTI_DRIVER_H */
