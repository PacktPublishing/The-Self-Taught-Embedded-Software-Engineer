/**
 ******************************************************************************
 * @file    pwm_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.6: PWM driver for LED dimming on TIM2_CH1
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Public interface for generating a PWM signal on PA5 (the LD2 user
 *          LED) using channel 1 of TIM2. PWM produces an analog-like average
 *          voltage by switching the pin on and off quickly and varying the
 *          fraction of time it stays on (the duty cycle). At a switching
 *          frequency well above the eye's flicker-fusion threshold, the LED
 *          simply looks brighter or dimmer.
 *
 *          PA5 is routed to TIM2_CH1 through alternate function AF1. The driver
 *          configures the pin and the timer, then exposes a single call to set
 *          the duty cycle as a 0-100 percentage.
 *
 *          Frequency: TIM2 counter clock 16 MHz, prescaler /16 -> 1 MHz,
 *          auto-reload 1000 -> 1 kHz PWM, far above any visible flicker.
 ******************************************************************************
 */

#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/** Auto-reload value: 1000 timer steps per PWM period. The hardware can
 *  therefore resolve 0.1% of a period, even though pwm_led_set_duty()
 *  deliberately exposes only whole percentages. */
#define PWM_RESOLUTION_STEPS    (1000U)

/**
 * @brief  Configure PA5 (TIM2_CH1, AF1) and TIM2 for 1 kHz PWM output.
 *         Starts with a 0% duty cycle (LED off).
 */
void pwm_led_init(void);

/**
 * @brief  Set the LED brightness as a duty-cycle percentage.
 *
 * @param  percent  0 (off) to 100 (full brightness). Values above 100 are
 *                  clamped.
 */
void pwm_led_set_duty(uint8_t percent);

#ifdef __cplusplus
}
#endif

#endif /* PWM_DRIVER_H */
