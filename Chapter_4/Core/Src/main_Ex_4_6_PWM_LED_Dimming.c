/**
 ******************************************************************************
 * @file    main_Ex_4_6_PWM_LED_Dimming.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.6: Dim an LED smoothly with PWM
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 user LED → PA5 (TIM2_CH1, AF1)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Exercise 4.5 switched the LED fully on and off. Here the LED is
 *          never switched by software at all: TIM2 generates a 1 kHz PWM wave
 *          on PA5, and the program only changes the duty cycle. Ramping the
 *          duty up from 0 to 100% and back produces a smooth "breathing"
 *          brightness the eye reads as analog dimming, even though the pin is
 *          still purely digital.
 *
 *          The ramp uses a crude software delay between brightness steps; the
 *          PWM frequency that actually drives the LED comes from the hardware
 *          timer, independent of this loop's timing.
 ******************************************************************************
 */

#include "main.h"
#include "pwm_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* Software delay between 1% brightness steps (~8 ms at 16 MHz). */
#define STEP_DELAY_LOOPS    (32000UL)

static void crude_delay(volatile uint32_t loops)
{
    while (loops > 0UL)
    {
        loops--;
    }
}

int main(void)
{
    pwm_led_init();

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 4 - Exercise 4.6: PWM LED Dimming\r\n");
    printf("LD2 breathes via TIM2_CH1 at 1 kHz PWM.\r\n");

    while (1)
    {
        /* Fade in: 0 -> 100%. */
        for (uint8_t duty = 0U; duty <= 100U; duty++)
        {
            pwm_led_set_duty(duty);
            crude_delay(STEP_DELAY_LOOPS);
        }
        printf("peak brightness (100%%)\r\n");

        /* Fade out: 100 -> 0%. */
        for (uint8_t duty = 100U; duty > 0U; duty--)
        {
            pwm_led_set_duty(duty);
            crude_delay(STEP_DELAY_LOOPS);
        }
        pwm_led_set_duty(0U);
        printf("off (0%%)\r\n");
    }

    return 0;
}

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
        /* spin */
    }
}
