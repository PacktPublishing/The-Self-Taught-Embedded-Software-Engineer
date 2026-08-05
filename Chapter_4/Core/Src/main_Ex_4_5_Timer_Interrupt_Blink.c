/**
 ******************************************************************************
 * @file    main_Ex_4_5_Timer_Interrupt_Blink.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.5: Blink an LED at a configurable rate using
 *          timer interrupts
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 user LED → PA5 (output, via the Ex 4.2 GPIO driver)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details The LED is toggled entirely from the TIM3 update interrupt, so the
 *          blink rate is hardware-defined and the main loop never busy-waits.
 *          To show the rate is configurable, the program steps the period
 *          through a small table (1000, 500, 250, 100 ms), changing it every
 *          twenty toggles via tim3_set_period_ms().
 *
 *          ISR/main split:
 *          - The interrupt callback toggles LD2 (one register write) and bumps
 *            a `volatile` toggle counter — short, as an ISR should be.
 *          - The main loop watches the counter, logs each toggle, and adjusts
 *            the period. All printf work stays in thread context.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "tim3_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define LED_PORT    GPIOA
#define LED_PIN     (5U)

/* Shared with the ISR -> must be volatile. */
static volatile uint32_t s_toggles = 0U;

/* Blink periods cycled through to prove the rate is configurable. */
static const uint32_t k_periods_ms[] = { 1000U, 500U, 250U, 100U };
#define PERIOD_COUNT    (sizeof(k_periods_ms) / sizeof(k_periods_ms[0]))
#define TOGGLES_PER_STEP    (20U)

/* Runs inside TIM3_IRQHandler: keep it to the bare minimum. */
static void on_tim3_tick(void)
{
    gpio_toggle_pin(LED_PORT, LED_PIN);
    s_toggles++;
}

int main(void)
{
    const gpio_config_t led_cfg = {
        .pin   = LED_PIN,
        .mode  = GP_MODE_OUTPUT,
        .otype = GP_OTYPE_PUSHPULL,
        .pull  = GP_PULL_NONE,
        .af    = GP_AF_NONE
    };
    gpio_init(LED_PORT, &led_cfg);

    uint32_t step = 0U;
    tim3_periodic_init(k_periods_ms[step], on_tim3_tick);

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 4 - Exercise 4.5: Timer-Interrupt Blink\r\n");
    printf("Period now %lu ms\r\n", (unsigned long)k_periods_ms[step]);

    uint32_t handled = 0U;

    while (1)
    {
        /* Consume one tick per pass, the same pattern Exercise 4.3 uses,
         * so a step boundary can never be stepped over if the loop was
         * busy when two interrupts arrived close together.                */
        if (handled != s_toggles)
        {
            handled++;

            /* Every TOGGLES_PER_STEP flips, speed the blink up a notch. */
            if ((handled % TOGGLES_PER_STEP) == 0U)
            {
                step = (step + 1U) % PERIOD_COUNT;
                tim3_set_period_ms(k_periods_ms[step]);
                printf("Period now %lu ms\r\n",
                       (unsigned long)k_periods_ms[step]);
            }
        }
        else
        {
            __WFI();   /* sleep until the next timer interrupt             */
        }
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
