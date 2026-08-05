/**
 ******************************************************************************
 * @file    main_Ex_4_4_TIM6_Delay.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.4: Precise 1-second LED blink with TIM6
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 user LED → PA5 (output, via the Ex 4.2 GPIO driver)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Exercise 4.1 blinked the LED with a guessed software loop whose
 *          period drifts with compiler flags and clock speed. This exercise
 *          replaces that guess with TIM6, a hardware timer clocked from the
 *          same 16 MHz source, so the blink period is an exact 1000 ms that
 *          can be checked on a scope.
 *
 *          The main loop toggles LD2 and then blocks in tim6_delay_ms(1000).
 *          A counter is printed each second over the SWV console so the timing
 *          is visible without instruments.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "tim6_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define LED_PORT    GPIOA
#define LED_PIN     (5U)

#define BLINK_PERIOD_MS   (1000U)

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

    tim6_init();

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 4 - Exercise 4.4: TIM6 Precise Delay\r\n");
    printf("LD2 toggles every %u ms (hardware-timed).\r\n", BLINK_PERIOD_MS);

    uint32_t seconds = 0U;

    while (1)
    {
        gpio_toggle_pin(LED_PORT, LED_PIN);
        printf("tick: %lu s\r\n", (unsigned long)seconds);
        seconds++;

        tim6_delay_ms(BLINK_PERIOD_MS);
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
