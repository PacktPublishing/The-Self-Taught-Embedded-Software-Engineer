/**
 ******************************************************************************
 * @file    main_Ex_4_7_IWDG_Watchdog.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.7: Recover from a stalled main loop with the
 *          Independent Watchdog
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 user LED → PA5 (output, via the Ex 4.2 GPIO driver)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Demonstrates the watchdog doing its job. On boot the program reports
 *          whether the previous reset was caused by the IWDG, then starts the
 *          watchdog with a ~1 s timeout and enters a loop that blinks LD2 and
 *          feeds the watchdog each pass.
 *
 *          After a fixed number of healthy iterations the program deliberately
 *          stops feeding the watchdog and spins forever — simulating a firmware
 *          lockup. About one second later the IWDG resets the MCU, execution
 *          restarts at main(), and this time iwdg_caused_last_reset() returns
 *          true, proving the recovery path.
 *
 *          Timing: LSI ~32 kHz, prescaler /32 -> 1 kHz watchdog tick;
 *          reload 999 -> ~1.0 s timeout.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "iwdg_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define LED_PORT    GPIOA
#define LED_PIN     (5U)

/* Healthy iterations before we simulate a hang. */
#define HEALTHY_ITERATIONS  (10U)

/* Rough software delay (~200 ms at 16 MHz) so the blink is visible. */
#define VISIBLE_DELAY_LOOPS (800000UL)

static void crude_delay(volatile uint32_t loops)
{
    while (loops > 0UL)
    {
        loops--;
    }
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

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 4 - Exercise 4.7: Independent Watchdog\r\n");

    if (iwdg_caused_last_reset() != 0U)
    {
        printf(">> Previous reset was caused by the IWDG (recovery worked).\r\n");
    }
    else
    {
        printf(">> Power-on / normal reset.\r\n");
    }

    /* ~1 s timeout: divider 32, reload 999. */
    iwdg_start(IWDG_PRE_32, 999U);
    printf("IWDG started (~1 s). Feeding it for %u iterations...\r\n",
           HEALTHY_ITERATIONS);

    uint32_t i = 0U;

    while (1)
    {
        gpio_toggle_pin(LED_PORT, LED_PIN);

        if (i < HEALTHY_ITERATIONS)
        {
            iwdg_refresh();
            printf("iteration %lu: fed the watchdog\r\n", (unsigned long)i);
            i++;
            crude_delay(VISIBLE_DELAY_LOOPS);
        }
        else
        {
            /* Simulated lockup: stop feeding and hang. The IWDG will reset
             * the MCU in ~1 s.                                             */
            printf("Main loop STALLED - no longer feeding the watchdog.\r\n");
            printf("Waiting for the IWDG to reset the MCU...\r\n");
            while (1)
            {
                /* deliberately stuck */
            }
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
