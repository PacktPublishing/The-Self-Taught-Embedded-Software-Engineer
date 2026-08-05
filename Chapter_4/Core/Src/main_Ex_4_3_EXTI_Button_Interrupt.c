/**
 ******************************************************************************
 * @file    main_Ex_4_3_EXTI_Button_Interrupt.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.3: Toggle an LED from a button EXTI interrupt
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 user LED  → PA5  (output, via the Ex 4.2 GPIO driver)
 *          B1 user button → PC13 (EXTI line 13, falling edge)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Exercise 4.2 polled the button in the main loop. Here the CPU does
 *          no polling at all: it sits in an idle loop while EXTI line 13 fires
 *          an interrupt on each press. The interrupt sets a `volatile` flag;
 *          the main loop notices the flag, toggles LD2, and logs the event.
 *
 *          Why a flag instead of toggling the LED inside the ISR:
 *          keeping interrupt handlers short is the rule introduced here and
 *          reinforced for the rest of the book. The ISR records that an event
 *          happened; the slower work (printf over ITM) runs in thread context.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "exti_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define LED_PORT    GPIOA
#define LED_PIN     (5U)

/* Set in interrupt context, consumed in thread context: must be volatile. */
static volatile uint32_t s_press_events = 0U;

/* Callback handed to the EXTI driver — runs inside EXTI15_10_IRQHandler. */
static void on_button_press(void)
{
    s_press_events++;
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
    gpio_write_pin(LED_PORT, LED_PIN, GP_LOW);

    exti_pc13_init(on_button_press);

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 4 - Exercise 4.3: EXTI Button Interrupt\r\n");
    printf("Press B1 (PC13). The CPU never polls the pin.\r\n");

    uint32_t handled = 0U;

    while (1)
    {
        /* Consume exactly one event per pass rather than jumping straight
         * to the latest count. If the ISR fired several times while the
         * loop was busy, every one of those edges still produces its own
         * toggle and its own console line, so the numbering can never skip.
         * That honesty matters here: a mechanical button bounces, and one
         * physical press often produces more than one falling edge. Seeing
         * the extra interrupts is the point. Debouncing them is a separate
         * problem, and hiding them behind a counter that jumps would only
         * make the effect harder to notice.                                */
        if (handled != s_press_events)
        {
            handled++;
            gpio_toggle_pin(LED_PORT, LED_PIN);
            printf("Interrupt #%lu -> LD2 toggled\r\n", (unsigned long)handled);
        }
        else
        {
            __WFI();   /* nothing pending: sleep until the next interrupt   */
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
