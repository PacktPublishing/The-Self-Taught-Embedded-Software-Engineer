/**
 ******************************************************************************
 * @file    main_Ex_4_2_GPIO_Driver_API.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.2: Minimal register-level GPIO driver
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 user LED  → PA5  (output)
 *          B1 user button → PC13 (input, active-low: pressed = 0)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Exercise 4.1 drove PA5 with inline register writes. This exercise
 *          replaces those writes with calls to the driver in gpio_driver.c and
 *          then proves the API does more than blink: it polls the user button
 *          on PC13 and toggles LD2 on each fresh press.
 *
 *          Clocking: register-level, no HAL. The MCU runs on the reset-default
 *          HSI clock at 16 MHz (see main.h). The button is sampled in a simple
 *          software-debounced poll loop — interrupt-driven input is the subject
 *          of Exercise 4.3.
 *
 *          What this exercise demonstrates:
 *          - gpio_init() configuring a pin from a settings struct
 *          - gpio_read_pin() reading a button with an internal pull-up
 *          - gpio_toggle_pin() / gpio_write_pin() driving an LED
 *          - The same driver working across two different ports (A and C)
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* Board pin map for the Nucleo-F446RE ----------------------------------- */
#define LED_PORT        GPIOA
#define LED_PIN         (5U)     /* LD2  */
#define BTN_PORT        GPIOC
#define BTN_PIN         (13U)    /* B1, active-low on this board */

/* Crude debounce: number of poll iterations the button must read stable. */
#define DEBOUNCE_COUNT  (2000U)

/* Private prototypes ----------------------------------------------------- */
static void board_gpio_init(void);
static void crude_delay(volatile uint32_t loops);

/* =========================================================================
 * board_gpio_init — configure LD2 as output and B1 as a pulled-up input
 * ========================================================================= */
static void board_gpio_init(void)
{
    const gpio_config_t led_cfg = {
        .pin   = LED_PIN,
        .mode  = GP_MODE_OUTPUT,
        .otype = GP_OTYPE_PUSHPULL,
        .pull  = GP_PULL_NONE,
        .af    = GP_AF_NONE
    };

    /* B1 on the Nucleo board connects PC13 to ground when pressed, so the
     * pin needs a pull-up to read a definite HIGH while the button is
     * released. Enabling the internal one costs nothing and makes the
     * intent explicit, which is why Exercise 4.3 configures it the same
     * way rather than relying on whatever the board happens to provide.
     * The pin then reads HIGH when released and LOW when pressed.        */
    const gpio_config_t btn_cfg = {
        .pin   = BTN_PIN,
        .mode  = GP_MODE_INPUT,
        .otype = GP_OTYPE_PUSHPULL,   /* not written for inputs          */
        .pull  = GP_PULL_UP,
        .af    = GP_AF_NONE
    };

    gpio_init(LED_PORT, &led_cfg);
    gpio_init(BTN_PORT, &btn_cfg);
}

/* =========================================================================
 * crude_delay — volatile busy-wait so the debounce window is observable
 * ========================================================================= */
static void crude_delay(volatile uint32_t loops)
{
    while (loops > 0UL)
    {
        loops--;
    }
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    board_gpio_init();

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 4 - Exercise 4.2: GPIO Driver API\r\n");
    printf("Press B1 (PC13) to toggle LD2 (PA5).\r\n");

    /* Start with the LED off so the first press is unambiguous.          */
    gpio_write_pin(LED_PORT, LED_PIN, GP_LOW);

    gpio_level_t prev = GP_HIGH;   /* released state with the pull-up    */

    while (1)
    {
        const gpio_level_t now = gpio_read_pin(BTN_PORT, BTN_PIN);

        /* Detect a falling edge: released (HIGH) -> pressed (LOW).        */
        if ((prev == GP_HIGH) && (now == GP_LOW))
        {
            gpio_toggle_pin(LED_PORT, LED_PIN);
            printf("B1 pressed -> LD2 toggled\r\n");
            crude_delay(DEBOUNCE_COUNT);   /* swallow contact bounce       */
        }

        prev = now;
    }

    return 0;
}

/* =========================================================================
 * Error_Handler — shared trap (declared in main.h)
 * ========================================================================= */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
        /* spin so a debugger can inspect the state */
    }
}
