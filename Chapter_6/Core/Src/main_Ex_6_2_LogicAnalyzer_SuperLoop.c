/**
 ******************************************************************************
 * @file    main_Ex_6_2_LogicAnalyzer_SuperLoop.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.2: a cooperative super-loop that blinks an
 *          LED, runs a five-state button machine, and echoes UART - none of
 *          them blocking the others   [Logic-analyzer / USART6 build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 = PA5 (blink) | B1 = PC13 (EXTI) | USART6 PC6/PC7 -> morpho header (probe PC6)
 * @output  A serial terminal on USART6 PC6/PC7 (probe PC6 with the analyzer), 115200 8N1
 *
 * @details This is the first program in the book where several independent jobs
 *          share one CPU without an RTOS. The trick is that no job is allowed to
 *          wait: each is a short step that runs when it is due and then returns.
 *
 *          Three cooperating subsystems, all driven from the 1 ms tick:
 *            - Blink : toggles LD2 every blink_period_ms using sched_due().
 *            - Button: an explicit five-state machine (IDLE, DEBOUNCE, PRESSED,
 *                      HELD, RELEASED) in button_fsm.c. It debounces without
 *                      delaying and tells a short press from a long one. The
 *                      EXTI ISR still does no real work: it only wakes the
 *                      machine out of IDLE. Build with -DBUTTON_TRACE=1 to
 *                      print every transition and watch it walk its diagram.
 *            - Echo  : drains whatever the interrupt-driven USART6 receiver has
 *                      queued and echoes it straight back, one non-blocking
 *                      getchar at a time.
 *
 *          Because every step is short and returns quickly, the blink stays
 *          even while you type and the echo stays responsive while the LED
 *          blinks. That is the whole lesson of Section 6.2.
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "exti_driver.h"
#include "scheduler.h"
#include "button_fsm.h"
#include "uart_driver_isr.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define LED_PORT        GPIOA
#define LED_PIN         (5U)          /* LD2 */
/* Per-transition tracing is OFF by default.
 *
 * Each trace line is about 29 bytes, and uart2_write_string() spins on TXE,
 * so one line costs roughly 2.5 ms of blocking inside a loop this chapter
 * spends its time telling you never to block. Useful for watching the machine
 * walk its diagram, but it is instrumentation, not behavior: turn it on
 * deliberately, and know what it costs. Build with -DBUTTON_TRACE=1.        */
#ifndef BUTTON_TRACE
#define BUTTON_TRACE    (0)
#endif

#define BTN_PORT        GPIOC
#define BTN_PIN         (13U)         /* B1, active low, pulled up by EXTI init */

/* Blink rates the button cycles through, in milliseconds. */
static const uint32_t k_rates_ms[] = { 1000U, 500U, 200U, 100U };
#define NUM_RATES       (sizeof(k_rates_ms) / sizeof(k_rates_ms[0]))

/* Set in the EXTI ISR, consumed (and cleared) by the main loop. */
static volatile uint8_t s_button_flag = 0U;

static void on_button_press(void)
{
    s_button_flag = 1U;               /* keep the ISR to a single store */
}

static void led_init(void)
{
    const gpio_config_t led = {
        .pin = LED_PIN, .mode = GP_MODE_OUTPUT,
        .otype = GP_OTYPE_PUSHPULL, .pull = GP_PULL_NONE
    };
    gpio_init(LED_PORT, &led);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    led_init();
    scheduler_init();                 /* 1 ms time base (TIM3)           */
    exti_pc13_init(on_button_press);  /* B1 -> flag only                 */

    const uart_config_t uart = { .baud = 115200U };
    usart6_init(&uart);

    usart6_write_string("\r\nChapter 6 - Exercise 6.2: cooperative super-loop (logic-analyzer)\r\n");
    usart6_write_string("Type to echo. Tap B1 to change the blink rate; hold it to pause.\r\n");

    uint32_t rate_index    = 0U;
    uint32_t blink_period  = k_rates_ms[rate_index];
    uint32_t last_blink    = 0U;
    uint8_t  blink_paused  = 0U;      /* toggled by a long press */

    button_fsm_t button;
    button_fsm_init(&button);
#if BUTTON_TRACE
    btn_state_t last_state = button.state;
#endif

    while (1)
    {
        const uint32_t now = millis();

        /* --- Subsystem 1: non-blocking blink ------------------------------ */
        if ((blink_paused == 0U) && sched_due(&last_blink, blink_period))
        {
            gpio_toggle_pin(LED_PORT, LED_PIN);
        }

        /* --- Subsystem 2: button, as an explicit five-state machine -------
         * The EXTI ISR still does nothing but set a flag. That flag is what
         * wakes the machine out of IDLE; from there the machine tracks the pin
         * level itself, because it must also see the release, and a
         * falling-edge interrupt never reports one.                         */
        uint8_t edge = 0U;
        if (s_button_flag != 0U)
        {
            s_button_flag = 0U;       /* consume the edge the ISR recorded */
            edge = 1U;
        }

        /* Read the pin only when it can actually matter. While the machine
         * sits in IDLE there is nothing to track, so the loop touches no GPIO
         * at all until the EXTI edge says a press has begun. From DEBOUNCE
         * onward the machine needs levels, not edges, because it has to see
         * the release too, and an edge-triggered interrupt never reports it.
         * That split is what the interrupt is for: it removes the polling
         * cost from the common case.
         *
         * B1 is wired active low: the pin reads 0 while the button is down. */
        uint8_t is_down = 0U;
        if ((button.state != BTN_IDLE) || (edge != 0U))
        {
            is_down = (gpio_read_pin(BTN_PORT, BTN_PIN) == GP_LOW) ? 1U : 0U;
        }

        const btn_event_t ev = button_fsm_update(&button, is_down, now);

#if BUTTON_TRACE
        if (button.state != last_state)
        {
            char trace[64];
            (void)snprintf(trace, sizeof(trace), "  [button] %s -> %s\r\n",
                           button_fsm_state_name(last_state),
                           button_fsm_state_name(button.state));
            usart6_write_string(trace);
            last_state = button.state;
        }
#endif

        if (ev == BTN_EVENT_SHORT)
        {
            rate_index   = (rate_index + 1U) % NUM_RATES;
            blink_period = k_rates_ms[rate_index];
            char msg[48];
            (void)snprintf(msg, sizeof(msg),
                           "short press: blink rate -> %lu ms\r\n",
                           (unsigned long)blink_period);
            usart6_write_string(msg);
        }
        else if (ev == BTN_EVENT_LONG)
        {
            blink_paused = (uint8_t)(blink_paused ^ 1U);
            if (blink_paused != 0U)
            {
                gpio_write_pin(LED_PORT, LED_PIN, GP_LOW);
            }
            usart6_write_string(blink_paused ? "long press: blink paused\r\n"
                                             : "long press: blink resumed\r\n");
        }
        else
        {
            /* no button event this pass */
        }

        /* --- Subsystem 3: non-blocking UART echo -------------------------- */
        uint8_t rx;
        if (usart6_getchar(&rx) != 0)
        {
            usart6_write_byte(rx);                 /* echo */
            if (rx == (uint8_t)'\r')
            {
                usart6_write_byte((uint8_t)'\n');  /* CR -> CRLF for terminals */
            }
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
