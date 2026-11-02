/**
 ******************************************************************************
 * @file    scheduler.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.2: 1 ms time base implementation
 *
 * @details The TIM3 driver from Chapter 4 already delivers a periodic update
 *          interrupt and calls a registered callback from its ISR. Here that
 *          callback is the smallest useful thing: bump a millisecond counter.
 *          Keeping the ISR to a single increment is the point - all real work
 *          happens later, in the main loop, at a time the CPU chooses.
 *
 *          Everything else in this file (millis, sched_due) just reads that one
 *          counter, so the cooperative super-loop needs no other timer and no
 *          per-subsystem interrupts.
 ******************************************************************************
 */

#include "scheduler.h"
#include "tim3_driver.h"

/* The single source of time for the whole super-loop. Written by the TIM3 ISR
 * (via on_tick), read by the main loop. `volatile` because it crosses the
 * interrupt/thread boundary, so the compiler must re-read it every time.     */
static volatile uint32_t s_ticks_ms = 0U;

/* Runs in TIM3 interrupt context once per millisecond. Deliberately trivial -
 * a single store - so the ISR adds no measurable jitter to the main loop.    */
static void on_tick(void)
{
    s_ticks_ms++;
}

void scheduler_init(void)
{
    s_ticks_ms = 0U;
    tim3_periodic_init(1U, on_tick);   /* program TIM3 for a 1 ms update IRQ */
}

uint32_t millis(void)
{
    /* A 32-bit load is a single, atomic instruction on Cortex-M4, so this read
     * cannot be torn by the ISR firing mid-access - no masking needed.        */
    return s_ticks_ms;
}

int sched_due(uint32_t *last_ms, uint32_t period_ms)
{
    if (last_ms == 0)
    {
        return 0;                      /* NULL-safe: nothing to schedule */
    }

    const uint32_t now = s_ticks_ms;

    /* Unsigned subtraction is the key trick: even when s_ticks_ms wraps past
     * 2^32 (after ~49 days), (now - *last_ms) still yields the correct elapsed
     * count, so the comparison keeps working across the wrap.                 */
    if ((uint32_t)(now - *last_ms) >= period_ms)
    {
        *last_ms = now;                /* remember when this slot last ran */
        return 1;                      /* due now */
    }
    return 0;                          /* not yet */
}
