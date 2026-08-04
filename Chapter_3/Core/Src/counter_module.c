/**
 ******************************************************************************
 * @file    counter_module.c
 * @author  Renato Soriano
 * @brief   Chapter 3 — Exercise 3.4b: Multi-file `static` / `extern`
 *          Counter module implementation.
 *
 * @details Three uses of `static` appear in this file, and each one means
 *          something different.
 *
 *          1. File-scope `static` on a variable
 *             (`static uint32_t s_counter;`)
 *             Internal linkage. No other translation unit can see this
 *             symbol — not even via `extern`. That is what gives us our
 *             encapsulation.
 *
 *          2. File-scope `static` on a helper function
 *             (`static void log_change(...);`)
 *             Internal linkage. The function name does not leak into the
 *             link namespace, so the same helper name can be reused in
 *             other modules without collision.
 *
 *          3. Function-scope `static` on a local variable
 *             (`static uint32_t s_call_count;` inside `counter_increment()`)
 *             Persistent storage with function-only scope. The variable
 *             lives for the entire program — exactly like a global — but
 *             only `counter_increment()` is allowed to see it.
 ******************************************************************************
 */

#include "counter_module.h"
#include "itm_debug.h"          /* For trace logging via SWV/ITM           */
#include <stdio.h>

/* -------------------------------------------------------------------------
 * 1. File-scope `static` data — INTERNAL LINKAGE.
 *
 * The compiler emits this symbol but the linker hides it from other
 * translation units. Even `extern uint32_t s_counter;` in main.c would
 * fail to link.
 * ------------------------------------------------------------------------- */
static uint32_t s_counter = 0U;

/* -------------------------------------------------------------------------
 * 2. File-scope `static` helper — INTERNAL LINKAGE.
 *
 * Used only inside this file; not advertised in counter_module.h.
 * ------------------------------------------------------------------------- */
static void log_change(const char *event, uint32_t value);

void counter_reset(void)
{
    s_counter = 0U;
    log_change("reset",  s_counter);
}

uint32_t counter_increment(void)
{
    /* ---------------------------------------------------------------------
     * 3. Function-scope `static` — persistent across calls, but local in
     *    scope. Only this function can see `s_call_count`.
     * ------------------------------------------------------------------- */
    static uint32_t s_call_count = 0U;
    s_call_count++;

    s_counter++;
    log_change("increment", s_counter);
    printf("    (counter_increment has been called %lu time(s) so far)\r\n",
           (unsigned long)s_call_count);

    return s_counter;
}

uint32_t counter_get(void)
{
    return s_counter;
}

static void log_change(const char *event, uint32_t value)
{
    printf("  [counter_module] %-9s -> value = %lu\r\n",
           event, (unsigned long)value);
}
