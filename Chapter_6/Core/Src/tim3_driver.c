/**
 ******************************************************************************
 * @file    tim3_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.5: TIM3 periodic interrupt (implementation)
 *
 * @details Registers used (STM32F446RE Reference Manual RM0390, general-
 *          purpose timer chapter):
 *
 *              RCC_APB1ENR  §6.3.13   TIM3 clock gate (TIM3EN, bit 1)
 *              TIM3_PSC     prescaler: counter clock = f / (PSC + 1)
 *              TIM3_ARR     auto-reload: counts per update event
 *              TIM3_DIER    UIE: enable the update interrupt
 *              TIM3_SR      UIF: update interrupt flag (clear in the ISR)
 *              TIM3_EGR     UG: force prescaler/ARR to load immediately
 *              TIM3_CR1     CEN: counter enable
 *
 *          The ISR is intentionally tiny: clear the flag, call the callback.
 ******************************************************************************
 */

#include "tim3_driver.h"

/* 16 MHz / 1600 = 10 kHz -> one counter tick every 0.1 ms.
 *
 * The base is 10 kHz, not 1 kHz, on purpose. A 1 ms period on a 1 kHz base
 * needs ARR = (1 - 1) = 0, and an STM32 timer with ARR = 0 generates no
 * periodic update event: the counter never overflows, UIF never sets, and
 * the tick would never fire. At 10 kHz a 1 ms period is ARR = 9, which
 * counts and overflows normally. */
#define TIM3_TICKS_PER_MS   (10U)
#define TIM3_PSC_10KHZ      (1600U - 1U)

static volatile tim3_tick_cb_t s_tick_cb = (tim3_tick_cb_t)0;

/* Compile-time guard on the timebase itself.
 *
 * Everything below assumes at least one counter tick per millisecond, because
 * the shortest period the driver accepts is 1 ms. If someone later "simplifies"
 * the prescaler to a 1 kHz base, this stops the build instead of producing a
 * timer that silently never fires. That failure is nasty to diagnose from the
 * outside: the tick stops, so every millis()-based comparison in the caller
 * freezes at once and the whole program looks dead rather than mistimed. */
#if (TIM3_TICKS_PER_MS < 2U)
#error "TIM3 timebase too coarse: a 1 ms period would need ARR = 0, which never fires."
#endif

static uint16_t period_to_arr(uint32_t period_ms)
{
    if (period_ms == 0U)
    {
        period_ms = 1U;
    }
    else if (period_ms > 6553U)
    {
        period_ms = 6553U;   /* keep period_ms * 10 - 1 within the 16-bit ARR */
    }

    const uint32_t arr = (period_ms * TIM3_TICKS_PER_MS) - 1U;

    /* Belt and braces: ARR = 0 must never reach the register. With the guard
     * above this is unreachable, but it costs one comparison and it documents
     * the invariant at the point where the value is produced. */
    return (uint16_t)((arr == 0U) ? 1U : arr);
}

void tim3_periodic_init(uint32_t period_ms, tim3_tick_cb_t cb)
{
    s_tick_cb = cb;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->PSC = TIM3_PSC_10KHZ;
    TIM3->ARR = period_to_arr(period_ms);
    TIM3->EGR = TIM_EGR_UG;     /* load PSC/ARR now                        */
    TIM3->SR  = 0U;             /* clear the UIF that UG just raised        */

    TIM3->DIER |= TIM_DIER_UIE; /* enable the update interrupt             */
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 |= TIM_CR1_CEN;   /* start counting                          */
}

void tim3_set_period_ms(uint32_t period_ms)
{
    TIM3->ARR = period_to_arr(period_ms);
    TIM3->EGR = TIM_EGR_UG;     /* apply the new reload at once            */
    TIM3->SR  = 0U;             /* swallow the UG-generated flag           */
}

/* =========================================================================
 * TIM3_IRQHandler — fires once per programmed period
 * ========================================================================= */
void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        TIM3->SR = ~TIM_SR_UIF;     /* clear only the update flag          */

        if (s_tick_cb != (tim3_tick_cb_t)0)
        {
            s_tick_cb();
        }
    }
}
