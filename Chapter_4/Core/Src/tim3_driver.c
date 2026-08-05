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

/* 16 MHz / 16000 = 1 kHz -> one counter tick per millisecond. */
#define TIM3_PSC_1KHZ   (16000U - 1U)

static volatile tim3_tick_cb_t s_tick_cb = (tim3_tick_cb_t)0;

static uint16_t period_to_arr(uint32_t period_ms)
{
    if (period_ms == 0U)
    {
        period_ms = 1U;
    }
    else if (period_ms > 65536U)
    {
        period_ms = 65536U;
    }
    return (uint16_t)(period_ms - 1U);
}

void tim3_periodic_init(uint32_t period_ms, tim3_tick_cb_t cb)
{
    s_tick_cb = cb;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->PSC = TIM3_PSC_1KHZ;
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
