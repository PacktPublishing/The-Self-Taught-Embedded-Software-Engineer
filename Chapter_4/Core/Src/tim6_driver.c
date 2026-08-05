/**
 ******************************************************************************
 * @file    tim6_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.4: TIM6 basic-timer delay (implementation)
 *
 * @details Registers used (STM32F446RE Reference Manual RM0390, chapter on
 *          basic timers TIM6/TIM7):
 *
 *              RCC_APB1ENR  §6.3.13   TIM6 clock gate (TIM6EN, bit 4)
 *              TIM6_PSC     §19.4.7   prescaler: counter clock = f / (PSC+1)
 *              TIM6_ARR     §19.4.8   auto-reload: counts before an update
 *              TIM6_CNT     §19.4.6   the counter itself
 *              TIM6_EGR     §19.4.5   UG bit forces the prescaler to load
 *              TIM6_SR      §19.4.4   UIF update flag
 *              TIM6_CR1     §19.4.1   CEN enable, OPM one-pulse mode
 *
 *          Prescaler maths:
 *              counter clock = 16 MHz / (PSC + 1)
 *          For a 1 ms tick we want 1 kHz, so PSC + 1 = 16000, i.e.
 *              PSC = 16000 - 1 = 15999.
 *          The delay length is then just the auto-reload value in ms.
 ******************************************************************************
 */

#include "tim6_driver.h"

/* 16 MHz / 16000 = 1 kHz -> one counter tick every millisecond. */
#define TIM6_PSC_1KHZ   (16000U - 1U)

void tim6_init(void)
{
    /* Enable the TIM6 peripheral clock on APB1. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    (void)RCC->APB1ENR;   /* read-back so the gate is live before use      */

    /* Program the 1 ms tick. Writing the prescaler alone does not take
     * effect until an update event, so we pulse UG to load it now.        */
    TIM6->PSC = TIM6_PSC_1KHZ;
    TIM6->ARR = 1U;                 /* placeholder; set per call in delay   */
    TIM6->EGR = TIM_EGR_UG;         /* force prescaler + ARR to load        */
    TIM6->SR  = 0U;                 /* clear the update flag raised by UG   */

    /* One-pulse mode: the counter stops itself after a single update, which
     * is exactly the behaviour a blocking delay wants.                     */
    TIM6->CR1 |= TIM_CR1_OPM;
}

void tim6_delay_ms(uint32_t ms)
{
    /* TIM6 is 16-bit: ARR holds 0..65535, i.e. 1..65536 ms of delay. */
    if (ms == 0U)
    {
        ms = 1U;
    }
    else if (ms > 65536U)
    {
        ms = 65536U;
    }

    TIM6->ARR = (uint16_t)(ms - 1U);   /* `ms` ticks before the update     */
    TIM6->CNT = 0U;
    TIM6->SR  = 0U;                     /* clear any stale UIF              */

    TIM6->CR1 |= TIM_CR1_CEN;           /* start; OPM clears CEN at the end */

    /* Spin until the counter reaches ARR and raises the update flag. */
    while ((TIM6->SR & TIM_SR_UIF) == 0U)
    {
        /* busy-wait — the LED is off-CPU, so nothing else to do here */
    }

    TIM6->SR = 0U;                      /* clear UIF for the next call      */
}
