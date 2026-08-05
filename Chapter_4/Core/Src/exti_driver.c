/**
 ******************************************************************************
 * @file    exti_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.3: EXTI driver implementation (PC13 button)
 *
 * @details Register-level configuration of EXTI line 13, plus the interrupt
 *          service routine that the vector table calls. Registers used
 *          (STM32F446RE Reference Manual RM0390):
 *
 *              RCC_APB2ENR        §6.3.14   SYSCFG clock gate
 *              SYSCFG_EXTICR4     §8.2.6    maps EXTI13 to a port
 *              EXTI_IMR           §10.3.1   interrupt mask
 *              EXTI_FTSR          §10.3.4   falling-edge trigger select
 *              EXTI_PR            §10.3.6   pending register (write-1-to-clear)
 *
 *          PC13 sits on EXTI line 13, which shares the EXTI15_10 NVIC vector
 *          with lines 10-15. The handler must therefore check the pending
 *          flag for its own line before acting.
 ******************************************************************************
 */

#include "exti_driver.h"
#include "gpio_driver.h"

/* Single registered callback. `volatile` because it is written from thread
 * context (exti_pc13_init) and read from interrupt context (the ISR).      */
static volatile exti_callback_t s_pc13_cb = (exti_callback_t)0;

/* EXTI line 13 bit, shared by IMR / FTSR / PR. */
#define PC13_EXTI_BIT    (1UL << 13)

void exti_pc13_init(exti_callback_t cb)
{
    s_pc13_cb = cb;

    /* 1. PC13 as a pulled-up input, through the driver from Exercise 4.2.
     *    Reusing it here does three things at once: it enables the GPIOC
     *    clock, it sets MODER to input, and it programs PUPDR. That last
     *    one matters. An EXTI line watches the pin, so a pin left floating
     *    would fire the interrupt on ambient noise rather than on a press.
     *    The pull-up matches how Exercise 4.2 configured the same button.  */
    const gpio_config_t btn_cfg = {
        .pin   = 13U,
        .mode  = GP_MODE_INPUT,
        .otype = GP_OTYPE_PUSHPULL,   /* not written for inputs           */
        .pull  = GP_PULL_UP,
        .af    = GP_AF_NONE
    };
    gpio_init(GPIOC, &btn_cfg);

    /* 2. SYSCFG lives on APB2 and needs its own clock gate before the
     *    EXTI line can be routed to a port.                               */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* 3. Route EXTI13 to port C. EXTICR4 covers lines 12-15; the line-13
     *    nibble is bits [7:4]. Port C is code 0x2.                         */
    SYSCFG->EXTICR[3] &= ~(0xFUL << 4U);
    SYSCFG->EXTICR[3] |=  (0x2UL << 4U);

    /* 4. Unmask line 13 and trigger on the falling edge (press pulls low). */
    EXTI->IMR  |= PC13_EXTI_BIT;
    EXTI->FTSR |= PC13_EXTI_BIT;
    EXTI->RTSR &= ~PC13_EXTI_BIT;   /* be explicit: no rising-edge trigger  */

    /* 5. Enable the shared vector in the NVIC. */
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* =========================================================================
 * EXTI15_10_IRQHandler
 *
 * Vector shared by EXTI lines 10-15. We service only line 13 here. Clearing
 * the pending bit (write 1) before calling the callback prevents the ISR
 * from immediately re-entering on the same event.
 * ========================================================================= */
void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & PC13_EXTI_BIT) != 0UL)
    {
        EXTI->PR = PC13_EXTI_BIT;   /* write-1-to-clear the pending flag    */

        if (s_pc13_cb != (exti_callback_t)0)
        {
            s_pc13_cb();
        }
    }
}
