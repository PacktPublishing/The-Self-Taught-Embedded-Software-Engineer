/**
 ******************************************************************************
 * @file    pwm_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.6: PWM LED-dimming driver (implementation)
 *
 * @details Registers used (STM32F446RE Reference Manual RM0390):
 *
 *            GPIO (PA5 -> TIM2_CH1, AF1), all via the Ex 4.2 gpio driver:
 *              RCC_AHB1ENR   GPIOA clock gate
 *              GPIOA_MODER   PA5 = alternate-function mode (10)
 *              GPIOA_AFRL    PA5 alternate function = AF1 (TIM2)
 *
 *            Timer (TIM2, general-purpose, 32-bit):
 *              RCC_APB1ENR   TIM2 clock gate (TIM2EN, bit 0)
 *              TIM2_PSC      prescaler -> 1 MHz counter clock
 *              TIM2_ARR      auto-reload -> PWM period (1000 steps)
 *              TIM2_CCMR1    OC1M = PWM mode 1, OC1PE = preload enable
 *              TIM2_CCER     CC1E = enable channel-1 output
 *              TIM2_CCR1     compare value -> duty cycle
 *              TIM2_CR1      ARPE = ARR preload, CEN = counter enable
 *              TIM2_EGR      UG = load prescaler/ARR immediately
 *
 *          PWM mode 1: the output is active while CNT < CCR1, so CCR1 directly
 *          sets the on-time. Duty% = CCR1 / ARR * 100.
 ******************************************************************************
 */

#include "pwm_driver.h"
#include "gpio_driver.h"

/* 16 MHz / 16 = 1 MHz counter clock. */
#define TIM2_PSC_1MHZ   (16U - 1U)

/* OC1M = 0b110 (PWM mode 1) sits in CCMR1 bits [6:4]. */
#define CCMR1_OC1M_PWM1 (0x6UL << 4U)

void pwm_led_init(void)
{
    /* ---- GPIO: PA5 as alternate function AF1 (TIM2_CH1) ----
     * The same driver from Exercise 4.2 does this job. Handing the pin to
     * a peripheral is still just MODER plus one nibble of AFRL, so there
     * is no reason to write those registers by hand here. The pull stays
     * off: the timer drives the pin push-pull, so nothing needs holding.  */
    const gpio_config_t pwm_pin_cfg = {
        .pin   = 5U,
        .mode  = GP_MODE_ALT,
        .otype = GP_OTYPE_PUSHPULL,
        .pull  = GP_PULL_NONE,
        .af    = GP_AF1_TIM2
    };
    gpio_init(GPIOA, &pwm_pin_cfg);

    /* ---- TIM2: 1 kHz PWM on channel 1 ---- */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    TIM2->PSC = TIM2_PSC_1MHZ;
    TIM2->ARR = PWM_RESOLUTION_STEPS - 1U;   /* 1000 steps -> 1 kHz         */

    /* PWM mode 1 with output preload on channel 1. */
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |=  CCMR1_OC1M_PWM1;
    TIM2->CCMR1 |=  TIM_CCMR1_OC1PE;         /* preload CCR1                 */

    TIM2->CCR1 = 0U;                        /* start at 0% duty (LED off)  */
    TIM2->CCER |= TIM_CCER_CC1E;             /* enable CH1 output           */

    TIM2->CR1 |= TIM_CR1_ARPE;             /* buffer ARR                  */
    TIM2->EGR = TIM_EGR_UG;                /* load PSC/ARR now            */
    TIM2->CR1 |= TIM_CR1_CEN;               /* start the timer             */
}

void pwm_led_set_duty(uint8_t percent)
{
    if (percent > 100U)
    {
        percent = 100U;
    }

    /* Scale percentage to the 0..ARR compare range. With ARR = 999 and a
     * 1000-step resolution, CCR1 = percent * 10.                          */
    TIM2->CCR1 = ((uint32_t)percent * PWM_RESOLUTION_STEPS) / 100U;
}
