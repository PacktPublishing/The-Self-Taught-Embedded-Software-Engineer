/**
 ******************************************************************************
 * @file    iwdg_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.7: Independent Watchdog driver (implementation)
 *
 * @details Registers used (STM32F446RE Reference Manual RM0390, IWDG chapter):
 *
 *              IWDG_KR   key register — magic words drive the state machine:
 *                          0x5555 unlock PR/RLR for writing
 *                          0xAAAA reload the counter (feed)
 *                          0xCCCC start the watchdog
 *              IWDG_PR   prescaler select (divides the LSI clock)
 *              IWDG_RLR  12-bit reload value
 *              IWDG_SR   status: PVU/RVU busy flags for PR/RLR writes
 *              RCC_CSR   reset-cause flags (IWDGRSTF) and RMVF clear bit
 *
 *          PR and RLR are clocked in the LSI domain, so after writing them we
 *          must wait for the matching busy flag in IWDG_SR to clear before the
 *          values are guaranteed to have taken effect.
 *
 *          Start-key ordering matters. The PVU/RVU busy flags only clear once
 *          the LSI oscillator is running, and the LSI is started as a side
 *          effect of the 0xCCCC start key — not by the 0x5555 unlock key. The
 *          start key must therefore be written FIRST; otherwise, on a cold
 *          boot where the LSI is still off, the wait loops below would spin
 *          forever. This is the exact sequence documented in RM0390:
 *          0xCCCC (start) -> 0x5555 (unlock) -> PR -> RLR -> wait -> 0xAAAA
 ******************************************************************************
 */

#include "iwdg_driver.h"

#define IWDG_KEY_UNLOCK     (0x5555U)
#define IWDG_KEY_FEED       (0xAAAAU)
#define IWDG_KEY_START      (0xCCCCU)

void iwdg_start(iwdg_prescaler_t prescaler, uint16_t reload)
{
    if (reload > 0x0FFFU)
    {
        reload = 0x0FFFU;   /* RLR is 12-bit */
    }

    /* 1. Start the IWDG. This also turns on the LSI oscillator, which is what
     *    clocks the prescaler/reload update logic below. After this the IWDG
     *    cannot be stopped in software — only a reset clears it.            */
    IWDG->KR = IWDG_KEY_START;

    /* 2. Unlock write access to PR and RLR. */
    IWDG->KR = IWDG_KEY_UNLOCK;

    /* 3. Program prescaler and reload. */
    IWDG->PR  = (uint32_t)prescaler;
    IWDG->RLR = reload;

    /* 4. Wait for both update flags to clear (values now live in the LSI
     *    domain). The LSI is running because of step 1, so these terminate. */
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U)
    {
        /* PR/RLR writes propagating into the LSI domain */
    }

    /* 5. Reload the counter with the freshly programmed value. */
    IWDG->KR = IWDG_KEY_FEED;
}

void iwdg_refresh(void)
{
    IWDG->KR = IWDG_KEY_FEED;
}

uint8_t iwdg_caused_last_reset(void)
{
    uint8_t was_iwdg = 0U;

    if ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U)
    {
        was_iwdg = 1U;
    }

    /* Clear all reset-cause flags so the next boot starts from a clean slate. */
    RCC->CSR |= RCC_CSR_RMVF;

    return was_iwdg;
}
