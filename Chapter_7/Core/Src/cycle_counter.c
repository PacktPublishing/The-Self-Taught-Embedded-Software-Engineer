/**
 ******************************************************************************
 * @file    cycle_counter.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - DWT cycle counter helpers (implementation)
 *
 * @details See cycle_counter.h. The three enable steps are the same ones the
 *          Chapter 6 pipeline performed inline: open the trace block, zero
 *          the counter, start it.
 ******************************************************************************
 */

#include "cycle_counter.h"
#include "stm32f4xx.h"   /* CMSIS: CoreDebug and DWT register blocks */

void cycle_counter_init(void)
{
    /* TRCENA powers the DWT unit. Without it CYCCNT reads as zero forever. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t cycle_counter_now(void)
{
    return DWT->CYCCNT;
}

uint32_t cycle_counter_elapsed_us(uint32_t start)
{
    /* Unsigned subtraction is correct across one wrap of the counter. */
    const uint32_t cycles = DWT->CYCCNT - start;
    return cycles / CYCLES_PER_US;
}

void cycle_counter_delay_us(uint32_t us)
{
    const uint32_t start  = DWT->CYCCNT;
    const uint32_t cycles = us * CYCLES_PER_US;

    while ((DWT->CYCCNT - start) < cycles)
    {
        /* deliberate busy-wait */
    }
}
