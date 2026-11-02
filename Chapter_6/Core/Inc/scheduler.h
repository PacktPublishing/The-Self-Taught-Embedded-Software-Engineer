/**
 ******************************************************************************
 * @file    scheduler.h
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.2: 1 ms time base for a cooperative super-loop
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details The whole cooperative super-loop hangs off one periodic interrupt.
 *          This module owns that time base: it programs the Chapter 4 TIM3
 *          driver for a 1 ms update interrupt and, from the ISR, does the one
 *          smallest possible thing - increment a free-running millisecond
 *          counter. Everything else stays in the main loop.
 *
 *          Subsystems do not each arm their own timer. Instead each keeps a
 *          "last time I ran" stamp and asks sched_due() whether its period has
 *          elapsed. That turns scheduling into cheap integer comparisons in the
 *          main loop and keeps the ISR trivial, which is exactly the discipline
 *          Section 6.2 argues for.
 ******************************************************************************
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief  Start the 1 ms time base (TIM3 update interrupt) and reset the
 *         millisecond counter to zero. Call once before the super-loop.
 */
void scheduler_init(void);

/**
 * @brief  Free-running milliseconds since scheduler_init().
 *
 *         A 32-bit read is atomic on Cortex-M4, so no interrupt masking is
 *         needed here even though the counter is written from the TIM3 ISR.
 *         The value wraps after ~49 days; sched_due() uses unsigned
 *         subtraction so it keeps working correctly across that wrap.
 */
uint32_t millis(void);

/**
 * @brief  Cooperative "is it my turn yet?" test for a periodic subsystem.
 *
 *         Compares now against @p *last_ms. If at least @p period_ms have
 *         elapsed, it advances @p *last_ms and returns 1; otherwise it returns
 *         0 and touches nothing. Each subsystem owns its own @p last_ms stamp.
 *
 * @param  last_ms    Caller's last-run timestamp (updated on a due tick).
 * @param  period_ms  Desired period in milliseconds.
 * @return 1 if the subsystem should run now, 0 if not yet.
 */
int sched_due(uint32_t *last_ms, uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
