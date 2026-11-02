/**
 ******************************************************************************
 * @file    stack_monitor.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - Stack painting, high-water mark, and a canary word
 *          (implementation)
 *
 * @details Painting stops a few words short of the live stack pointer
 *          (STACK_PAINT_GUARD) because the painting loop itself, and any
 *          exception taken while it runs, use stack below the value read at
 *          the start. Everything the guard leaves unpainted is counted as
 *          used, so the reported peak can only err on the high side.
 *
 *          The reserved stack, as this file sees it (addresses grow upward):
 *
 *              _estack (0x20020000) -> +------------------------+ top
 *                                      | in use by main() now   |  <- MSP
 *                                      | guard, left unpainted  |
 *                                      | painted 0xC5C5C5C5 ... |
 *                                      | ...                    |
 *              limit + 4            -> | painted 0xC5C5C5C5     |
 *              limit (0x2001FC00)   -> | canary  0xDEADBEEF     | bottom
 *                                      +------------------------+
 *                                        below: heap, .bss, .data
 *
 *          The stack grows down from _estack. As the program runs, deeper
 *          calls overwrite paint from the top downward and the paint never
 *          comes back, so the lowest overwritten word marks the deepest point
 *          the stack ever reached.
 *
 *          The two patterns come from stack_monitor.h:
 *            STACK_PAINT_WORD  0xC5C5C5C5. Chosen to be unlikely as real
 *                              stack data: not 0, not all ones, not a small
 *                              integer, and not a pointer into Flash
 *                              (0x08...) or SRAM (0x20...).
 *            STACK_CANARY_WORD 0xDEADBEEF. The traditional marker, easy to
 *                              spot by eye in the Memory Browser.
 *
 *          Two limits of the method, worth knowing before trusting a number:
 *            - Painting sees writes, not reservations. A function that
 *              reserves a large local array but writes only part of it
 *              leaves paint in the unwritten part, so the peak can read low.
 *              Size the reserve with a margin above the measured peak.
 *            - The canary is one word. A frame that reserves space past the
 *              canary but never writes that exact word slips by it.
 *              Exercise 7.2's stack_hog() writes every byte, so it cannot.
 *
 *          Interrupt handlers run on the same main stack (MSP) as main()
 *          here, so their stack use is included in the peak. That is
 *          correct: the reserve has to hold the deepest call chain plus
 *          whatever interrupts nest on top of it.
 *
 *          Used by Exercise 7.2 only.
 ******************************************************************************
 */

#include "stack_monitor.h"
#include "stm32f4xx.h"   /* CMSIS: __get_MSP() */

/* Linker-script symbols: only their addresses are meaningful. The linker
 * defines them as positions, with no storage behind them, so &_estack is
 * 0x20020000 and &_Min_Stack_Size is 0x400 (1024): reading the variable
 * itself would read whatever happens to be stored at that address. */
extern uint32_t _estack;
extern uint32_t _Min_Stack_Size;

/* Bytes left unpainted just below the stack pointer read at the start of
 * stack_paint(): 64 bytes, room for the painting code's own needs and one
 * exception frame (32 bytes without floating-point state). The words in the
 * guard are counted as used, which can only make the peak read high. */
#define STACK_PAINT_GUARD   (64UL)   /* bytes left unpainted below the live SP */

/* =========================================================================
 * stack_top_address
 *
 * _estack, the address just above the first word the stack uses. The very
 * first push after reset writes to _estack - 4: the Cortex-M stack is
 * "full descending", so the stack pointer is decremented before each write.
 * ========================================================================= */
uint32_t stack_top_address(void)
{
    return (uint32_t)&_estack;
}

/* =========================================================================
 * stack_reserved_bytes
 *
 * _Min_Stack_Size, the reserve the linker script sets aside (0x400 by
 * default, set at the top of STM32F446RETX_FLASH.ld).
 * ========================================================================= */
uint32_t stack_reserved_bytes(void)
{
    return (uint32_t)&_Min_Stack_Size;
}

/* =========================================================================
 * stack_limit_address
 *
 * The bottom of the reserved stack: _estack - _Min_Stack_Size, 0x2001FC00
 * with the default 1 KB. This is the same limit the C library's _sbrk()
 * (sysmem.c) uses to stop the heap from growing into the stack, and the
 * address where the canary lives. Nothing in hardware enforces it.
 * ========================================================================= */
uint32_t stack_limit_address(void)
{
    return stack_top_address() - stack_reserved_bytes();
}

/* =========================================================================
 * stack_used_now_bytes
 *
 * Bytes of stack in use at this moment: _estack minus the live main stack
 * pointer. Because it is read inside this function, the figure includes
 * this function's own small frame as well as everything its callers use.
 *
 * MSP is the right register here because there is no RTOS: main() and
 * every interrupt run on the main stack. Under FreeRTOS (Chapter 8), tasks
 * run on their own stacks through PSP, and this figure would only describe
 * the interrupt stack.
 * ========================================================================= */
uint32_t stack_used_now_bytes(void)
{
    return stack_top_address() - __get_MSP();
}

/* =========================================================================
 * stack_paint
 *
 * Writes the canary at the bottom of the reserve, then fills every word
 * from just above the canary up to STACK_PAINT_GUARD bytes below the
 * current stack pointer with STACK_PAINT_WORD. Call it once, as early in
 * main() as possible: everything already used before the call cannot be
 * painted and will count as used.
 * ========================================================================= */
void stack_paint(void)
{
    /* The canary first: the lowest word of the reserve. volatile so the
     * compiler performs the store even though no C code reads it back. */
    volatile uint32_t *const canary = (volatile uint32_t *)stack_limit_address();
    *canary = STACK_CANARY_WORD;

    /* Stop the paint STACK_PAINT_GUARD bytes below where the stack is right
     * now, rounded down to a word boundary (& ~0x3). */
    const uint32_t paint_end = (__get_MSP() - STACK_PAINT_GUARD) & ~0x3UL;

    /* Word by word, upward from just above the canary. volatile writes,
     * because the optimizer would otherwise see stores to memory nothing
     * reads and remove them. */
    for (uint32_t addr = stack_limit_address() + 4U; addr < paint_end; addr += 4U)
    {
        *(volatile uint32_t *)addr = STACK_PAINT_WORD;
    }
}

/* =========================================================================
 * stack_peak_bytes
 *
 * The high-water mark: the deepest the stack has been since stack_paint(),
 * in bytes measured down from _estack.
 *
 * It scans upward from just above the canary and stops at the first word
 * that no longer holds the paint pattern. That word is the lowest point the
 * stack ever wrote, so everything from there to _estack has been used.
 *
 * If the canary is gone, the stack went at least to the bottom of the
 * reserve and probably further, past what this function can see. It then
 * returns the whole reserve as a floor, not as the true figure.
 *
 * A real stack value that happens to equal 0xC5C5C5C5 at the boundary would
 * make the peak read one word low. With this pattern that is unlikely, and
 * one word is far below any sensible safety margin.
 * ========================================================================= */
uint32_t stack_peak_bytes(void)
{
    if (stack_canary_intact() == 0)
    {
        return stack_reserved_bytes();    /* overflowed: true peak unknown */
    }

    uint32_t addr = stack_limit_address() + 4U;
    while ((addr < stack_top_address()) &&
           (*(volatile const uint32_t *)addr == STACK_PAINT_WORD))
    {
        addr += 4U;                       /* still untouched paint: go up */
    }
    /* addr is now the lowest word the stack has written. */
    return stack_top_address() - addr;
}

/* =========================================================================
 * stack_canary_intact
 *
 * 1 if the word at the bottom of the reserve still holds STACK_CANARY_WORD,
 * 0 if anything has overwritten it. A 0 means the stack reached the very end
 * of its reserve at some point, and in Exercise 7.2 that it went past it
 * into the heap. The check costs one memory read, so a program can afford to
 * run it regularly, for example once per main-loop pass.
 * ========================================================================= */
int stack_canary_intact(void)
{
    const volatile uint32_t *const canary =
        (const volatile uint32_t *)stack_limit_address();
    return (*canary == STACK_CANARY_WORD) ? 1 : 0;
}
