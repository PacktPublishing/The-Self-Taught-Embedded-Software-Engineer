/**
 ******************************************************************************
 * @file    main_Ex_4_1_BareMetal_Blink.c
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.1: Bare-Metal LED Blink (No HAL, No CubeMX)
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 user LED → PA5 (GPIOA, pin 5)
 *
 * @details Demonstrates direct register-level GPIO control without any
 *          Hardware Abstraction Layer. Every address used here is taken
 *          from the STM32F446RE Reference Manual (RM0390).
 *
 *          What this exercise teaches:
 *          - How to enable a peripheral clock via RCC_AHB1ENR
 *          - How to configure a GPIO pin as push-pull output via GPIOA_MODER
 *          - How to set and clear a pin via GPIOA_ODR
 *          - Why pointers to absolute addresses are the C equivalent of
 *            hardware registers
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* -------------------------------------------------------------------------
 * Register base addresses (RM0390 §2.2.2 Memory map)
 * ------------------------------------------------------------------------- */

/** RCC AHB1 peripheral clock enable register (RM0390 §6.3.10) */
#define RCC_AHB1ENR_ADDR    (0x40023830UL)

/** GPIOA — mode register (RM0390 §7.4.1) */
#define GPIOA_MODER_ADDR    (0x40020000UL)

/** GPIOA — output data register (RM0390 §7.4.6) */
#define GPIOA_ODR_ADDR      (0x40020014UL)

/* -------------------------------------------------------------------------
 * Bit definitions
 * ------------------------------------------------------------------------- */

/** Bit 0 of RCC_AHB1ENR enables the GPIOA peripheral clock */
#define RCC_AHB1ENR_GPIOAEN_BIT     (0U)

/**
 * GPIOA_MODER: two bits per pin.
 * PA5 occupies bits [11:10].  Value 0b01 = General-purpose output mode.
 */
#define GPIO_MODER_PA5_BIT          (10U)
#define GPIO_MODER_PA5_MASK         (0x3UL << GPIO_MODER_PA5_BIT)
#define GPIO_MODER_PA5_OUTPUT       (0x1UL << GPIO_MODER_PA5_BIT)

/** PA5 bit in the Output Data Register */
#define GPIO_ODR_PA5_BIT            (5U)

/* -------------------------------------------------------------------------
 * Software delay tuning
 * -------------------------------------------------------------------------
 * At 16 MHz (HSI, reset default) 300 000 iterations of the loop below take
 * a few tens of milliseconds, which is slow enough to see and fast enough
 * to look like a blink.  Deliberately no exact figure is quoted: the cost
 * per iteration depends on the optimisation level, because a volatile
 * counter forces a load and a store every time round.  That imprecision is
 * the lesson.  A hardware timer replaces this guess in Exercise 4.4.
 * Use a volatile counter so the optimiser cannot remove the loop.
 * -------------------------------------------------------------------------*/
#define BLINK_DELAY_COUNT           (300000UL)

/* =========================================================================
 * Helper: cast an address literal to a volatile uint32_t pointer.
 * Using a macro keeps every dereference explicit and readable.
 * ========================================================================= */
#define REG32(addr)     (*(volatile uint32_t *)(addr))

/* =========================================================================
 * gpio_pa5_init
 *
 * Enables the GPIOA peripheral clock, then configures PA5 as a push-pull
 * general-purpose output (no pull-up/pull-down, low speed — sufficient for
 * a blink exercise).
 * ========================================================================= */
static void gpio_pa5_init(void)
{
    /* Step 1 — Enable GPIOA peripheral clock.
     * The RCC AHB1ENR GPIOAEN bit must be set before any GPIOA register
     * can be written; writing to a register whose clock is gated has no
     * effect and may lock the bus.                                         */
    REG32(RCC_AHB1ENR_ADDR) |= (1UL << RCC_AHB1ENR_GPIOAEN_BIT);

    /* Step 2 — Configure PA5 as output.
     * First clear both mode bits (reset to input = 0b00), then write 0b01
     * (general-purpose output).  Never write both bits in a single OR
     * without clearing first — a stale reset value could leave garbage.   */
    REG32(GPIOA_MODER_ADDR) &= ~GPIO_MODER_PA5_MASK;   /* clear [11:10]  */
    REG32(GPIOA_MODER_ADDR) |=  GPIO_MODER_PA5_OUTPUT; /* set  [11:10]=01 */
}

/* =========================================================================
 * delay_cycles
 *
 * Busy-wait software delay.  The loop variable is volatile so the compiler
 * cannot optimise it away.  This is intentionally simple — accurate timing
 * requires a hardware timer (see Chapter 4, Timer Fundamentals).
 * ========================================================================= */
static void delay_cycles(volatile uint32_t count)
{
    while (count > 0UL)
    {
        count--;
    }
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    /* Initialise PA5 as a GPIO output (LD2 on the Nucleo-F446RE)          */
    gpio_pa5_init();

    /* Toggle the LED forever — this is the bare-metal super-loop          */
    while (1)
    {
        /* Set PA5 HIGH → LED on                                           */
        REG32(GPIOA_ODR_ADDR) |= (1UL << GPIO_ODR_PA5_BIT);
        delay_cycles(BLINK_DELAY_COUNT);

        /* Clear PA5 LOW → LED off                                         */
        REG32(GPIOA_ODR_ADDR) &= ~(1UL << GPIO_ODR_PA5_BIT);
        delay_cycles(BLINK_DELAY_COUNT);
    }

    return 0;
}
