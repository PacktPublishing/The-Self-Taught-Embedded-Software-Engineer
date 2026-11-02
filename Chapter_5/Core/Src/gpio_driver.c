/**
 ******************************************************************************
 * @file    gpio_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 4 GPIO driver, extended in Chapter 5 with output speed
 *
 * @details Implementation of the API declared in gpio_driver.h. Unchanged from
 *          Chapter 4 apart from the OSPEEDR write in gpio_init(), which is the
 *          one register the Chapter 4 exercises had no reason to touch.
 *
 *              RCC_AHB1ENR   §6.3.10   per-port clock gate
 *              GPIOx_MODER   §7.4.1    2 bits per pin: I/O direction
 *              GPIOx_OTYPER  §7.4.2    1 bit  per pin: push-pull / open-drain
 *              GPIOx_OSPEEDR §7.4.3    2 bits per pin: output slew rate
 *              GPIOx_PUPDR   §7.4.4    2 bits per pin: pull-up / pull-down
 *              GPIOx_IDR     §7.4.5    input data register (read-only)
 *              GPIOx_BSRR    §7.4.7    atomic bit set/reset register
 *              GPIOx_AFRL    §7.4.9    4 bits per pin, pins 0-7:  AF select
 *              GPIOx_AFRH    §7.4.10   4 bits per pin, pins 8-15: AF select
 ******************************************************************************
 */

#include "gpio_driver.h"
#include <stddef.h>   /* for NULL */

/* =========================================================================
 * gpio_enable_port_clock  (file-private helper)
 *
 * Sets the correct RCC_AHB1ENR bit for the requested port. The GPIO ports
 * occupy consecutive AHB1ENR bits (GPIOAEN = bit 0, GPIOBEN = bit 1, ...),
 * so we translate the port base address into that bit index rather than
 * writing a long if/else ladder.
 * ========================================================================= */
static void gpio_enable_port_clock(const GPIO_TypeDef *port)
{
    /* Distance between two port register blocks in the memory map. On the
     * STM32F4 the GPIO ports are spaced 0x400 bytes apart, so the offset
     * from GPIOA divided by 0x400 gives the port index 0..7.              */
    const uint32_t port_index =
        ((uint32_t)port - (uint32_t)GPIOA_BASE) / 0x400UL;

    RCC->AHB1ENR |= (1UL << port_index);

    /* Read back so the clock-enable has taken effect before the first
     * configuration write — a documented STM32 erratum work-around.       */
    (void)RCC->AHB1ENR;
}

/* =========================================================================
 * gpio_select_alt_function  (file-private helper)
 *
 * Writes the pin's 4-bit alternate-function code. The 16 pins of a port are
 * split across two registers, AFRL for pins 0-7 and AFRH for pins 8-15,
 * which CMSIS exposes as the array AFR[0] and AFR[1]. Within the chosen
 * register each pin owns a nibble, so pin 5 lives at bits [23:20] of AFRL.
 * ========================================================================= */
static void gpio_select_alt_function(GPIO_TypeDef *port, uint32_t pin,
                                     uint32_t af)
{
    const uint32_t reg_index    = pin >> 3U;          /* 0 = AFRL, 1 = AFRH */
    const uint32_t nibble_shift = (pin & 0x7UL) * 4U; /* 4 bits per pin     */

    port->AFR[reg_index] &= ~(0xFUL << nibble_shift);
    port->AFR[reg_index] |=  ((af & 0xFUL) << nibble_shift);
}

/* =========================================================================
 * gpio_init
 * ========================================================================= */
void gpio_init(GPIO_TypeDef *port, const gpio_config_t *cfg)
{
    if ((port == NULL) || (cfg == NULL) || (cfg->pin > 15U))
    {
        return;   /* invalid request — fail safe, write nothing            */
    }

    gpio_enable_port_clock(port);

    const uint32_t pin      = (uint32_t)cfg->pin;
    const uint32_t pin_x2   = pin * 2U;   /* bit offset for 2-bit fields    */

    /* The alternate function must be selected before MODER hands the pin
     * over, so the peripheral never sees a moment on the wrong function.  */
    if (cfg->mode == GP_MODE_ALT)
    {
        gpio_select_alt_function(port, pin, (uint32_t)cfg->af);
    }

    /* MODER: clear the pin's 2-bit field, then write the requested mode.  */
    port->MODER &= ~(0x3UL << pin_x2);
    port->MODER |=  ((uint32_t)cfg->mode << pin_x2);

    /* OTYPER and OSPEEDR: only meaningful when something is driving the
     * pin. Skipping them for inputs and analog pins keeps the caller from
     * having to fill in fields that would be ignored.                     */
    if ((cfg->mode == GP_MODE_OUTPUT) || (cfg->mode == GP_MODE_ALT))
    {
        port->OTYPER &= ~(0x1UL << pin);
        port->OTYPER |=  ((uint32_t)cfg->otype << pin);

        /* New in Chapter 5. GP_SPEED_LOW is the reset value, so a caller
         * that leaves the field alone gets the Chapter 4 behaviour.       */
        port->OSPEEDR &= ~(0x3UL << pin_x2);
        port->OSPEEDR |=  ((uint32_t)cfg->speed << pin_x2);
    }

    /* PUPDR: clear the pin's 2-bit field, then write the pull selection.  */
    port->PUPDR &= ~(0x3UL << pin_x2);
    port->PUPDR |=  ((uint32_t)cfg->pull << pin_x2);
}

/* =========================================================================
 * gpio_read_pin
 * ========================================================================= */
gpio_level_t gpio_read_pin(GPIO_TypeDef *port, uint8_t pin)
{
    if ((port == NULL) || (pin > 15U))
    {
        return GP_LOW;
    }

    return ((port->IDR & (1UL << pin)) != 0UL) ? GP_HIGH : GP_LOW;
}

/* =========================================================================
 * gpio_write_pin
 *
 * BSRR is split in two halves: writing bit n sets pin n, writing bit
 * (n + 16) resets pin n. Using BSRR instead of a MODER-style read-modify-
 * write keeps the operation atomic, so it is safe to call from an ISR.
 * ========================================================================= */
void gpio_write_pin(GPIO_TypeDef *port, uint8_t pin, gpio_level_t level)
{
    if ((port == NULL) || (pin > 15U))
    {
        return;
    }

    if (level == GP_HIGH)
    {
        port->BSRR = (1UL << pin);          /* set bits  [15:0]            */
    }
    else
    {
        port->BSRR = (1UL << (pin + 16U));  /* reset bits [31:16]          */
    }
}

/* =========================================================================
 * gpio_toggle_pin
 *
 * XOR on the output data register flips just the target bit. ODR is safe to
 * read-modify-write here because the exercises drive one pin per port from
 * a single context.
 * ========================================================================= */
void gpio_toggle_pin(GPIO_TypeDef *port, uint8_t pin)
{
    if ((port == NULL) || (pin > 15U))
    {
        return;
    }

    port->ODR ^= (1UL << pin);
}
