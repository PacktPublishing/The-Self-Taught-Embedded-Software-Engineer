/**
 ******************************************************************************
 * @file    gpio_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 4 GPIO driver, extended in Chapter 5 with output speed
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details This is the same driver you built in *Chapter 4, Section 4.1*, with
 *          one field added. Chapter 4 handed a pin to a peripheral through the
 *          mode and af fields, which is what every bus driver in this chapter
 *          needs: USART2 and USART6 want AF7 and AF8, SPI2 wants AF5, and I2C1
 *          wants AF4 plus open-drain outputs and a pull-up.
 *
 *          Chapter 5 adds the speed field, which programs OSPEEDR. Chapter 4
 *          never needed it, because a blinking LED and a PWM output switch far
 *          too slowly for the pin's slew rate to matter. A bus is different:
 *          the driver here selects a faster edge for the SPI clock and data
 *          lines, and the section explains the trade-off you are making when
 *          you do (sharper edges buy you signal margin at higher clock rates,
 *          and cost you ringing, overshoot, and radiated noise).
 *
 *          The field is added at the END of gpio_config_t and the LOW value is
 *          the reset default on every pin this book touches, so every Chapter 4
 *          exercise keeps compiling and keeps behaving exactly as it did.
 *
 *          Registers touched (RM0390):
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

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"   /* CMSIS: GPIO_TypeDef, RCC, peripheral base pointers */
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Pin mode — value written to the two MODER bits of a pin (RM0390 7.4.1).
 * ------------------------------------------------------------------------- */
typedef enum
{
    GP_MODE_INPUT  = 0x0U,   /* 00: digital input                          */
    GP_MODE_OUTPUT = 0x1U,   /* 01: general-purpose output                 */
    GP_MODE_ALT    = 0x2U,   /* 10: alternate function, see the af field   */
    GP_MODE_ANALOG = 0x3U    /* 11: analog                                 */
} gpio_mode_t;

/* -------------------------------------------------------------------------
 * Output driver type — OTYPER bit for the pin (RM0390 7.4.2).
 * ------------------------------------------------------------------------- */
typedef enum
{
    GP_OTYPE_PUSHPULL  = 0x0U,   /* 0: push-pull (default)                 */
    GP_OTYPE_OPENDRAIN = 0x1U    /* 1: open-drain, required by I2C         */
} gpio_otype_t;

/* -------------------------------------------------------------------------
 * Output speed — the two OSPEEDR bits for the pin (RM0390 7.4.3). New in
 * Chapter 5.
 *
 * This does NOT set a data rate. It selects how hard the pad drives the line,
 * which sets the slew rate: how quickly each edge rises and falls. LOW is the
 * reset value on every pin this book configures and is right for anything that
 * switches slowly, which is why Chapter 4 never touched it. A bus clock is the
 * case where it starts to matter, because a slow edge on a fast clock eats into
 * the receiver's setup window. The cost of a fast edge is ringing, overshoot,
 * and radiated noise, so the rule is to pick the slowest setting that still
 * gives a clean edge, not the fastest one available.
 *
 * Table 58 of the STM32F446 datasheet (DS10693) gives the numbers, at
 * VDD >= 2.7 V and a 50 pF load:
 *
 *      00 Low      4 MHz max, edges up to 100 ns
 *      01 Medium  25 MHz max, edges up to  10 ns
 *      10 Fast    50 MHz max, edges up to   6 ns   (40 pF load)
 *      11 High   100 MHz max, edges up to   4 ns   (30 pF load)
 *
 * Those figures are what make the choice in spi_driver.c concrete rather than
 * a matter of taste.
 * ------------------------------------------------------------------------- */
typedef enum
{
    GP_SPEED_LOW    = 0x0U,   /* 00: reset default, slowest edges          */
    GP_SPEED_MEDIUM = 0x1U,   /* 01                                        */
    GP_SPEED_FAST   = 0x2U,   /* 10                                        */
    GP_SPEED_HIGH   = 0x3U    /* 11: fastest edges, worst noise            */
} gpio_speed_t;

/* -------------------------------------------------------------------------
 * Internal pull resistor — value written to the two PUPDR bits (RM0390 7.4.4).
 * ------------------------------------------------------------------------- */
typedef enum
{
    GP_PULL_NONE = 0x0U,   /* 00: no pull-up, no pull-down                 */
    GP_PULL_UP   = 0x1U,   /* 01: internal pull-up                         */
    GP_PULL_DOWN = 0x2U    /* 10: internal pull-down                       */
} gpio_pull_t;

/* -------------------------------------------------------------------------
 * Logical pin level returned by gpio_read_pin() / accepted by gpio_write_pin().
 * ------------------------------------------------------------------------- */
typedef enum
{
    GP_LOW  = 0U,
    GP_HIGH = 1U
} gpio_level_t;

/* -------------------------------------------------------------------------
 * Alternate-function numbers used in this book. Which peripheral each AFn
 * selects for a given pin is listed in the alternate-function mapping table
 * of the STM32F446 datasheet (DS10693). The Chapter 5 entries are the four
 * the bus drivers need.
 * ------------------------------------------------------------------------- */
#define GP_AF_NONE      (0U)    /* ignored unless the mode is GP_MODE_ALT   */
#define GP_AF1_TIM2     (1U)    /* PA5  -> TIM2_CH1,  Chapter 4 Exercise 4.6 */
#define GP_AF4_I2C1     (4U)    /* PB8/PB9   -> I2C1_SCL / I2C1_SDA          */
#define GP_AF5_SPI2     (5U)    /* PB13/14/15 -> SPI2_SCK / MISO / MOSI      */
#define GP_AF7_USART2   (7U)    /* PA2/PA3   -> USART2_TX / USART2_RX        */
#define GP_AF8_USART6   (8U)    /* PC6/PC7   -> USART6_TX / USART6_RX        */

/* -------------------------------------------------------------------------
 * One pin's configuration. The caller fills this in and passes it to
 * gpio_init() — a small, self-documenting alternative to six positional
 * function arguments.
 *
 * speed is last and its zero value is the hardware reset default, so a
 * Chapter 4 initializer that names no speed still compiles and still produces
 * the identical register writes it always did.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t      pin;     /* pin number within the port, 0-15               */
    gpio_mode_t  mode;    /* input / output / alternate / analog            */
    gpio_otype_t otype;   /* push-pull or open-drain (driven modes only)    */
    gpio_pull_t  pull;    /* internal pull resistor selection               */
    uint8_t      af;      /* alternate function 0-15, read only when the    */
                          /* mode is GP_MODE_ALT                            */
    gpio_speed_t speed;   /* output slew rate (driven modes only). Omit for */
                          /* GP_SPEED_LOW, which is the reset default.      */
} gpio_config_t;

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
 * @brief  Enable a GPIO port's clock and configure one pin.
 *
 *         Enables the port's RCC AHB1 clock gate (idempotent — safe to call
 *         once per pin), then programs MODER and PUPDR for the pin described
 *         by @p cfg using read-modify-write so neighbouring pins are
 *         untouched. OTYPER and OSPEEDR are programmed only for driven modes,
 *         and the AFRL/AFRH nibble only when the mode is GP_MODE_ALT.
 *
 * @param  port  CMSIS port handle, e.g. GPIOA, GPIOB or GPIOC.
 * @param  cfg   Pointer to the pin configuration. Must not be NULL.
 */
void gpio_init(GPIO_TypeDef *port, const gpio_config_t *cfg);

/**
 * @brief  Read the live input level of a pin from the input data register.
 *
 * @param  port  CMSIS port handle.
 * @param  pin   Pin number, 0-15.
 * @return GP_HIGH if the pin reads logic 1, otherwise GP_LOW.
 */
gpio_level_t gpio_read_pin(GPIO_TypeDef *port, uint8_t pin);

/**
 * @brief  Drive an output pin to a given level using the bit-set/reset
 *         register (BSRR), which is atomic and needs no read-modify-write.
 *
 * @param  port   CMSIS port handle.
 * @param  pin    Pin number, 0-15.
 * @param  level  GP_HIGH or GP_LOW.
 */
void gpio_write_pin(GPIO_TypeDef *port, uint8_t pin, gpio_level_t level);

/**
 * @brief  Invert a pin's output level.
 * 
 * @param  port  CMSIS port handle.
 * @param  pin   Pin number, 0-15. 
 */
void gpio_toggle_pin(GPIO_TypeDef *port, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_DRIVER_H */
