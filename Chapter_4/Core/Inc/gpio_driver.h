/**
 ******************************************************************************
 * @file    gpio_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.2: Minimal register-level GPIO driver API
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details This is the public interface of the GPIO driver. RCC_AHB1ENR,
 *          GPIOA_MODER and GPIOA_ODR register writes are wrapped behind four
 *          named operations:
 *
 *              gpio_init()        configure one pin from a settings struct
 *              gpio_read_pin()    read the current input level of one pin
 *              gpio_write_pin()   drive one pin high or low
 *              gpio_toggle_pin()  flip one pin's output level
 *
 *          The driver deliberately stays at the register level (no STM32 HAL).
 *          It uses the CMSIS typed peripheral structs from stm32f4xx.h, so a
 *          GPIO_TypeDef * such as GPIOA or GPIOC is passed as the port handle.
 *
 *          Scope kept small on purpose: mode, output type, pull resistors, and
 *          alternate-function selection. Output speed and pin locking are not
 *          exposed, because nothing in this chapter switches fast enough to
 *          need them.
 *
 *          Alternate functions matter from Exercise 4.6 onward. A timer or a
 *          serial peripheral does not drive the pin through the GPIO data
 *          registers; it takes the pin over. Selecting GP_MODE_ALT tells MODER
 *          to hand the pin to a peripheral, and the af field then says which
 *          one, because most pins can serve several. On PA5 the value 1
 *          selects TIM2_CH1, which is what the PWM exercise needs.
 *
 * @note    The enum constants use the short GP_ prefix (GP_MODE_OUTPUT,
 *          GP_PULL_UP, ...) rather than GPIO_*. This is deliberate: when this
 *          driver is built inside a HAL-enabled STM32CubeIDE project, the
 *          vendor header stm32f4xx_hal_gpio.h already #defines names such as
 *          GPIO_MODE_INPUT and GPIO_MODE_ANALOG. Reusing those names would let
 *          the preprocessor rewrite our enum and break the build, so the driver
 *          keeps its own private namespace.
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
    GP_OTYPE_OPENDRAIN = 0x1U    /* 1: open-drain                          */
} gpio_otype_t;

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
 * of the STM32F446 datasheet (DS10693).
 * ------------------------------------------------------------------------- */
#define GP_AF_NONE      (0U)    /* ignored unless the mode is GP_MODE_ALT   */
#define GP_AF1_TIM2     (1U)    /* PA5 -> TIM2_CH1, used by Exercise 4.6    */

/* -------------------------------------------------------------------------
 * One pin's configuration. The caller fills this in and passes it to
 * gpio_init() — a small, self-documenting alternative to five positional
 * function arguments.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t      pin;     /* pin number within the port, 0-15               */
    gpio_mode_t  mode;    /* input / output / alternate / analog            */
    gpio_otype_t otype;   /* push-pull or open-drain (driven modes only)    */
    gpio_pull_t  pull;    /* internal pull resistor selection               */
    uint8_t      af;      /* alternate function 0-15, read only when the    */
                          /* mode is GP_MODE_ALT                            */
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
 *         untouched. OTYPER is programmed only for driven modes, and the
 *         AFRL/AFRH nibble only when the mode is GP_MODE_ALT.
 *
 * @param  port  CMSIS port handle, e.g. GPIOA or GPIOC.
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
 * @brief  Flip an output pin's current level.
 *
 * @param  port  CMSIS port handle.
 * @param  pin   Pin number, 0-15.
 */
void gpio_toggle_pin(GPIO_TypeDef *port, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_DRIVER_H */
