/**
 ******************************************************************************
 * @file    iwdg_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 4 — Exercise 4.7: Independent Watchdog (IWDG) driver
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Public interface for the Independent Watchdog. The IWDG is a
 *          down-counter clocked from the low-speed internal oscillator (LSI,
 *          ~32 kHz), fully independent of the main system clock. If firmware
 *          fails to refresh ("feed") it before the counter reaches zero, the
 *          IWDG resets the MCU — the standard recovery mechanism for a hung
 *          main loop.
 *
 *          Once started the IWDG cannot be stopped in software, which is the
 *          point: production code must keep feeding it. This driver therefore
 *          exposes only start and refresh, plus a helper to ask whether the
 *          last reset came from the watchdog.
 *
 * @note    The LSI is an untrimmed RC oscillator, not a crystal, so its real
 *          frequency varies from part to part and drifts with temperature and
 *          supply voltage. Every timeout computed from a nominal 32 kHz is
 *          therefore an estimate. Size a production timeout from the fastest
 *          end of the range published in the STM32F446 datasheet (DS10693,
 *          internal clock source characteristics), not from the typical
 *          value, or the watchdog may fire earlier than the design intends.
 ******************************************************************************
 */

#ifndef IWDG_DRIVER_H
#define IWDG_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/* IWDG prescaler codes written to IWDG_PR — divide the ~32 kHz LSI clock. */
typedef enum
{
    IWDG_PRE_4   = 0x0U,
    IWDG_PRE_8   = 0x1U,
    IWDG_PRE_16  = 0x2U,
    IWDG_PRE_32  = 0x3U,
    IWDG_PRE_64  = 0x4U,
    IWDG_PRE_128 = 0x5U,
    IWDG_PRE_256 = 0x6U
} iwdg_prescaler_t;

/**
 * @brief  Start the IWDG with a given prescaler and reload value.
 *
 *         Timeout (seconds) is approximately:
 *             (prescaler_divider * (reload + 1)) / 32000
 *         e.g. divider 32, reload 999 -> about 1.0 s.
 *
 * @param  prescaler  One of the iwdg_prescaler_t codes.
 * @param  reload     12-bit reload value, 0..4095.
 */
void iwdg_start(iwdg_prescaler_t prescaler, uint16_t reload);

/**
 * @brief  Refresh ("feed") the watchdog, reloading its down-counter.
 *         Must be called periodically, faster than the configured timeout.
 */
void iwdg_refresh(void);

/**
 * @brief  Report whether the most recent reset was caused by the IWDG, then
 *         clear the reset-cause flags so the next boot reads cleanly.
 *
 * @return 1 if the last reset was an IWDG reset, otherwise 0.
 */
uint8_t iwdg_caused_last_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* IWDG_DRIVER_H */
