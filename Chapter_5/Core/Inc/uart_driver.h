/**
 ******************************************************************************
 * @file    uart_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2a: Register-level USART2 driver (polling)
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 is wired to the on-board ST-LINK Virtual COM Port:
 *              PA2  = USART2_TX  (AF7)  -> ST-LINK -> USB -> PC terminal
 *              PA3  = USART2_RX  (AF7)  <- ST-LINK <- USB <- PC terminal
 *
 * @details Public interface for a minimal, blocking (polling-mode) UART driver.
 *          This is the first peripheral in the book that talks to the world
 *          outside the MCU, so the driver is intentionally small: 8 data bits,
 *          no parity, 1 stop bit (8N1), configurable baud rate.
 *
 *          Polling means the CPU waits on status flags (TXE, RXNE, TC) inside
 *          each call. That is fine for bring-up and simple prints; Exercise
 *          5.2b replaces the receive path with interrupts and a ring buffer.
 *
 *          Baud rate is derived from APB1_CLOCK_HZ (main.h). The same naming
 *          and config-struct style as the Chapter 4 GPIO driver is reused.
 ******************************************************************************
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Status / error codes. Returning an explicit code (rather than void) is a
 * habit worth forming early: every driver call can fail, and Section 5.5
 * builds a common error-handling layer on top of exactly these enums.
 * ------------------------------------------------------------------------- */
typedef enum
{
    UART_OK = 0,        /* operation completed                              */
    UART_ERR_PARAM,     /* a NULL pointer or invalid argument was passed    */
    UART_ERR_TIMEOUT    /* a status flag did not assert within the budget   */
} uart_status_t;

/* -------------------------------------------------------------------------
 * Driver configuration. Kept to one field for now (8N1 is fixed); a struct is
 * still used so later chapters can add word length / parity without changing
 * every call site.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint32_t baud;      /* e.g. 9600, 115200                                */
} uart_config_t;

/**
 * @brief  Configure PA2/PA3 as USART2 AF7 and bring USART2 up in 8N1 at the
 *         requested baud rate (polling mode, TX + RX enabled).
 * @param  cfg  Pointer to a configuration with a non-zero baud. Must not be NULL.
 * @return UART_OK, or UART_ERR_PARAM on a bad argument.
 */
uart_status_t uart2_init(const uart_config_t *cfg);

/**
 * @brief  Transmit one byte, blocking until the transmit data register is empty.
 */
uart_status_t uart2_write_byte(uint8_t byte);

/**
 * @brief  Transmit a buffer, byte by byte, then wait for the final byte to
 *         fully leave the shift register (TC) so the line is idle on return.
 * @param  data  Source buffer. Must not be NULL when len > 0.
 * @param  len   Number of bytes to send.
 */
uart_status_t uart2_write(const uint8_t *data, uint32_t len);

/**
 * @brief  Transmit a null-terminated C string (convenience wrapper).
 */
uart_status_t uart2_write_string(const char *str);

/**
 * @brief  Block until one byte arrives, then return it.
 * @param  out  Where to store the received byte. Must not be NULL.
 */
uart_status_t uart2_read_byte(uint8_t *out);

/**
 * @brief  Non-blocking check for a pending received byte.
 * @return 1 if RXNE is set (a byte is waiting), 0 otherwise.
 */
int uart2_rx_ready(void);

/* =========================================================================
 * USART6 — header-native UART for logic-analyzer capture (PC6/PC7).
 *
 * Why a second UART exists in this driver:
 * On the Nucleo-F446RE, USART2 (PA2/PA3) reaches the ST-LINK Virtual COM Port
 * through solder bridges SB13/SB14 (ON by default), while the bridges to the
 * morpho/Arduino header pins, SB62/SB63, are OFF by default (see UM1724). That
 * makes USART2 perfect for a PC terminal but means its TX/RX are NOT on a
 * header pin you can clip a logic-analyzer probe to without reworking bridges.
 *
 * USART6 on PC6 (TX) / PC7 (RX) is brought straight out to the ST morpho
 * header with no bridges in the way, so it is the channel the logic-analyzer
 * variant of this exercise uses. PORTC is otherwise unused by the book's
 * peripherals (only PC13/button), so these pins stay conflict-free in later
 * chapters. There is no VCP on USART6: probe PC6 with the analyzer (UART
 * decoder, 8N1), or attach a USB-TTL adapter if a PC terminal is also wanted.
 *
 * The API mirrors uart2_* exactly so application code can switch transport by
 * changing only the prefix. USART2 functions above are left untouched.
 * ========================================================================= */

/**
 * @brief  Configure PC6/PC7 as USART6 AF8 and bring USART6 up in 8N1 at the
 *         requested baud rate (polling mode, TX + RX enabled).
 * @param  cfg  Pointer to a configuration with a non-zero baud. Must not be NULL.
 * @return UART_OK, or UART_ERR_PARAM on a bad argument.
 */
uart_status_t usart6_init(const uart_config_t *cfg);

/** @brief Transmit one byte, blocking until TXE. */
uart_status_t usart6_write_byte(uint8_t byte);

/** @brief Transmit a buffer, then wait for TC so the line is idle on return. */
uart_status_t usart6_write(const uint8_t *data, uint32_t len);

/** @brief Transmit a null-terminated C string. */
uart_status_t usart6_write_string(const char *str);

/** @brief Block until one byte arrives, then return it via @p out. */
uart_status_t usart6_read_byte(uint8_t *out);

/** @brief Non-blocking check: 1 if a received byte is waiting (RXNE), else 0. */
int usart6_rx_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_H */
