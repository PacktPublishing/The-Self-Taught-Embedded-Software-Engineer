/**
 ******************************************************************************
 * @file    uart_driver_isr.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2b: USART2 driver, interrupt-driven receive
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 on the ST-LINK VCP: PA2 = TX (AF7), PA3 = RX (AF7)
 *
 * @details This evolves the Exercise 5.2a driver. Transmit stays polling
 *          (simple and good enough for log-style output), but receive becomes
 *          interrupt-driven: RXNE raises USART2_IRQn, the handler reads DR and
 *          pushes the byte into a ring buffer, and the main loop drains that
 *          buffer at its own pace. The CPU no longer blocks waiting for input.
 *
 *          An optional per-byte callback can be registered for code that wants
 *          to react inside the ISR; if none is set, bytes simply queue.
 ******************************************************************************
 */

#ifndef UART_DRIVER_ISR_H
#define UART_DRIVER_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

typedef enum
{
    UART_OK = 0,
    UART_ERR_PARAM,
    UART_ERR_TIMEOUT
} uart_status_t;

typedef struct
{
    uint32_t baud;
} uart_config_t;

/** Optional hook called from ISR context for each received byte. */
typedef void (*uart_rx_callback_t)(uint8_t byte);

/**
 * @brief  Bring USART2 up in 8N1 at @p cfg->baud with interrupt-driven RX.
 *         Enables RXNEIE and the USART2 NVIC line; TX remains polling.
 */
uart_status_t uart2_init(const uart_config_t *cfg);

/** 
 * @brief Polling transmit of one byte (blocks on TXE). 
 */
uart_status_t uart2_write_byte(uint8_t byte);

/** 
 * @brief Polling transmit of a buffer; waits for TC at the end. 
 */
uart_status_t uart2_write(const uint8_t *data, uint32_t len);

/** 
 * @brief Polling transmit of a null-terminated string. 
 */
uart_status_t uart2_write_string(const char *str);

/**
 * @brief  Non-blocking read from the RX ring buffer.
 * @param  out  Destination for one byte. Must not be NULL.
 * @return 1 if a byte was returned, 0 if the buffer is currently empty.
 */
int uart2_getchar(uint8_t *out);

/**
 * @brief  Register (or clear, with NULL) an ISR-context per-byte callback.
 *
 *         The driver stores a function pointer and the ISR calls it for every
 *         byte it receives, so the driver can hand control back to application
 *         code it knows nothing about. This is the callback registration you
 *         first met in *Chapter 3, Section 3.5*, now running from a real
 *         interrupt rather than from a dispatcher in the main loop.
 *
 *         Whatever you register runs in interrupt context, so the same rule
 *         that governs the ISR governs your callback: set a flag, push to a
 *         queue, return. No printing, no waiting, no long work.
 */
void uart2_set_rx_callback(uart_rx_callback_t cb);

/**
 * @brief  Number of bytes lost to receiver overrun since init.
 *
 *         Non-zero means the ISR did not drain the data register before the
 *         next byte arrived. It is the peripheral-level counterpart to a full
 *         ring buffer, and telling the two apart is the first step in
 *         diagnosing dropped serial data.
 */
uint32_t uart2_rx_overruns(void);

/* =========================================================================
 * USART6 — header-native UART (PC6/PC7), interrupt-driven RX, for the
 * logic-analyzer build of this exercise.
 *
 * Same reason as Exercise 5.2a: USART2 (PA2/PA3) is bonded to the ST-LINK VCP
 * (SB13/SB14 ON) and is NOT on a header pin by default (SB62/SB63 OFF), so a
 * logic-analyzer probe cannot see it. USART6 on PC6 (TX) / PC7 (RX) is brought
 * straight to the morpho header — probe PC6. PORTC is otherwise unused by the
 * book (only PC13/button), so these pins stay conflict-free in later chapters.
 *
 * The API mirrors uart2_* exactly (TX polling, RX interrupt + ring buffer +
 * optional callback). It uses its OWN ring buffer and callback, so both UARTs
 * can be active independently. USART2 functions above are untouched.
 * ========================================================================= */

/** @brief Bring USART6 up in 8N1 at @p cfg->baud with interrupt-driven RX. */
uart_status_t usart6_init(const uart_config_t *cfg);

/** @brief Polling transmit of one byte (blocks on TXE). */
uart_status_t usart6_write_byte(uint8_t byte);

/** @brief Polling transmit of a buffer; waits for TC at the end. */
uart_status_t usart6_write(const uint8_t *data, uint32_t len);

/** @brief Polling transmit of a null-terminated string. */
uart_status_t usart6_write_string(const char *str);

/** @brief Non-blocking read from the USART6 RX ring buffer (1=byte, 0=empty). */
int usart6_getchar(uint8_t *out);

/** @brief Register (or clear, with NULL) the USART6 ISR per-byte callback. */
void usart6_set_rx_callback(uart_rx_callback_t cb);

/** @brief Bytes lost to USART6 receiver overrun since init. */
uint32_t usart6_rx_overruns(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_ISR_H */
