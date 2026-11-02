/**
 ******************************************************************************
 * @file    uart_driver_dma.h
 * @author  Renato Soriano
 * @brief   Chapter 7 - USART2 driver upgraded to DMA transmit and receive
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 on the ST-Link VCP: PA2 = TX (AF7), PA3 = RX (AF7)
 *
 * @details This is the Chapter 5 USART2 driver (as carried into Chapter 6)
 *          with the byte-at-a-time interrupt receiver replaced by DMA.
 *          Initialization, pin setup through the Chapter 4 GPIO driver, the
 *          baud calculation, and the polled transmit calls are unchanged, so
 *          code that only logs text keeps working. What is new:
 *
 *            uart2_dma_send()               memory-to-peripheral, DMA1 S6 ch4.
 *                                           Returns at once; the CPU is free
 *                                           while the bytes go out.
 *            uart2_dma_receive()            peripheral-to-memory, DMA1 S5 ch4,
 *                                           normal mode: a fixed-length block.
 *            uart2_dma_rx_start_circular()  the same stream in circular mode,
 *            uart2_dma_rx_service()         reported in two halves (HT and TC)
 *                                           plus the USART idle-line event for
 *                                           the partial tail of a message.
 *
 *          The DMA interrupt handlers live in dma_driver.c. This file only
 *          registers callbacks with it. The USART2 vector is still owned
 *          here, now for the idle-line interrupt instead of RXNE.
 *
 *          This file replaces uart_driver_isr.c in the Chapter 7 project.
 *          Do not build both: each defines USART2_IRQHandler and the
 *          uart2_* functions.
 *
 *          Registers added to the Chapter 5 set (RM0390):
 *              USART_CR3  DMAT, DMAR   Section 25.6.6   hand TXE/RXNE to the DMA
 *              USART_CR1  IDLEIE       Section 25.6.4   idle-line interrupt
 *              USART_SR   IDLE, TC     Section 25.6.1   idle flag, transmit done
 *          Procedure: RM0390 Section 25.4.13, Continuous communication using DMA.
 ******************************************************************************
 */

#ifndef UART_DRIVER_DMA_H
#define UART_DRIVER_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

typedef enum
{
    UART_OK = 0,
    UART_ERR_PARAM,
    UART_ERR_TIMEOUT,
    UART_ERR_BUSY
} uart_status_t;

typedef struct
{
    uint32_t baud;
} uart_config_t;

/* Reasons a circular-receive chunk was delivered (may be combined). */
#define UART_RX_EVT_HALF   (1UL << 0)   /* DMA half-transfer: first half full  */
#define UART_RX_EVT_FULL   (1UL << 1)   /* DMA transfer-complete: second half  */
#define UART_RX_EVT_IDLE   (1UL << 2)   /* USART idle line: sender paused      */

/**
 * Receives one contiguous chunk of the circular buffer. Called from
 * uart2_dma_rx_service(), so it runs in thread context, not in an ISR.
 * @p data points into the DMA buffer itself: consume or copy it before the
 * hardware comes round again, which at 115200 baud is half the buffer
 * length times 86.8 us.
 */
typedef void (*uart_rx_chunk_cb_t)(const uint8_t *data, uint32_t len,
                                   uint32_t events);

/** Counters a circular receiver keeps, for the exercise's status line. */
typedef struct
{
    uint32_t half_events;   /* DMA HT interrupts                          */
    uint32_t full_events;   /* DMA TC interrupts                          */
    uint32_t idle_events;   /* USART idle-line interrupts                 */
    uint32_t bytes;         /* bytes handed to the chunk callback         */
    uint32_t overflows;     /* times the DMA lapped the reader            */
    uint32_t dma_errors;    /* transfer or direct-mode errors             */
} uart_rx_stats_t;

/**
 * @brief  Bring USART2 up in 8N1 at @p cfg->baud, with transmit and receive
 *         enabled and the transmit DMA stream prepared. No receive DMA is
 *         started until uart2_dma_receive() or the circular start is called.
 */
uart_status_t uart2_init(const uart_config_t *cfg);

/** @brief Polled transmit of one byte (waits for any DMA transmit first). */
uart_status_t uart2_write_byte(uint8_t byte);

/** @brief Polled transmit of a buffer; waits for TC at the end. */
uart_status_t uart2_write(const uint8_t *data, uint32_t len);

/** @brief Polled transmit of a null-terminated string. */
uart_status_t uart2_write_string(const char *str);

/**
 * @brief  Start a DMA transmit and return immediately.
 * @param  data  Must stay valid and unchanged until uart2_dma_tx_busy()
 *               returns 0. A local array that goes out of scope is a bug:
 *               the DMA would keep reading whatever reuses that stack.
 * @param  len   1 to 65535 bytes.
 * @return UART_ERR_BUSY if the previous DMA transmit has not finished.
 */
uart_status_t uart2_dma_send(const uint8_t *data, uint16_t len);

/** @return 1 while a DMA transmit is still feeding the USART, else 0. */
int uart2_dma_tx_busy(void);

/**
 * @brief  Receive exactly @p len bytes by DMA (normal mode) and return at
 *         once. Poll uart2_dma_rx_done() to learn when the block is full.
 * @return UART_ERR_BUSY if a circular receive is running.
 */
uart_status_t uart2_dma_receive(uint8_t *buf, uint16_t len);

/** @return 1 once the block started by uart2_dma_receive() is complete. */
int uart2_dma_rx_done(void);

/**
 * @brief  Start continuous reception into @p buf, treated as two halves.
 * @param  len  Even, at least 4. The DMA raises HT when the first half is
 *              full and TC when the second is, then wraps and keeps going.
 */
uart_status_t uart2_dma_rx_start_circular(uint8_t *buf, uint16_t len);

/**
 * @brief  Deliver every byte the DMA has written since the last call.
 *         Call it from the main loop. If the reader fell a whole buffer
 *         behind, the data in between is gone: the chunk is dropped, the
 *         overflow counter increments, and the reader resynchronizes.
 * @return Number of bytes delivered.
 */
uint32_t uart2_dma_rx_service(uart_rx_chunk_cb_t cb);

/** @brief Copy the circular receiver's counters into @p out. */
void uart2_dma_rx_stats(uart_rx_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_DMA_H */
