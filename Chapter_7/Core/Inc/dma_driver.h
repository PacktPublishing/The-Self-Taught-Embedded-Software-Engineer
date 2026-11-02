/**
 ******************************************************************************
 * @file    dma_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 7 - DMA stream driver shared by UART, ADC, and DAC
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details The F446 has two DMA controllers, DMA1 and DMA2, each with eight
 *          streams. A stream is one transfer engine. Each stream has an
 *          eight-way multiplexer in front of it, and the CHSEL field picks
 *          which peripheral request drives it. Which request sits on which
 *          stream and channel is fixed in silicon and listed in RM0390
 *          Table 28 (DMA1) and Table 29 (DMA2). The routes this chapter uses:
 *
 *              USART2_TX   DMA1 Stream 6  Channel 4   (Exercises 7.3a, 7.3b)
 *              USART2_RX   DMA1 Stream 5  Channel 4   (Exercises 7.3a, 7.3b)
 *              DAC1        DMA1 Stream 5  Channel 7   (Exercise 7.5)
 *              ADC1        DMA2 Stream 0  Channel 0   (Exercises 7.4b, 7.5)
 *              mem-to-mem  DMA2 Stream 7  any channel (Exercise 7.3a)
 *
 *          Notice that USART2_RX and DAC1 both need DMA1 Stream 5. A stream
 *          serves one request at a time, so those two can never run in the
 *          same program. That is a design constraint, not a driver bug, and
 *          it is why Exercise 7.5 keeps the UART on polled transmit only.
 *          Only DMA2 can perform memory-to-memory transfers.
 *
 *          The driver owns every DMA interrupt handler a Chapter 7 exercise
 *          uses and forwards the stream's events to a callback registered by
 *          the peripheral driver. Keeping the handlers here, rather than in
 *          uart_driver_dma.c and dac_driver.c, is what lets both of those
 *          files sit in the same STM32CubeIDE project even though they share
 *          DMA1 Stream 5: the vector is defined exactly once.
 *
 *          Registers touched (RM0390 Section 9.5):
 *              RCC_AHB1ENR   DMA1EN / DMA2EN   controller clock gate
 *              DMA_SxCR      stream configuration and enable
 *              DMA_SxNDTR    number of data items left to transfer
 *              DMA_SxPAR     peripheral address (source in mem-to-mem)
 *              DMA_SxM0AR    memory address
 *              DMA_SxFCR     FIFO / direct mode
 *              DMA_LISR/HISR, DMA_LIFCR/HIFCR   event flags and their clears
 ******************************************************************************
 */

#ifndef DMA_DRIVER_H
#define DMA_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

typedef enum
{
    DMA_OK = 0,
    DMA_ERR_PARAM,
    DMA_ERR_TIMEOUT
} dma_status_t;

/** Transfer direction, the value written to DMA_SxCR.DIR. */
typedef enum
{
    DMA_DIR_PERIPH_TO_MEM = 0U,
    DMA_DIR_MEM_TO_PERIPH = 1U,
    DMA_DIR_MEM_TO_MEM    = 2U
} dma_dir_t;

/** Data item width, the value written to DMA_SxCR.PSIZE and MSIZE. */
typedef enum
{
    DMA_SIZE_8BIT  = 0U,
    DMA_SIZE_16BIT = 1U,
    DMA_SIZE_32BIT = 2U
} dma_size_t;

/** Arbitration priority between streams of one controller (DMA_SxCR.PL). */
typedef enum
{
    DMA_PRIO_LOW       = 0U,
    DMA_PRIO_MEDIUM    = 1U,
    DMA_PRIO_HIGH      = 2U,
    DMA_PRIO_VERY_HIGH = 3U
} dma_prio_t;

/* Event bits handed to a callback. They use the same positions as the
 * stream's flags inside DMA_LISR/HISR (after shifting the stream's group
 * down to bit 0), so the driver can pass the hardware value straight on. */
#define DMA_EVT_FIFO_ERR     (1UL << 0)   /* FEIF  */
#define DMA_EVT_DIRECT_ERR   (1UL << 2)   /* DMEIF */
#define DMA_EVT_XFER_ERR     (1UL << 3)   /* TEIF  */
#define DMA_EVT_HALF         (1UL << 4)   /* HTIF: first half of the block done  */
#define DMA_EVT_COMPLETE     (1UL << 5)   /* TCIF: whole block done              */
#define DMA_EVT_ALL          (0x3DUL)

/**
 * Signature of a stream callback. It runs in interrupt context, so the rule
 * from Chapter 4 applies: record what happened in a volatile variable and
 * return. @p events is a mask of DMA_EVT_* bits.
 */
typedef void (*dma_callback_t)(uint32_t events);

/** Everything that stays fixed for a stream between transfers. */
typedef struct
{
    DMA_Stream_TypeDef *stream;       /* e.g. DMA1_Stream6                     */
    uint8_t             channel;      /* request multiplexer input, 0-7        */
    dma_dir_t           dir;
    dma_size_t          psize;        /* peripheral-side item width            */
    dma_size_t          msize;        /* memory-side item width                */
    uint8_t             pinc;         /* 1: peripheral address increments      */
    uint8_t             minc;         /* 1: memory address increments          */
    uint8_t             circular;     /* 1: NDTR reloads and the stream loops  */
    dma_prio_t          priority;
    uint8_t             irq_half;     /* 1: interrupt at the half-way point    */
    uint8_t             irq_complete; /* 1: interrupt at the end of the block  */
    dma_callback_t      callback;     /* NULL: no interrupt, poll the flags    */
} dma_config_t;

/**
 * @brief  Enable the controller clock and program a stream's fixed settings.
 *         Stops the stream first if it is running. Does not start it.
 * @return DMA_ERR_PARAM for an unknown stream, a channel above 7, or a
 *         circular mem-to-mem request (the hardware does not allow it).
 */
dma_status_t dma_stream_init(const dma_config_t *cfg);

/**
 * @brief  Load addresses and length, clear stale flags, and enable the stream.
 *
 * @param  stream       Stream configured earlier with dma_stream_init().
 * @param  periph_addr  Peripheral register address (the source address in a
 *                      mem-to-mem transfer).
 * @param  mem_addr     Memory buffer address (the destination in mem-to-mem).
 *                      The buffer must outlive the transfer: the DMA keeps
 *                      reading or writing it after this function returns.
 * @param  count        Number of data items (not bytes), 1 to 65535.
 */
dma_status_t dma_stream_start(DMA_Stream_TypeDef *stream, uint32_t periph_addr,
                              uint32_t mem_addr, uint16_t count);

/**
 * @brief  Disable a stream, wait until the hardware confirms it has stopped,
 *         and clear its flags.
 */
dma_status_t dma_stream_stop(DMA_Stream_TypeDef *stream);

/**
 * @brief  Items still to be transferred in the current block (DMA_SxNDTR).
 *         In circular mode this counts down and reloads, which is how a
 *         receiver works out where the hardware is writing right now.
 */
uint16_t dma_stream_remaining(const DMA_Stream_TypeDef *stream);

/**
 * @brief  Read a stream's event flags without an interrupt.
 * @param  clear  Non-zero: clear the flags that were read.
 * @return Mask of DMA_EVT_* bits.
 */
uint32_t dma_stream_poll_events(const DMA_Stream_TypeDef *stream, uint8_t clear);

#ifdef __cplusplus
}
#endif

#endif /* DMA_DRIVER_H */
