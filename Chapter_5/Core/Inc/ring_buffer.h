/**
 ******************************************************************************
 * @file    ring_buffer.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2b: Single-producer/single-consumer byte ring
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details A lock-free FIFO sized to a power of two so the index wrap is a
 *          cheap bitmask instead of a modulo. It is safe without disabling
 *          interrupts *provided* exactly one context writes (the USART RX ISR)
 *          and exactly one context reads (the main loop) — the classic 
 *          Single-producer/single-consumer pattern. `head` and `tail` are 
 *          volatile because they cross the ISR/thread boundary.
 ******************************************************************************
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Capacity must be a power of two; usable capacity is RB_SIZE - 1 bytes. */
#define RB_SIZE     (64U)
#define RB_MASK     (RB_SIZE - 1U)

typedef struct
{
    uint8_t data[RB_SIZE];
    volatile uint32_t head;   /* next write index (producer / ISR)  */
    volatile uint32_t tail;   /* next read index  (consumer / main) */
} ring_buffer_t;

/** @brief Reset a ring buffer to empty. */
void rb_init(ring_buffer_t *rb);

/** @brief Push one byte. @return 1 on success, 0 if the buffer was full. */
int rb_put(ring_buffer_t *rb, uint8_t byte);

/** @brief Pop one byte into *out. @return 1 on success, 0 if empty. */
int rb_get(ring_buffer_t *rb, uint8_t *out);

/** @brief @return 1 if the buffer holds no bytes, 0 otherwise. */
int rb_is_empty(const ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
