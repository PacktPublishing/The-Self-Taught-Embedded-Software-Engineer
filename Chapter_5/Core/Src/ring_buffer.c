/**
 ******************************************************************************
 * @file    ring_buffer.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2b: SPSC byte ring buffer (implementation)
 *
 * @details The buffer is considered full when advancing head would collide
 *          with tail, so one slot is always left empty to distinguish "full"
 *          from "empty" without a separate count variable (which would itself
 *          need protecting across the ISR/thread boundary).
 ******************************************************************************
 */

#include "ring_buffer.h"
#include <stddef.h>   /* NULL */

void rb_init(ring_buffer_t *rb)
{
    if (rb == NULL)
    {
        return;
    }
    rb->head = 0U;
    rb->tail = 0U;
}

int rb_put(ring_buffer_t *rb, uint8_t byte)
{
    if (rb == NULL)
    {
        return 0;
    }

    const uint32_t next = (rb->head + 1U) & RB_MASK;
    if (next == rb->tail)
    {
        return 0;   /* full — drop the byte rather than overwrite unread data */
    }

    rb->data[rb->head] = byte;
    rb->head = next;            /* publish only after the data is stored */
    return 1;
}

int rb_get(ring_buffer_t *rb, uint8_t *out)
{
    if ((rb == NULL) || (out == NULL))
    {
        return 0;
    }

    if (rb->head == rb->tail)
    {
        return 0;   /* empty */
    }

    *out = rb->data[rb->tail];
    rb->tail = (rb->tail + 1U) & RB_MASK;
    return 1;
}

int rb_is_empty(const ring_buffer_t *rb)
{
    if (rb == NULL)
    {
        return 1;
    }
    return (rb->head == rb->tail) ? 1 : 0;
}
