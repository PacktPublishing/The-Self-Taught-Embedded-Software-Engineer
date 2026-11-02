/**
 ******************************************************************************
 * @file    comm_if.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.5: A reusable communication interface
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details Exercises 5.2-5.4 produced three drivers with three different APIs
 *          (uart2_*, spi2_*, i2c1_*). This header defines ONE interface they
 *          can all hide behind, so application code can be written once and run
 *          over any transport. The pattern is a small virtual table: a struct
 *          of function pointers plus an opaque per-instance context.
 *
 *          Design choices the chapter argues for:
 *            - A single status enum (comm_status_t) so callers handle errors
 *              uniformly instead of learning each driver's private codes.
 *            - An opaque 'ctx' pointer so transports that need parameters
 *              (an I2C address, an SPI mode) carry them without widening the
 *              interface.
 *            - An optional completion callback, so the same interface supports
 *              the callback-on-event style used by interrupt-driven drivers.
 ******************************************************************************
 */

#ifndef COMM_IF_H
#define COMM_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef enum
{
    COMM_OK = 0,
    COMM_ERR_PARAM,
    COMM_ERR_TIMEOUT,
    COMM_ERR_NACK,
    COMM_ERR_IO
} comm_status_t;

typedef struct comm_if comm_if_t;

/* Optional hook invoked after each operation with its result. */
typedef void (*comm_event_cb_t)(const comm_if_t *self, comm_status_t st);

struct comm_if
{
    comm_status_t (*open)(comm_if_t *self);
    comm_status_t (*write)(comm_if_t *self, const uint8_t *data, uint32_t len);
    comm_status_t (*read)(comm_if_t *self, uint8_t *data, uint32_t len);
    void           *ctx;        /* per-instance backend context              */
    const char     *name;       /* human-readable label for logs             */
    comm_event_cb_t on_event;   /* optional; may be NULL                     */
};

/* Thin, NULL-safe dispatch wrappers — application code calls these. */
static inline comm_status_t comm_open(comm_if_t *s)
{
    return (s != NULL && s->open != NULL) ? s->open(s) : COMM_ERR_PARAM;
}
static inline comm_status_t comm_write(comm_if_t *s, const uint8_t *d, uint32_t n)
{
    return (s != NULL && s->write != NULL) ? s->write(s, d, n) : COMM_ERR_PARAM;
}
static inline comm_status_t comm_read(comm_if_t *s, uint8_t *d, uint32_t n)
{
    return (s != NULL && s->read != NULL) ? s->read(s, d, n) : COMM_ERR_PARAM;
}

/** @brief Map a status code to a short string for logging. */
const char *comm_status_str(comm_status_t st);

#ifdef __cplusplus
}
#endif

#endif /* COMM_IF_H */
