/**
 ******************************************************************************
 * @file    comm_backends.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.5: UART/SPI/I2C backends (implementation)
 *
 * @details Each backend implements open/write/read against one driver and
 *          translates that driver's private status into the shared
 *          comm_status_t. The optional on_event callback is invoked after every
 *          operation so interrupt-style "tell me when done" code can hook in
 *          without changing the interface.
 *
 *          Note the asymmetry the abstraction has to paper over:
 *            - UART read is a byte stream (we block for the requested count).
 *            - SPI is full-duplex with a software chip select around the frame.
 *            - I2C needs the 7-bit address and ACK/NACK handling.
 *          The application above never sees any of this.
 ******************************************************************************
 */

#include "comm_backends.h"
#include "uart_driver.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include <stddef.h>

/* Map the unified status code to a short label. Because every backend funnels
 * its private error into comm_status_t, the caller (and this helper) only ever
 * has to understand ONE set of codes — that is the whole point of the layer. */
const char *comm_status_str(comm_status_t st)
{
    switch (st)
    {
        case COMM_OK:          return "OK";
        case COMM_ERR_PARAM:   return "PARAM";
        case COMM_ERR_TIMEOUT: return "TIMEOUT";
        case COMM_ERR_NACK:    return "NACK";
        case COMM_ERR_IO:      return "IO";
        default:               return "?";
    }
}

/* Fire the optional completion callback, if one was registered on this
 * interface. Centralising it here means every backend reports completion the
 * same way, and a backend that has no callback set simply does nothing. This is
 * how the blocking API still supports an interrupt-style "tell me when done"
 * usage without changing any function signatures. */
static void notify(comm_if_t *self, comm_status_t st)
{
    if ((self != NULL) && (self->on_event != NULL))
    {
        self->on_event(self, st);
    }
}

/* ======================= UART backend ==================================
 * Each of the three functions below matches a slot in the comm_if_t vtable
 * (open/write/read). They are static because callers never name them directly:
 * they are reached only through the function pointers installed by
 * comm_uart_bind(). The per-instance parameters (here, the baud rate) travel
 * in the opaque ctx pointer, which we cast back to the concrete context type.
 * ======================================================================== */

/* Fold the UART driver's private status into the shared enum, so nothing above
 * this line has to know UART_ERR_* exists. Anything unexpected becomes IO. */
static comm_status_t uart_translate(uart_status_t s)
{
    switch (s)
    {
        case UART_OK:          return COMM_OK;
        case UART_ERR_PARAM:   return COMM_ERR_PARAM;
        case UART_ERR_TIMEOUT: return COMM_ERR_TIMEOUT;
        default:               return COMM_ERR_IO;
    }
}

static comm_status_t uart_open(comm_if_t *self)
{
    /* Recover the UART-specific config from the opaque context and start USART2. */
    const comm_uart_ctx_t *c = (const comm_uart_ctx_t *)self->ctx;
    const uart_config_t cfg = { .baud = c->baud };
    const comm_status_t st = uart_translate(uart2_init(&cfg));
    notify(self, st);
    return st;
}

static comm_status_t uart_write(comm_if_t *self, const uint8_t *d, uint32_t n)
{
    /* UART is a simple byte stream: hand the whole buffer to the driver. The
     * instance has no per-transfer parameters, so ctx is unused here. */
    const comm_status_t st = uart_translate(uart2_write(d, n));
    notify(self, st);
    return st;
}

static comm_status_t uart_read(comm_if_t *self, uint8_t *d, uint32_t n)
{
    /* The polling UART driver returns one byte per call, so a multi-byte read
     * loops. Bail out (and report) on the first byte that errors/ times out. */
    for (uint32_t i = 0U; i < n; i++)
    {
        const uart_status_t s = uart2_read_byte(&d[i]);
        if (s != UART_OK)
        {
            const comm_status_t st = uart_translate(s);
            notify(self, st);
            return st;
        }
    }
    notify(self, COMM_OK);
    return COMM_OK;
}

/* Wire this interface instance to the UART implementation: install the three
 * function pointers, attach the caller-owned context, label it, and start with
 * no completion callback (the app can set on_event afterwards if it wants one). */
void comm_uart_bind(comm_if_t *iface, comm_uart_ctx_t *ctx)
{
    iface->open     = uart_open;
    iface->write    = uart_write;
    iface->read     = uart_read;
    iface->ctx      = ctx;
    iface->name     = "UART";
    iface->on_event = NULL;
}

/* ======================= SPI backend ===================================
 * SPI differs from UART in two ways the adapter must hide: it is full-duplex,
 * and each transaction is framed by a software chip select. The write/read
 * helpers therefore bracket every transfer with cs_low()/cs_high().
 *
 * SPI has no addressing and no acknowledge, so it has no equivalent of a NACK
 * and its translator is the shortest of the three.
 * ======================================================================== */
static comm_status_t spi_translate(spi_status_t s)
{
    switch (s)
    {
        case SPI_OK:          return COMM_OK;
        case SPI_ERR_PARAM:   return COMM_ERR_PARAM;
        case SPI_ERR_TIMEOUT: return COMM_ERR_TIMEOUT;
        default:              return COMM_ERR_IO;
    }
}

static comm_status_t spi_open(comm_if_t *self)
{
    /* Pull the SPI mode/prescaler out of the context and bring SPI2 up. */
    const comm_spi_ctx_t *c = (const comm_spi_ctx_t *)self->ctx;
    const comm_status_t st = spi_translate(spi2_init(&c->cfg));
    notify(self, st);
    return st;
}

static comm_status_t spi_write(comm_if_t *self, const uint8_t *d, uint32_t n)
{
    /* Assert CS, shift the buffer out (discarding the bytes clocked in on
     * MISO with rx = NULL), then deassert CS so the frame is complete.
     * CS is released on the error path too, so a failed transfer does not
     * leave the slave selected. */
    spi2_cs_low();
    const spi_status_t s = spi2_transfer_buf(d, NULL, n);
    spi2_cs_high();

    const comm_status_t st = spi_translate(s);
    notify(self, st);
    return st;
}

static comm_status_t spi_read(comm_if_t *self, uint8_t *d, uint32_t n)
{
    /* A read is the same full-duplex exchange with the roles reversed: send
     * padding (tx = NULL -> 0xFF) purely to generate clocks, and capture MISO. */
    spi2_cs_low();
    const spi_status_t s = spi2_transfer_buf(NULL, d, n);
    spi2_cs_high();

    const comm_status_t st = spi_translate(s);
    notify(self, st);
    return st;
}

/* Install the SPI implementation into this interface instance. */
void comm_spi_bind(comm_if_t *iface, comm_spi_ctx_t *ctx)
{
    iface->open     = spi_open;
    iface->write    = spi_write;
    iface->read     = spi_read;
    iface->ctx      = ctx;
    iface->name     = "SPI";
    iface->on_event = NULL;
}

/* ======================= I2C backend ===================================
 * I2C adds a piece of state the other two do not have: the 7-bit slave
 * address, which the app supplies once in the context and every transfer then
 * uses. I2C also has a distinct failure mode — a slave that does not ACK — so
 * its translator maps I2C_ERR_NACK to COMM_ERR_NACK (a code the UART/SPI paths
 * never produce). The application still just calls comm_write()/comm_read().
 * ======================================================================== */
static comm_status_t i2c_translate(i2c_status_t s)
{
    switch (s)
    {
        case I2C_OK:          return COMM_OK;
        case I2C_ERR_PARAM:   return COMM_ERR_PARAM;
        case I2C_ERR_TIMEOUT: return COMM_ERR_TIMEOUT;
        case I2C_ERR_NACK:    return COMM_ERR_NACK;   /* unique to I2C */
        default:              return COMM_ERR_IO;
    }
}

static comm_status_t i2c_open(comm_if_t *self)
{
    /* The bus is shared, so init takes no per-device parameters; the address is
     * applied per transfer below. ctx is unused at open time. */
    i2c1_init();
    notify(self, COMM_OK);
    return COMM_OK;
}

static comm_status_t i2c_write(comm_if_t *self, const uint8_t *d, uint32_t n)
{
    /* Address the target slave (from ctx) and send the buffer with a STOP. */
    const comm_i2c_ctx_t *c = (const comm_i2c_ctx_t *)self->ctx;
    const comm_status_t st = i2c_translate(i2c1_write(c->addr7, d, n, 1));
    notify(self, st);
    return st;
}

static comm_status_t i2c_read(comm_if_t *self, uint8_t *d, uint32_t n)
{
    /* Same idea for receive: address the slave, read n bytes (driver handles
     * the ACK-until-last / NACK-last sequencing). */
    const comm_i2c_ctx_t *c = (const comm_i2c_ctx_t *)self->ctx;
    const comm_status_t st = i2c_translate(i2c1_read(c->addr7, d, n));
    notify(self, st);
    return st;
}

/* Install the I2C implementation into this interface instance. */
void comm_i2c_bind(comm_if_t *iface, comm_i2c_ctx_t *ctx)
{
    iface->open     = i2c_open;
    iface->write    = i2c_write;
    iface->read     = i2c_read;
    iface->ctx      = ctx;
    iface->name     = "I2C";
    iface->on_event = NULL;
}
