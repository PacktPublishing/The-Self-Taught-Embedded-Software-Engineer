/**
 ******************************************************************************
 * @file    main_Ex_5_5_Reusable_Comm_API.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.5: One API over UART, SPI, and I2C
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   USART2 (PA2/PA3, VCP) · SPI2 (PB12-PB15, BME280) · I2C1 (PB8/PB9, ADXL345))
 * @output  SWV ITM Data Console, Stimulus Port 0
 *
 * @details Exercises 5.2 to 5.4 produced three drivers with three shapes. This
 *          one hides all three behind a single comm_if_t vtable, so the same
 *          application routine runs over any of them without being recompiled
 *          against a particular bus.
 *
 *          Three things this build demonstrates:
 *
 *          1. **One routine, three transports.** send_probe() is written once
 *             and dispatched over UART, SPI and I2C from an array. It names no
 *             driver, no register, and no bus.
 *
 *          2. **A registered callback.** Each interface is given an on_event
 *             hook before anything is opened, so the driver layer reports every
 *             completion back into application code it knows nothing about.
 *             This is the callback registration from *Chapter 3, Section 3.5*
 *             serving as a driver completion notification. Exercise 5.2b showed
 *             the other half of the pattern, a callback that runs in interrupt
 *             context and may therefore do far less.
 *
 *          3. **A shared error vocabulary that actually reports errors.** The
 *             last channel is deliberately pointed at an I2C address where
 *             nothing lives, so the run ends on a real COMM_ERR_NACK printed by
 *             the same line of code that printed the successes. A status enum
 *             you never see fail teaches nothing.
 *
 *          Expected output: OK on all four operations of the UART, SPI and I2C
 *          channels, then NACK on the GONE channel's write. If the real I2C
 *          channel also reports NACK, the payload is landing on registers the
 *          ADXL345 will not accept; see the note on send_probe() below.
 *
 *          On the wire: the UART terminal shows "$PROBE", and MOSI carries
 *          24 50 52 4F 42 45 framed by the software chip select.
 ******************************************************************************
 */

#include "main.h"
#include "comm_if.h"
#include "comm_backends.h"
#include "spi_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* The ADXL345 at its default address, and an address nothing answers at. */
#define ADXL345_ADDR    (0x53U)
#define EMPTY_ADDR      (0x21U)

/* =========================================================================
 * on_comm_event — the application's completion callback.
 *
 * Registered on each interface below. The backends call it after every
 * operation with the result, so the application can watch the driver layer
 * without the driver layer knowing anything about the application. The
 * interface passes itself back as the first argument, which is what lets one
 * shared callback serve every channel and still say which one it is reporting.
 *
 * These backends block, so this runs in thread context and printing is safe.
 * Contrast Exercise 5.2b, where the equivalent hook runs inside the USART2
 * interrupt and may do nothing but set a flag.
 * ========================================================================= */
static uint32_t s_events = 0U;

static void on_comm_event(const comm_if_t *self, comm_status_t st)
{
    s_events++;
    printf("      (callback) %-4s completed with %s\r\n",
           self->name, comm_status_str(st));
}

/* Transport-agnostic application code. */
static void send_probe(comm_if_t *ch)
{
    /* A payload that is benign on all three buses, which is harder than it
     * sounds and worth spelling out.
     *
     * On UART these are characters on the wire, and the terminal shows
     * "$PROBE". On SPI they are six bytes clocked out on MOSI. On I2C they are
     * not data at all: the first byte after the address is the REGISTER
     * POINTER, and everything after it is written into the device starting
     * there, auto-incrementing.
     *
     * So a payload chosen to look nice in a terminal is a payload aimed at
     * whatever registers those characters happen to spell, and the abstraction
     * will not warn you.
     *
     * 0x24 is THRESH_ACT, and it is chosen so the five characters that follow
     * land on documented read/write registers rather than reserved ones:
     *
     *      0x24 THRESH_ACT     'P'
     *      0x25 THRESH_INACT   'R'
     *      0x26 TIME_INACT     'O'
     *      0x27 ACT_INACT_CTL  'B'
     *      0x28 THRESH_FF      'E'
     *
     * All five configure activity, inactivity, and free-fall detection, and
     * none of them touch the acceleration data path or the OFSX/OFSY/OFSZ
     * offset registers, so measured readings are unaffected. Detection itself
     * stays silent because INT_ENABLE (0x2E) is never written in this chapter
     * and its reset value is 0x00: nothing is routed to a pin, and the only
     * trace is a bit or two in INT_SOURCE (0x30), which nothing reads. The
     * settings do persist until power-cycle, which is worth knowing but has no
     * observable effect on Exercise 5.4.
     *
     * The general rule this is an instance of: a transport abstraction hides
     * how bytes travel, not what they mean at the far end. Picking the payload
     * is the application's job, and it is the one thing comm_if_t cannot do
     * for you. */
    const uint8_t payload[] = { 0x24U, 'P', 'R', 'O', 'B', 'E' };

    comm_status_t st = comm_open(ch);
    printf("[%s] open  -> %s\r\n", ch->name, comm_status_str(st));

    st = comm_write(ch, payload, (uint32_t)sizeof(payload));
    printf("[%s] write -> %s\r\n", ch->name, comm_status_str(st));
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 5 - Exercise 5.5: reusable comm interface (UART/SPI/I2C)\r\n");

    /* Per-transport contexts: each carries only what its own bus needs. */
    comm_uart_ctx_t uart_ctx = { .baud  = 115200UL };
    comm_spi_ctx_t  spi_ctx  = { .cfg   = { .mode = SPI_MODE0, .br = 4U } };
    comm_i2c_ctx_t  i2c_ctx  = { .addr7 = ADXL345_ADDR };
    comm_i2c_ctx_t  gone_ctx = { .addr7 = EMPTY_ADDR };

    /* Bind four interfaces to the same vtable shape. */
    comm_if_t uart_if, spi_if, i2c_if, gone_if;
    comm_uart_bind(&uart_if, &uart_ctx);
    comm_spi_bind (&spi_if,  &spi_ctx);
    comm_i2c_bind (&i2c_if,  &i2c_ctx);
    comm_i2c_bind (&gone_if, &gone_ctx);

    /* Register the completion callback. comm_*_bind() leaves on_event NULL, so
     * this is the line that turns the hook on. Assigning NULL again turns it
     * back off, and every backend keeps working exactly as before — which is
     * what "optional" has to mean for a hook to be worth having. */
    uart_if.on_event = on_comm_event;
    spi_if.on_event  = on_comm_event;
    i2c_if.on_event  = on_comm_event;
    gone_if.on_event = on_comm_event;

    /* A clearer label for the channel that is expected to fail. */
    gone_if.name = "GONE";

    /* Same application routine, four channels — selected at runtime. */
    comm_if_t *channels[] = { &uart_if, &spi_if, &i2c_if, &gone_if };
    for (uint32_t i = 0U; i < (sizeof(channels) / sizeof(channels[0])); i++)
    {
        send_probe(channels[i]);
    }

    printf("\r\nDone. One send_probe() ran over four channels; %lu callbacks fired.\r\n",
           (unsigned long)s_events);
    printf("The GONE channel reported its failure through the same enum as the rest.\r\n");

    while (1)
    {
        /* trap so the SWV log stays visible */
    }
}

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
