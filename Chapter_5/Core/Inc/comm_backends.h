/**
 ******************************************************************************
 * @file    comm_backends.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.5: UART/SPI/I2C backends for comm_if
 *
 * @details Three adapters that make the Chapter 5 drivers implement the
 *          comm_if_t virtual table. Each has a small context struct holding the
 *          parameters that transport needs, and a bind() function that wires
 *          the context and the function pointers into a comm_if_t the caller
 *          owns.
 ******************************************************************************
 */

#ifndef COMM_BACKENDS_H
#define COMM_BACKENDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "comm_if.h"
#include "spi_driver.h"
#include <stdint.h>

/* ---- UART backend (USART2 over the VCP) -------------------------------- */
typedef struct
{
    uint32_t baud;
} comm_uart_ctx_t;

/* ---- SPI backend (SPI2, software CS) ----------------------------------- */
typedef struct
{
    spi_config_t cfg;
} comm_spi_ctx_t;

/* ---- I2C backend (I2C1, 7-bit address) --------------------------------- */
typedef struct
{
    uint8_t addr7;
} comm_i2c_ctx_t;

/**
 * @brief  Bind a comm_if_t to the UART/SPI/I2C backend with the given context.
 *         The caller owns both the comm_if_t and the context structs.
 */
void comm_uart_bind(comm_if_t *iface, comm_uart_ctx_t *ctx);
void comm_spi_bind (comm_if_t *iface, comm_spi_ctx_t  *ctx);
void comm_i2c_bind (comm_if_t *iface, comm_i2c_ctx_t  *ctx);

#ifdef __cplusplus
}
#endif

#endif /* COMM_BACKENDS_H */
