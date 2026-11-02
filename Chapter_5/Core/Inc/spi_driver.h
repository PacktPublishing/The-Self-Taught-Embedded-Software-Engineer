/**
 ******************************************************************************
 * @file    spi_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.3: Register-level SPI2 master driver
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   SPI2 on the Morpho header (chosen so PA5/LD2 and PB3/SWO stay free):
 *              PB13 = SPI2_SCK   (AF5)
 *              PB14 = SPI2_MISO  (AF5)
 *              PB15 = SPI2_MOSI  (AF5)
 *              PB12 = chip select (plain GPIO, software-driven, active low)
 *
 * @details A full-duplex, 8-bit, MSB-first SPI master with software slave
 *          management. CPOL/CPHA are selectable so the reader can match a
 *          peripheral's required mode; the BME280 in Exercise 5.3 uses mode 0.
 *          Chip select is handled in software (spi2_cs_low/high) because most
 *          sensors want CS held across a multi-byte register transaction —
 *          something hardware NSS does not do cleanly.
 *
 *          The byte clock is set by the BR prescaler field. The default (/32 ->
 *          ~500 kHz from a 16 MHz APB1) is deliberately slow so the 24 MHz
 *          logic analyzer captures every edge with margin to spare.
 *
 *          Pin setup goes through the Chapter 4 gpio_driver, which is also
 *          where this chapter's OSPEEDR discussion lives: SPI is the first bus
 *          in the book where a pin's slew rate is a decision rather than a
 *          non-question.
 *
 *          Every entry point returns a spi_status_t and every hardware wait is
 *          bounded, matching uart_status_t and i2c_status_t.
 *          An unpowered or mis-wired sensor returns SPI_ERR_TIMEOUT instead
 *          of spinning forever on a status bit, and Section 5.5 has a real
 *          error to fold into its shared enum.
 ******************************************************************************
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Status / error codes. Same shape as uart_status_t so Section 5.5's
 * translator has something real to map. SPI has no addressing and no
 * acknowledge, so there is no NACK equivalent — an unresponsive slave shows
 * up as a timeout or as data that does not match what the datasheet promised.
 * ------------------------------------------------------------------------- */
typedef enum
{
    SPI_OK = 0,        /* operation completed                               */
    SPI_ERR_PARAM,     /* a NULL pointer or invalid argument was passed     */
    SPI_ERR_TIMEOUT    /* TXE, RXNE or BSY did not settle within the budget */
} spi_status_t;

/* SPI mode = (CPOL, CPHA). The BME280 supports mode 0 and mode 3. */
typedef enum
{
    SPI_MODE0 = 0,   /* CPOL=0, CPHA=0 */
    SPI_MODE1 = 1,   /* CPOL=0, CPHA=1 */
    SPI_MODE2 = 2,   /* CPOL=1, CPHA=0 */
    SPI_MODE3 = 3    /* CPOL=1, CPHA=1 */
} spi_mode_t;

/* Baud prescaler = APB1 / 2^(br+1). br=4 -> /32 -> 500 kHz at 16 MHz. */
typedef struct
{
    spi_mode_t mode;     /* clock polarity/phase                            */
    uint8_t    br;       /* BR[2:0] value, 0..7 (see RM0390 §26.7.1)        */
} spi_config_t;

/**
 * @brief  Configure PB12-PB15 and bring SPI2 up as an 8-bit MSB-first master.
 * @param  cfg  Mode and prescaler. NULL selects mode 0 at /32.
 * @return SPI_OK, or SPI_ERR_PARAM if the prescaler is out of range.
 */
spi_status_t spi2_init(const spi_config_t *cfg);

/** @brief Drive the software chip-select line low (assert / select slave). */
void spi2_cs_low(void);

/** @brief Drive the software chip-select line high (deassert). */
void spi2_cs_high(void);

/**
 * @brief  Full-duplex exchange of a single byte.
 * @param  tx  Byte to shift out on MOSI.
 * @param  rx  Where to store the byte simultaneously shifted in on MISO.
 *             May be NULL when only the transmit direction matters.
 * @return SPI_OK, or SPI_ERR_TIMEOUT if TXE or RXNE never asserted.
 */
spi_status_t spi2_transfer(uint8_t tx, uint8_t *rx);

/**
 * @brief  Full-duplex exchange of a buffer.
 * @param  tx   Source buffer, or NULL to send 0xFF padding (the "read" idiom).
 * @param  rx   Destination buffer, or NULL to discard what comes back.
 * @param  len  Number of bytes to exchange.
 * @return SPI_OK, SPI_ERR_PARAM if both buffers are NULL with len > 0, or
 *         SPI_ERR_TIMEOUT. Waits for BSY to clear so the caller can safely
 *         release the chip select on return.
 */
spi_status_t spi2_transfer_buf(const uint8_t *tx, uint8_t *rx, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* SPI_DRIVER_H */
