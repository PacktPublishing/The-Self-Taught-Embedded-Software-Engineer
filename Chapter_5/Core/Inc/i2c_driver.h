/**
 ******************************************************************************
 * @file    i2c_driver.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.4: Register-level I2C1 master driver
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   I2C1 on the Arduino header:
 *              PB8 = I2C1_SCL  (AF4, open-drain, external pull-up to 3V3)
 *              PB9 = I2C1_SDA  (AF4, open-drain, external pull-up to 3V3)
 *
 * @details A blocking 7-bit I2C master in Standard mode (100 kHz). It exposes
 *          the three operations sensor code actually needs:
 *            - a raw write (address + N bytes, optional STOP),
 *            - a raw read  (address + N bytes), and
 *            - a register read using a repeated START (write the register
 *              pointer, then read) — the dominant sensor access pattern.
 *
 *          7-bit addresses are passed un-shifted (e.g. 0x53 for the ADXL345);
 *          the driver forms the read/write byte internally. Clock stretching is
 *          handled implicitly: the status-flag waits simply block while a slave
 *          holds SCL low.
 ******************************************************************************
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include <stdint.h>

typedef enum
{
    I2C_OK = 0,
    I2C_ERR_PARAM,      /* bad argument                                    */
    I2C_ERR_TIMEOUT,    /* a status flag never asserted (no/stuck slave)   */
    I2C_ERR_NACK        /* the slave did not acknowledge its address/data  */
} i2c_status_t;

/** @brief Configure PB8/PB9 as I2C1 AF4 open-drain and start I2C1 at 100 kHz. */
void i2c1_init(void);

/**
 * @brief  Master-transmit @p len bytes to a 7-bit address.
 * @param  addr7  7-bit slave address (un-shifted).
 * @param  data   Source buffer (may be NULL only if len == 0).
 * @param  len    Byte count.
 * @param  stop   Non-zero to issue a STOP at the end; 0 to leave the bus held
 *                for a following repeated START.
 */
i2c_status_t i2c1_write(uint8_t addr7, const uint8_t *data, uint32_t len, int stop);

/**
 * @brief  Master-receive @p len bytes from a 7-bit address (issues STOP).
 */
i2c_status_t i2c1_read(uint8_t addr7, uint8_t *data, uint32_t len);

/**
 * @brief  Read @p len bytes from register @p reg of @p addr7 using the standard
 *         write-pointer / repeated-START / read sequence.
 */
i2c_status_t i2c1_mem_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* I2C_DRIVER_H */
