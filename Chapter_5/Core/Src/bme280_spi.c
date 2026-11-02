/**
 ******************************************************************************
 * @file    bme280_spi.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.3: BME280 temperature over SPI (implementation)
 *
 * @details BME280 SPI access rules (Bosch datasheet):
 *            - A control byte precedes each transfer. Bit 7 = R/W: 1 = read,
 *              0 = write. Bits 6:0 = register address.
 *            - In a multi-byte read the address auto-increments.
 *            - CS must stay low for the whole control+data sequence.
 *
 *          Registers used:
 *            0xD0 id | 0xE0 reset | 0xF3 status | 0xF4 ctrl_meas
 *            0x88..0x8D temperature calibration (dig_T1..T3)
 *            0xFA..0xFC raw temperature (20-bit, MSB:LSB:XLSB[7:4])
 *
 *          Temperature compensation is Bosch's published fixed-point algorithm;
 *          t_fine is retained between vars exactly as the datasheet specifies.
 ******************************************************************************
 */

#include "bme280_spi.h"
#include "spi_driver.h"
#include <stddef.h>   /* NULL */

/* Register addresses. */
#define REG_ID          (0xD0U)
#define REG_RESET       (0xE0U)
#define REG_STATUS      (0xF3U)
#define REG_CTRL_MEAS   (0xF4U)
#define REG_CALIB_T     (0x88U)   /* dig_T1 LSB starts here */
#define REG_TEMP_MSB    (0xFAU)

#define SPI_READ_BIT    (0x80U)   /* OR into the address byte to read */
#define SPI_WRITE_MASK  (0x7FU)   /* AND into the address byte to write */

/* Factory temperature calibration, read once in bme280_init(). */
static uint16_t s_dig_T1;
static int16_t  s_dig_T2;
static int16_t  s_dig_T3;
static int32_t  s_t_fine;

/* ---- low-level register helpers ---------------------------------------
 * Each helper frames one transaction between cs_low() and cs_high(). The SPI
 * driver returns a status, so the chip select is released on the error
 * path too — leaving CS asserted after a failed transfer would wedge the bus
 * for every transaction that followed.
 * ----------------------------------------------------------------------- */
static uint8_t bme_read_reg(uint8_t reg)
{
    uint8_t value = 0xFFU;   /* value seen when the bus is silent */

    spi2_cs_low();
    if (spi2_transfer((uint8_t)(reg | SPI_READ_BIT), NULL) == SPI_OK)
    {
        (void)spi2_transfer(0xFFU, &value);
    }
    spi2_cs_high();

    return value;
}

static void bme_read_regs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    spi2_cs_low();
    if (spi2_transfer((uint8_t)(reg | SPI_READ_BIT), NULL) == SPI_OK)
    {
        /* Send padding, capture data. The address auto-increments inside the
         * sensor, so one control byte covers the whole burst.               */
        (void)spi2_transfer_buf(NULL, buf, len);
    }
    spi2_cs_high();
}

static void bme_write_reg(uint8_t reg, uint8_t value)
{
    spi2_cs_low();
    if (spi2_transfer((uint8_t)(reg & SPI_WRITE_MASK), NULL) == SPI_OK)
    {
        (void)spi2_transfer(value, NULL);
    }
    spi2_cs_high();
}

uint8_t bme280_read_id(void)
{
    return bme_read_reg(REG_ID);
}

int bme280_init(void)
{
    if (bme280_read_id() != BME280_CHIP_ID)
    {
        return -1;   /* not a BME280 (or wiring/mode problem) */
    }

    /* Read the 6 temperature calibration bytes (little-endian per datasheet). */
    uint8_t c[6];
    bme_read_regs(REG_CALIB_T, c, 6U);
    s_dig_T1 = (uint16_t)((uint16_t)c[1] << 8) | c[0];
    s_dig_T2 = (int16_t)(((uint16_t)c[3] << 8) | c[2]);
    s_dig_T3 = (int16_t)(((uint16_t)c[5] << 8) | c[4]);

    /* Leave the sensor in sleep mode; each measurement uses forced mode. */
    bme_write_reg(REG_CTRL_MEAS, 0x00U);
    return 0;
}

int32_t bme280_read_temperature_c100(void)
{
    /* Forced mode, temperature oversampling x1:
     * ctrl_meas = osrs_t(001)<<5 | osrs_p(000)<<2 | mode(01). */
    bme_write_reg(REG_CTRL_MEAS, (0x1U << 5) | (0x0U << 2) | 0x1U);

    /* Wait for the conversion to finish: status bit 3 (measuring) clears. */
    uint32_t guard = 100000U;
    while (((bme_read_reg(REG_STATUS) & 0x08U) != 0U) && (guard != 0U))
    {
        guard--;
    }

    /* Read the 20-bit raw temperature (MSB, LSB, XLSB[7:4]). */
    uint8_t t[3];
    bme_read_regs(REG_TEMP_MSB, t, 3U);
    const int32_t adc_T = (int32_t)(((uint32_t)t[0] << 12) |
                                    ((uint32_t)t[1] << 4)  |
                                    ((uint32_t)t[2] >> 4));

    /* Bosch integer compensation (datasheet). Result is in 0.01 C. */
    const int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_dig_T1 << 1))) *
                          ((int32_t)s_dig_T2)) >> 11;
    const int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_dig_T1)) *
                            ((adc_T >> 4) - ((int32_t)s_dig_T1))) >> 12) *
                          ((int32_t)s_dig_T3)) >> 14;
    s_t_fine = var1 + var2;

    return (s_t_fine * 5 + 128) >> 8;
}
