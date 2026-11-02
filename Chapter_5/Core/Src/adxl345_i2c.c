/**
 ******************************************************************************
 * @file    adxl345_i2c.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.4: ADXL345 over I2C (implementation)
 *
 * @details Access rules (Analog Devices ADXL345 datasheet):
 *            DEVID (0x00)        reads the fixed signature 0xE5
 *            BW_RATE (0x2C)      bits[3:0] set the output data rate
 *            POWER_CTL (0x2D)    bit 3 (Measure) leaves standby and starts
 *                                sampling; the part boots in standby
 *            DATA_FORMAT (0x31)  bits[1:0] range, bit 3 FULL_RES,
 *                                bit 2 justify (kept 0 = right-justified)
 *            DATAX0..DATAZ1      0x32..0x37, little-endian per axis, two's
 *                                complement; auto-incremented on a burst read
 *
 *          Scaling: in full-resolution mode the sensitivity is fixed at
 *          256 LSB/g regardless of range, i.e. 1000/256 = 3.906 mg/LSB. We use
 *          integer maths (raw * 1000 / 256) to avoid floating point.
 ******************************************************************************
 */

#include "adxl345_i2c.h"
#include "i2c_driver.h"
#include <stddef.h>

/* POWER_CTL Measure bit and DATA_FORMAT FULL_RES bit. */
#define ADXL_POWER_CTL_MEASURE   (0x08U)
#define ADXL_DATA_FORMAT_FULLRES (0x08U)

/* Full-resolution sensitivity: a constant ~3.9 mg per count, i.e. 256 LSB per
 * g, at every measurement range. That constancy is the whole point of the
 * mode: widening the range adds bits instead of coarsening the step. */
#define ADXL_LSB_PER_G_FULLRES   (256)

/* The configuration the driver is actually running, remembered at init so the
 * scaling function does not have to guess. Storing it is what keeps
 * adxl345_read_mg() honest: sensitivity depends on how the part was set up,
 * and a driver that accepts a setting it then ignores is worse than one that
 * never offered the choice. */
static adxl_config_t s_cfg = { ADXL_RANGE_4G, ADXL_ODR_100HZ, 1U };

/* =========================================================================
 * adxl_lsb_per_g — counts per g for the active configuration.
 *
 * In FULL_RES mode the step is fixed at ~3.9 mg, so there are always 256
 * counts in a g. In fixed 10-bit mode the ten bits are stretched across
 * whichever range is selected, so the count halves each time the range
 * doubles: 256 at +/-2 g, 128 at +/-4 g, 64 at +/-8 g, and 32 at +/-16 g
 * (ADXL345 datasheet, sensitivity table).
 * ========================================================================= */
static int32_t adxl_lsb_per_g(void)
{
    if (s_cfg.full_res != 0U)
    {
        return ADXL_LSB_PER_G_FULLRES;
    }

    switch (s_cfg.range)
    {
        case ADXL_RANGE_2G:  return 256;
        case ADXL_RANGE_4G:  return 128;
        case ADXL_RANGE_8G:  return 64;
        case ADXL_RANGE_16G: return 32;
        default:             return 256;
    }
}

/* ---- small register helpers over the I2C driver ------------------------ */
static adxl_status_t reg_write(uint8_t reg, uint8_t value)
{
    const uint8_t payload[2] = { reg, value };
    return (i2c1_write(ADXL_I2C_ADDR, payload, 2U, 1) == I2C_OK)
               ? ADXL_OK : ADXL_ERR_BUS;
}

uint8_t adxl345_read_id(void)
{
    uint8_t id = 0U;
    (void)i2c1_mem_read(ADXL_I2C_ADDR, (uint8_t)ADXL_REG_DEVID, &id, 1U);
    return id;
}

adxl_status_t adxl345_init(const adxl_config_t *cfg)
{
    /* Sensible defaults if the caller passes NULL. */
    adxl_config_t local = { ADXL_RANGE_4G, ADXL_ODR_100HZ, 1U };
    if (cfg != NULL)
    {
        local = *cfg;
    }

    /* Presence check: the ADXL345 has a fixed device ID, identify the part before configuring it. */
    if (adxl345_read_id() != ADXL_DEVICE_ID)
    {
        return ADXL_ERR_ID;
    }

    /* Output data rate (BW_RATE bits [3:0]). */
    if (reg_write((uint8_t)ADXL_REG_BW_RATE, (uint8_t)local.odr) != ADXL_OK)
    {
        return ADXL_ERR_BUS;
    }

    /* DATA_FORMAT: range in [1:0], optional FULL_RES in bit 3. */
    uint8_t fmt = (uint8_t)(local.range & 0x3U);
    if (local.full_res != 0U)
    {
        fmt |= ADXL_DATA_FORMAT_FULLRES;
    }
    if (reg_write((uint8_t)ADXL_REG_DATA_FORMAT, fmt) != ADXL_OK)
    {
        return ADXL_ERR_BUS;
    }

    /* Leave standby: set the Measure bit so conversions begin. */
    if (reg_write((uint8_t)ADXL_REG_POWER_CTL, ADXL_POWER_CTL_MEASURE) != ADXL_OK)
    {
        return ADXL_ERR_BUS;
    }

    /* Remember what we actually programmed, so the milli-g conversion below
     * scales by the sensitivity this configuration really has.              */
    s_cfg = local;

    return ADXL_OK;
}

adxl_status_t adxl345_read_raw(adxl_sample_t *out)
{
    if (out == NULL)
    {
        return ADXL_ERR_PARAM;
    }

    /* One transaction: point at DATAX0 and burst-read six auto-incremented
     * bytes (X0,X1, Y0,Y1, Z0,Z1). Each axis is little-endian, two's comp.  */
    uint8_t b[6];
    if (i2c1_mem_read(ADXL_I2C_ADDR, (uint8_t)ADXL_REG_DATAX0, b, 6U) != I2C_OK)
    {
        return ADXL_ERR_BUS;
    }

    out->x = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
    out->y = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
    out->z = (int16_t)(((uint16_t)b[5] << 8) | b[4]);
    return ADXL_OK;
}

adxl_status_t adxl345_read_mg(int32_t *x_mg, int32_t *y_mg, int32_t *z_mg)
{
    if ((x_mg == NULL) || (y_mg == NULL) || (z_mg == NULL))
    {
        return ADXL_ERR_PARAM;
    }

    adxl_sample_t s;
    const adxl_status_t st = adxl345_read_raw(&s);
    if (st != ADXL_OK)
    {
        return st;
    }

    /* milli-g = raw * 1000 / counts-per-g, with counts-per-g taken from the
     * configuration this driver actually programmed rather than assumed.    */
    const int32_t lsb_per_g = adxl_lsb_per_g();

    *x_mg = ((int32_t)s.x * 1000) / lsb_per_g;
    *y_mg = ((int32_t)s.y * 1000) / lsb_per_g;
    *z_mg = ((int32_t)s.z * 1000) / lsb_per_g;
    return ADXL_OK;
}
