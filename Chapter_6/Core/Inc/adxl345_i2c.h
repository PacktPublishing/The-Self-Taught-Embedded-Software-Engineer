/**
 ******************************************************************************
 * @file    adxl345_i2c.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.4: ADXL345 3-axis accelerometer over I2C
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @sensor  Analog Devices ADXL345 digital accelerometer, I2C mode.
 *          7-bit address 0x53 with the SDO/ALT-ADDRESS pin tied low
 *          (0x1D if tied high). On a GY-291 module SDO usually idles low.
 *
 * @details Sits on top of the register-level I2C driver (i2c_driver.c) and
 *          reads acceleration on X, Y and Z. 
 *
 *          Authored from the Analog Devices ADXL345 datasheet. The register
 *          set is exposed as a typed enum (adxl_reg_t) and the API uses the
 *          book's config-struct / status-enum style.
 *
 *          I2C-specific note: on a burst read the ADXL345 auto-increments 
 *          its register pointer, so reading the six data registers only 
 *          needs a single "set pointer to DATAX0, read 6" transaction — 
 *          there is no multi-byte-enable bit to set (that bit exists only 
 *          on the SPI interface).
 *
 *          Features beyond a bare "read the axes" driver:
 *            - device-ID check against the fixed 0xE5 signature,
 *            - selectable output data rate (BW_RATE) and measurement range,
 *            - full-resolution mode with correct signed scaling to milli-g.
 ******************************************************************************
 */

#ifndef ADXL345_I2C_H
#define ADXL345_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Default 7-bit I2C address (SDO/ALT ADDRESS low). Tie SDO high for 0x1D. */
#define ADXL_I2C_ADDR       (0x53U)
/* Fixed value of the device-ID register — used as a presence check. */
#define ADXL_DEVICE_ID      (0xE5U)

/* Register map (subset used here). */
typedef enum
{
    ADXL_REG_DEVID       = 0x00U,   /* R  device ID, reads 0xE5            */
    ADXL_REG_BW_RATE     = 0x2CU,   /* RW output data rate / low power     */
    ADXL_REG_POWER_CTL   = 0x2DU,   /* RW sleep / measure control          */
    ADXL_REG_DATA_FORMAT = 0x31U,   /* RW range, full-res, justify         */
    ADXL_REG_DATAX0      = 0x32U    /* R  first of six data bytes (X0..Z1) */
} adxl_reg_t;

/* Measurement range (DATA_FORMAT bits [1:0]). */
typedef enum
{
    ADXL_RANGE_2G  = 0x0U,
    ADXL_RANGE_4G  = 0x1U,
    ADXL_RANGE_8G  = 0x2U,
    ADXL_RANGE_16G = 0x3U
} adxl_range_t;

/* Output data rate (BW_RATE bits [3:0]); a useful subset. */
typedef enum
{
    ADXL_ODR_12_5HZ = 0x07U,
    ADXL_ODR_25HZ   = 0x08U,
    ADXL_ODR_50HZ   = 0x09U,
    ADXL_ODR_100HZ  = 0x0AU,
    ADXL_ODR_200HZ  = 0x0BU,
    ADXL_ODR_400HZ  = 0x0CU
} adxl_odr_t;

typedef enum
{
    ADXL_OK = 0,
    ADXL_ERR_PARAM,   /* NULL argument                                     */
    ADXL_ERR_ID,      /* device-ID register did not read 0xE5              */
    ADXL_ERR_BUS      /* an underlying I2C transfer failed                 */
} adxl_status_t;

typedef struct
{
    adxl_range_t range;      /* +/-2/4/8/16 g                              */
    adxl_odr_t   odr;        /* output data rate                           */
    uint8_t      full_res;   /* 1 = full-resolution mode (fixed ~3.9 mg/LSB) */
} adxl_config_t;

/* One raw sample: signed 16-bit counts per axis (two's complement). */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} adxl_sample_t;

/**
 * @brief  Verify the device ID, program range/ODR/resolution, and start
 *         measurement mode. Assumes i2c1_init() has already run.
 * @param  cfg  Configuration (NULL selects +/-4 g, 100 Hz, full-res defaults).
 * @return ADXL_OK, ADXL_ERR_ID if 0xE5 is not read back, else ADXL_ERR_BUS.
 */
adxl_status_t adxl345_init(const adxl_config_t *cfg);

/** @brief Read the device-ID register (expected 0xE5). */
uint8_t adxl345_read_id(void);

/** @brief Read one raw X/Y/Z sample (six auto-incremented data registers). */
adxl_status_t adxl345_read_raw(adxl_sample_t *out);

/**
 * @brief  Read one sample already converted to milli-g on each axis.
 *         Full-resolution scaling is ~3.9 mg/LSB (256 LSB/g).
 */
adxl_status_t adxl345_read_mg(int32_t *x_mg, int32_t *y_mg, int32_t *z_mg);

#ifdef __cplusplus
}
#endif

#endif /* ADXL345_I2C_H */
