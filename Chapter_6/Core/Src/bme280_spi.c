/**
 ******************************************************************************
 * @file    bme280_spi.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.4a: BME280 temperature, pressure and
 *          humidity over SPI (implementation)
 *
 * @details BME280 SPI access rules (Bosch BST-BME280-DS002):
 *            - A control byte precedes each transfer. Bit 7 = R/W: 1 = read,
 *              0 = write. Bits 6:0 = register address.
 *            - In a multi-byte read the address auto-increments.
 *            - CS must stay low for the whole control+data sequence.
 *
 *          Register groups used here (beyond the Chapter 5 temperature path):
 *            0xF2 ctrl_hum   - humidity oversampling (must be written BEFORE
 *                              ctrl_meas for the setting to take effect)
 *            0xF4 ctrl_meas  - temperature/pressure oversampling + mode
 *            0x88..0xA1      - temperature and pressure calibration + dig_H1
 *            0xE1..0xE7      - humidity calibration dig_H2..dig_H6
 *            0xF7..0xFE      - burst: pressure(3) temperature(3) humidity(2)
 *
 *          Compensation is Bosch's published 32-bit fixed-point algorithm from
 *          the datasheet. t_fine is computed by the temperature stage and then
 *          reused by the pressure and humidity stages exactly as the datasheet
 *          specifies, so the three quantities must be compensated in that
 *          order from the same sample.
 ******************************************************************************
 */

#include "bme280_spi.h"
#include "spi_driver.h"
#include <stddef.h>   /* NULL */

/* ---- register addresses ----------------------------------------------- */
#define REG_ID          (0xD0U)
#define REG_RESET       (0xE0U)
#define REG_CTRL_HUM    (0xF2U)
#define REG_STATUS      (0xF3U)
#define REG_CTRL_MEAS   (0xF4U)
#define REG_CALIB_TP    (0x88U)   /* dig_T1..dig_P9 + dig_H1 region start   */
#define REG_CALIB_H     (0xE1U)   /* dig_H2..dig_H6 region start            */
#define REG_CONFIG      (0xF5U)   /* t_sb[7:5], IIR filter[4:2] (normal mode)*/
#define REG_DATA        (0xF7U)   /* press_msb; burst of 8 bytes to 0xFE    */

#define SPI_READ_BIT    (0x80U)   /* OR into the address byte to read       */
#define SPI_WRITE_MASK  (0x7FU)   /* AND into the address byte to write     */

#define STATUS_MEASURING (0x08U)  /* ctrl status bit: conversion in progress */

/* ---- factory calibration, read once in bme280_init() ------------------ */
static uint16_t s_dig_T1;
static int16_t  s_dig_T2, s_dig_T3;
static uint16_t s_dig_P1;
static int16_t  s_dig_P2, s_dig_P3, s_dig_P4, s_dig_P5,
                s_dig_P6, s_dig_P7, s_dig_P8, s_dig_P9;
static uint8_t  s_dig_H1, s_dig_H3;
static int16_t  s_dig_H2, s_dig_H4, s_dig_H5;
static int8_t   s_dig_H6;

/* Shared intermediate temperature term; set by the temperature stage and
 * consumed by the pressure and humidity stages within the same read.       */
static int32_t  s_t_fine;

/* ---- time source (injected, so the driver stays free of the scheduler) --
 * The blocking read needs a real deadline, not a spin count: a loop counter
 * silently changes duration whenever the SPI prescaler changes. The caller
 * hands in its own millisecond source (the super-loop passes millis()), which
 * keeps this driver independent of any particular time base. If no source is
 * registered the driver falls back to a bounded poll count so it can still
 * never hang.                                                               */
static uint32_t (*s_now_ms)(void) = NULL;

/* When the current forced-mode conversion was triggered. Used by
 * bme280_measurement_ready() to refuse to answer "yes" too early; see the
 * notes on BME280_CONVERSION_TIME_MS in the header for why that matters. */
static uint32_t s_meas_start_ms = 0U;

void bme280_set_time_source(uint32_t (*now_ms)(void))
{
    s_now_ms = now_ms;
}

/* ---- low-level register helpers ---------------------------------------
 * Each helper frames one transaction between cs_low() and cs_high(). The SPI
 * driver returns a status, so the chip select is released on the error path
 * too: leaving CS asserted after a failed transfer would wedge the bus for
 * every transaction that followed. This is the Chapter 5 audited pattern.
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

/* Compensation stages are defined further down; declared here so the burst
 * decoder can sit beside the other low-level helpers.                      */
static int32_t  compensate_temperature(int32_t adc_T);
static uint32_t compensate_pressure(int32_t adc_P);
static uint32_t compensate_humidity(int32_t adc_H);

/* ---- shared decode of one 8-byte measurement burst ---------------------
 * Used by every read path (blocking, normal mode and split phase) so the
 * register layout and the compensation order are stated exactly once.
 * Order matters: temperature must be compensated first because it sets
 * t_fine, which the pressure and humidity stages then consume.             */
static void bme_decode_burst(const uint8_t *d, bme280_reading_t *out)
{
    const int32_t adc_P = (int32_t)(((uint32_t)d[0] << 12) |
                                    ((uint32_t)d[1] << 4)  |
                                    ((uint32_t)d[2] >> 4));
    const int32_t adc_T = (int32_t)(((uint32_t)d[3] << 12) |
                                    ((uint32_t)d[4] << 4)  |
                                    ((uint32_t)d[5] >> 4));
    const int32_t adc_H = (int32_t)(((uint32_t)d[6] << 8) | d[7]);

    out->temperature_c100 = compensate_temperature(adc_T);
    out->pressure_pa      = compensate_pressure(adc_P);
    out->humidity_rh_1024 = compensate_humidity(adc_H);
}

uint8_t bme280_read_id(void)
{
    return bme_read_reg(REG_ID);
}

/* ---- calibration read -------------------------------------------------- */
static void bme_read_calibration(void)
{
    /* Block 1: 0x88..0xA1 holds dig_T1..dig_T3, dig_P1..dig_P9 and dig_H1. */
    uint8_t a[26];
    bme_read_regs(REG_CALIB_TP, a, 26U);

    s_dig_T1 = (uint16_t)((uint16_t)a[1]  << 8) | a[0];
    s_dig_T2 = (int16_t) (((uint16_t)a[3]  << 8) | a[2]);
    s_dig_T3 = (int16_t) (((uint16_t)a[5]  << 8) | a[4]);

    s_dig_P1 = (uint16_t)((uint16_t)a[7]  << 8) | a[6];
    s_dig_P2 = (int16_t) (((uint16_t)a[9]  << 8) | a[8]);
    s_dig_P3 = (int16_t) (((uint16_t)a[11] << 8) | a[10]);
    s_dig_P4 = (int16_t) (((uint16_t)a[13] << 8) | a[12]);
    s_dig_P5 = (int16_t) (((uint16_t)a[15] << 8) | a[14]);
    s_dig_P6 = (int16_t) (((uint16_t)a[17] << 8) | a[16]);
    s_dig_P7 = (int16_t) (((uint16_t)a[19] << 8) | a[18]);
    s_dig_P8 = (int16_t) (((uint16_t)a[21] << 8) | a[20]);
    s_dig_P9 = (int16_t) (((uint16_t)a[23] << 8) | a[22]);
    /* a[24] is 0xA0 (reserved); dig_H1 is at 0xA1 = a[25]. */
    s_dig_H1 = a[25];

    /* Block 2: 0xE1..0xE7 holds dig_H2..dig_H6, some fields split across
     * a shared nibble in 0xE5 (see the datasheet calibration table).        */
    uint8_t b[7];
    bme_read_regs(REG_CALIB_H, b, 7U);

    s_dig_H2 = (int16_t)(((uint16_t)b[1] << 8) | b[0]);       /* 0xE1/0xE2   */
    s_dig_H3 = b[2];                                          /* 0xE3        */
    s_dig_H4 = (int16_t)(((int16_t)(int8_t)b[3] << 4) |       /* 0xE4[7:0]   */
                         (b[4] & 0x0FU));                     /* 0xE5[3:0]   */
    s_dig_H5 = (int16_t)(((int16_t)(int8_t)b[5] << 4) |       /* 0xE6[7:0]   */
                         (b[4] >> 4));                        /* 0xE5[7:4]   */
    s_dig_H6 = (int8_t)b[6];                                  /* 0xE7        */
}

int bme280_init(void)
{
    if (bme280_read_id() != BME280_CHIP_ID)
    {
        return -1;   /* not a BME280 (or wiring/mode problem) */
    }

    bme_read_calibration();

    /* Humidity oversampling x1. ctrl_hum only latches on the next write to
     * ctrl_meas, so it is programmed first and ctrl_meas follows.           */
    bme_write_reg(REG_CTRL_HUM, 0x01U);

    /* Leave the sensor asleep; each measurement uses forced mode. */
    bme_write_reg(REG_CTRL_MEAS, 0x00U);
    return 0;
}

/* ---- Bosch fixed-point compensation (datasheet BST-BME280-DS002) ------ */
static int32_t compensate_temperature(int32_t adc_T)
{
    const int32_t var1 = ((((adc_T >> 3) - ((int32_t)s_dig_T1 << 1))) *
                          ((int32_t)s_dig_T2)) >> 11;
    const int32_t var2 = (((((adc_T >> 4) - ((int32_t)s_dig_T1)) *
                            ((adc_T >> 4) - ((int32_t)s_dig_T1))) >> 12) *
                          ((int32_t)s_dig_T3)) >> 14;
    s_t_fine = var1 + var2;
    return (s_t_fine * 5 + 128) >> 8;         /* 0.01 C */
}

static uint32_t compensate_pressure(int32_t adc_P)
{
    int32_t var1 = (((int32_t)s_t_fine) >> 1) - (int32_t)64000;
    int32_t var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)s_dig_P6);
    var2 = var2 + ((var1 * ((int32_t)s_dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)s_dig_P4) << 16);
    var1 = (((s_dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
            ((((int32_t)s_dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)s_dig_P1)) >> 15);
    if (var1 == 0)
    {
        return 0U;   /* avoid a divide-by-zero on an uncalibrated part */
    }

    uint32_t p = (uint32_t)(((int32_t)1048576 - adc_P) - (var2 >> 12)) * 3125U;
    if (p < 0x80000000U)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2U;
    }
    var1 = (((int32_t)s_dig_P9) *
            ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)s_dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + s_dig_P7) >> 4));
    return p;        /* Pa */
}

static uint32_t compensate_humidity(int32_t adc_H)
{
    int32_t v = s_t_fine - (int32_t)76800;
    v = (((((adc_H << 14) - (((int32_t)s_dig_H4) << 20) -
            (((int32_t)s_dig_H5) * v)) + ((int32_t)16384)) >> 15) *
         (((((((v * ((int32_t)s_dig_H6)) >> 10) *
             (((v * ((int32_t)s_dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) * ((int32_t)s_dig_H2) + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)s_dig_H1)) >> 4);
    v = (v < 0) ? 0 : v;
    v = (v > 419430400) ? 419430400 : v;
    return (uint32_t)(v >> 12);   /* Q22.10: %RH = value / 1024 */
}

/* ========================================================================= *
 *                                                                           *
 *  THREE WAYS TO GET A READING                                              *
 *                                                                           *
 *  The rest of this file is three public entry paths to the same sensor,    *
 *  laid out in the order Section 6.4 presents them. They share every helper *
 *  above, so the only thing that differs between them is who does the       *
 *  waiting.                                                                 *
 *                                                                           *
 *    Path 1  blocking     one call, waits ~8 ms       for Chapter 8 / RTOS  *
 *    Path 2  normal mode  sensor free-runs, no wait   the cheap fix         *
 *    Path 3  split phase  the loop does the waiting   the general fix       *
 *                                                                           *
 *  Note the direction of reuse: Path 1 is built out of Path 3 (it starts a  *
 *  conversion, polls it, then fetches), and Path 2 borrows Path 3's fetch.  *
 *  So the split-phase functions are the primitives; the other two paths are *
 *  convenience wrappers around them. That is worth seeing, because it is    *
 *  the usual shape of a well-factored driver: expose the fine-grained       *
 *  operations, then build the easy call on top rather than the reverse.     *
 *                                                                           *
 * ========================================================================= */

/* ======================================================================== *
 *  PATH 1 - blocking whole-transaction read.                                *
 *                                                                           *
 *  The Chapter 5 shape: ask for a reading, get a reading. Simple to call    *
 *  and simple to read, but it does not return until the sensor is done, so  *
 *  it must not be used from the cooperative super-loop. Kept because the    *
 *  Chapter 8 capstone calls it from an RTOS task, where only that task      *
 *  blocks and the rest of the system keeps running.                         *
 * ======================================================================== */

int bme280_read_all(bme280_reading_t *out)
{
    if (out == NULL)
    {
        return -1;
    }

    (void)bme280_start_measurement();          /* Path 3 primitive */

    /* Bounded wait. With a registered time source this is a real millisecond
     * deadline. Computed from the formula in BST-BME280-DS002 Section 9.1 for
     * this driver's x1/x1/x1 oversampling, one conversion takes 8.0 ms
     * typical and 9.3 ms maximum, so BME280_CONVERSION_TIMEOUT_MS (20 ms)
     * leaves headroom. Without a time source it degrades to a bounded poll
     * count, which is less precise but still cannot hang.                   */
    if (s_now_ms != NULL)
    {
        const uint32_t start = s_now_ms();
        while (bme280_measurement_ready() == 0)   /* Path 3 primitive */
        {
            if ((uint32_t)(s_now_ms() - start) >= BME280_CONVERSION_TIMEOUT_MS)
            {
                return -1;   /* conversion never completed */
            }
        }
    }
    else
    {
        uint32_t guard = 100000U;
        while ((bme280_measurement_ready() == 0) && (guard != 0U))
        {
            guard--;
        }
        if (guard == 0U)
        {
            return -1;
        }
    }

    return bme280_fetch(out);                  /* Path 3 primitive */
}

int32_t bme280_read_temperature_c100(void)
{
    /* Chapter 5 compatibility wrapper: the earlier chapters only ever asked
     * for temperature, and their code still links against this name.        */
    bme280_reading_t r;
    if (bme280_read_all(&r) != 0)
    {
        return 0;
    }
    return r.temperature_c100;
}

/* ======================================================================== *
 *  PATH 2 - normal mode: the sensor free-runs, so no read ever waits.       *
 *                                                                           *
 *  Two register writes replace the whole state machine. The cost is that    *
 *  the caller no longer knows how old the sample is: it is whatever the     *
 *  sensor produced up to one standby interval ago. Section 6.4 shows this   *
 *  before Path 3 on purpose, because reconfiguring a peripheral often beats *
 *  writing code around it.                                                  *
 * ======================================================================== */

int bme280_set_mode_normal(uint8_t standby_sel, uint8_t filter_sel)
{
    /* config (0xF5): t_sb in [7:5], IIR filter coefficient in [4:2]. Written
     * while the sensor is still asleep, because in normal mode parts of this
     * register are ignored unless the device is in sleep.                   */
    bme_write_reg(REG_CONFIG, (uint8_t)(((standby_sel & 0x7U) << 5) |
                                        ((filter_sel  & 0x7U) << 2)));

    /* ctrl_meas (0xF4): same x1 oversampling as the forced path, mode = 11
     * (normal). From here the sensor cycles measure -> standby on its own.  */
    bme_write_reg(REG_CTRL_MEAS, (0x1U << 5) | (0x1U << 2) | 0x3U);
    return 0;
}

int bme280_read_latest(bme280_reading_t *out)
{
    /* No status poll, no trigger: just take whatever the free-running sensor
     * last wrote into the data registers. This is the entire cheap fix, and
     * it is Path 3's fetch with nothing in front of it.                     */
    return bme280_fetch(out);
}

int bme280_set_mode_sleep(void)
{
    bme_write_reg(REG_CTRL_MEAS, 0x00U);
    return 0;
}

/* ======================================================================== *
 *  PATH 3 - split phase: forced mode, but the wait belongs to the caller.   *
 *                                                                           *
 *  Start the conversion and return; poll cheaply on later loop iterations;  *
 *  fetch when the sensor says it is done. Nothing here ever waits, so a     *
 *  conversion spans several passes of the super-loop instead of stalling    *
 *  one. This is the pattern to reach for whenever a device cannot be told   *
 *  to free-run: a flash page erase, an ADC conversion, a DMA completion.    *
 *                                                                           *
 *  These three are also the primitives Path 1 and Path 2 are built from.    *
 * ======================================================================== */

int bme280_start_measurement(void)
{
    /* Stamp the start time BEFORE the trigger, so the elapsed-time guard in
     * bme280_measurement_ready() can never under-count the conversion.      */
    if (s_now_ms != NULL)
    {
        s_meas_start_ms = s_now_ms();
    }

    /* Forced mode, temperature and pressure oversampling x1:
     * ctrl_meas = osrs_t(001)<<5 | osrs_p(001)<<2 | mode(01). Humidity
     * oversampling was latched by the ctrl_hum write in bme280_init().      */
    bme_write_reg(REG_CTRL_MEAS, (0x1U << 5) | (0x1U << 2) | 0x1U);
    return 0;
}

int bme280_measurement_ready(void)
{
    /* Guard first, status bit second. Both must agree.
     *
     * The status register's measuring bit is not set the instant ctrl_meas is
     * written; the device needs a moment to arm. Poll before that and the bit
     * still reads 0, which a naive loop reads as "finished" when the truth is
     * "not started". The read that follows then returns the previous
     * conversion's data. Nothing errors, the numbers stay plausible, and the
     * 8 ms wait silently costs nothing. Refusing to answer before the
     * conversion could physically have finished closes that hole.
     *
     * Unsigned subtraction, so this stays correct across the millis() wrap. */
    if (s_now_ms != NULL)
    {
        if ((uint32_t)(s_now_ms() - s_meas_start_ms) < BME280_CONVERSION_TIME_MS)
        {
            return 0;   /* too early to even ask */
        }
    }

    /* Enough time has passed; now the bit means what it says. */
    return ((bme_read_reg(REG_STATUS) & STATUS_MEASURING) == 0U) ? 1 : 0;
}

int bme280_fetch(bme280_reading_t *out)
{
    if (out == NULL)
    {
        return -1;
    }

    /* One burst read of pressure(3), temperature(3), humidity(2). */
    uint8_t d[8];
    bme_read_regs(REG_DATA, d, 8U);
    bme_decode_burst(d, out);
    return 0;
}
