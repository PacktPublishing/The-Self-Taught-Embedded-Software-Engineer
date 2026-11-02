/**
 ******************************************************************************
 * @file    bme280_spi.h
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.4a: BME280 full temperature, pressure and
 *          humidity over SPI, with three read paths (blocking, normal mode,
 *          split phase). The driver reaches its finished form here.
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @sensor  Bosch BME280 (GY-BME280 module) in 4-wire SPI mode, SPI mode 0.
 *
 * @details This is the Chapter 5 (Exercise 5.3) BME280 driver grown up. In
 *          Chapter 5 the driver read temperature only, on purpose, to keep the
 *          SPI lesson focused. Chapter 6 finishes the driver: it now returns
 *          the sensor's complete measurement set - temperature, barometric
 *          pressure and relative humidity - in one forced-mode conversion and
 *          one contiguous burst read of the measurement registers (0xF7..0xFE).
 *
 *          The SPI mechanics are unchanged from Chapter 5; only the register
 *          coverage grows: the humidity control register (0xF2) is programmed,
 *          the full pressure and humidity factory calibration is read at init,
 *          and Bosch's published integer compensation is applied to each of the
 *          three quantities. The single-quantity Chapter 5 entry point is kept
 *          so earlier exercises still link, and a new bme280_read_all() returns
 *          everything at once for the Chapter 6 pipeline.
 *
 *          Three ways to get a reading, deliberately kept side by side because
 *          Section 6.4 compares them:
 *            1. bme280_read_all()      - blocking. One call, waits ~8 ms.
 *            2. normal mode            - sensor free-runs; reads never wait.
 *            3. split phase            - start / ready / fetch, so the wait
 *                                        belongs to the caller's loop.
 *          Path 1 is the simplest and is what Chapter 8 calls from an RTOS
 *          task. Path 2 is the cheapest fix for this particular sensor. Path 3
 *          is the one that generalizes to devices with no free-running mode.
 *
 *          Authored from the Bosch BME280 datasheet (BST-BME280-DS002): the
 *          register map, the measurement sequence, and the fixed-point
 *          compensation algorithms are the datasheet's; the driver structure,
 *          naming and packaging are the book's own.
 ******************************************************************************
 */

#ifndef BME280_SPI_H
#define BME280_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define BME280_CHIP_ID      (0x60U)    /* value expected in register 0xD0 */

/**
 * @brief One complete environmental record from a single conversion.
 *
 * The three quantities are kept in fixed-point integer units so the driver
 * pulls in no floating-point math:
 *   - temperature_c100 : hundredths of a degree Celsius (2534 = 25.34 C)
 *   - pressure_pa       : pascals               (101325 = 1013.25 hPa)
 *   - humidity_rh_1024  : %RH in Q22.10         (%RH = humidity_rh_1024 / 1024)
 */
typedef struct
{
    int32_t  temperature_c100;
    uint32_t pressure_pa;
    uint32_t humidity_rh_1024;
} bme280_reading_t;

/**
 * @brief  Probe the BME280, read the full factory calibration (temperature,
 *         pressure and humidity), program 1x oversampling on all three
 *         channels, and leave the sensor asleep ready for forced measurements.
 * @return 0 on success, -1 if the chip-ID register does not read 0x60.
 */
int bme280_init(void);

/** @brief Read the chip-ID register (0xD0); expected value is 0x60. */
uint8_t bme280_read_id(void);

/** Time one forced conversion actually needs, rounded up, in milliseconds.
 *
 *  This is the *floor* on how long a conversion takes, and the driver refuses
 *  to believe the sensor is finished before it has elapsed. That guard is not
 *  belt-and-braces, it is load-bearing.
 *
 *  The status register's `measuring` bit (0xF3, bit 3) does not go high the
 *  instant you write ctrl_meas. There is a short arming delay first. A poll
 *  issued immediately after the trigger therefore reads the bit as 0, and a
 *  naive "wait until measuring clears" loop exits on its very first pass,
 *  concluding the conversion is done before it has even started. The read that
 *  follows returns the *previous* conversion's data, and the whole thing looks
 *  like it works: the numbers are plausible, just one sample stale, and the
 *  wait that should have cost 8 ms costs nothing.
 *
 *  This was measured on hardware during drafting. The pipeline reported 1267 us
 *  per sample in blocking mode where the arithmetic says it should have been
 *  about 8900 us, and all three read paths landed within 320 us of each other,
 *  which is impossible if one of them is really waiting 8 ms.
 *
 *  8.0 ms typical / 9.3 ms maximum at x1/x1/x1 (see the timeout constant
 *  below for the derivation), so 10 ms clears the worst case.               */
#define BME280_CONVERSION_TIME_MS      (10U)

/** Upper bound on one forced conversion, in milliseconds.
 *
 *  Section 9.1 of BST-BME280-DS002 does not tabulate conversion times; it
 *  gives a formula, because the answer depends on the oversampling you chose:
 *
 *      t_typ = 1    + [2  *T_os] + [2  *P_os + 0.5  ] + [2  *H_os + 0.5  ]
 *      t_max = 1.25 + [2.3*T_os] + [2.3*P_os + 0.575] + [2.3*H_os + 0.575]
 *
 *  where each bracket is included only if that channel is enabled, and the
 *  value substituted is the oversampling multiplier (1, 2, 4, 8, 16), not the
 *  register code. This driver runs x1 on all three channels, so:
 *
 *      t_typ = 1    + 2   + 2.5   + 2.5   = 8.0 ms
 *      t_max = 1.25 + 2.3 + 2.875 + 2.875 = 9.3 ms
 *
 *  Do not confuse these with the 11.5 / 13.325 ms printed in the datasheet's
 *  worked example: that example uses temperature x1, pressure x4 and humidity
 *  disabled, which is a different configuration from this one.
 *
 *  20 ms leaves headroom over the 9.3 ms worst case without letting a dead
 *  sensor stall a caller for long. Raising oversampling raises the conversion
 *  time steeply (x2/x16/x1 reaches 46.1 ms maximum), so anyone changing
 *  ctrl_meas must revisit this constant.                                     */
#define BME280_CONVERSION_TIMEOUT_MS   (20U)

/** Standby-interval selectors for normal mode (config register, t_sb[7:5]). */
#define BME280_STANDBY_0M5      (0x0U)   /* 0.5 ms  */
#define BME280_STANDBY_62M5     (0x1U)   /* 62.5 ms */
#define BME280_STANDBY_125M     (0x2U)   /* 125 ms  */
#define BME280_STANDBY_250M     (0x3U)   /* 250 ms  */

/** IIR filter coefficients (config register, filter[4:2]). */
#define BME280_FILTER_OFF       (0x0U)
#define BME280_FILTER_2         (0x1U)
#define BME280_FILTER_4         (0x2U)

/**
 * @brief  Register the caller's millisecond time source.
 *
 *         Used only by the blocking read, so its wait is a real deadline
 *         rather than a spin count whose duration changes with the SPI
 *         prescaler. Optional: without it the blocking read falls back to a
 *         bounded poll count. Keeping the source injected means this driver
 *         does not depend on the chapter's scheduler module.
 */
void bme280_set_time_source(uint32_t (*now_ms)(void));

/* ---------------- Path 1: blocking, one call does everything -------------
 * The Chapter 5 shape. Simple to call and simple to read, but it does not
 * return until the sensor is done, so it must not be called from a
 * cooperative super-loop. Chapter 8 calls it from an RTOS task, where only
 * that task blocks.
 * ------------------------------------------------------------------------ */

/**
 * @brief  Trigger one forced-mode conversion, wait for it, and return all
 *         three compensated values through @p out.
 * @param  out  Destination record. Must not be NULL.
 * @return 0 on success, -1 on a NULL argument or conversion timeout.
 * @warning Blocks for roughly 8 ms. Do not call from the super-loop.
 */
int bme280_read_all(bme280_reading_t *out);

/**
 * @brief  Convenience wrapper kept for Chapter 5 compatibility: trigger one
 *         conversion and return only the compensated temperature in hundredths
 *         of a degree Celsius (e.g. 2534 = 25.34 C). Blocks, as above.
 */
int32_t bme280_read_temperature_c100(void);

/* ---------------- Path 2: normal mode, the cheap fix ---------------------
 * Reconfigure the sensor so it measures continuously on its own schedule.
 * Reads then never wait, because there is nothing to wait for. Two register
 * writes replace an entire state machine. What it costs: the sample may be
 * up to one standby interval old and the caller cannot tell how old.
 * ------------------------------------------------------------------------ */

/**
 * @brief  Put the sensor into normal (free-running) mode.
 * @param  standby_sel  One of the BME280_STANDBY_* selectors.
 * @param  filter_sel   One of the BME280_FILTER_* coefficients.
 * @return 0 on success.
 */
int bme280_set_mode_normal(uint8_t standby_sel, uint8_t filter_sel);

/**
 * @brief  Read whatever the free-running sensor last produced. Never waits.
 * @return 0 on success, -1 on a NULL argument.
 */
int bme280_read_latest(bme280_reading_t *out);

/** @brief Return the sensor to sleep mode (ctrl_meas mode bits cleared). */
int bme280_set_mode_sleep(void);

/* ---------------- Path 3: split phase, the general fix -------------------
 * Forced mode, but the wait belongs to the caller. Start the conversion,
 * return to the loop, poll cheaply on later iterations, then fetch. This is
 * the pattern to reach for whenever a device cannot be told to free-run: a
 * flash page erase, an ADC conversion, a DMA completion.
 * ------------------------------------------------------------------------ */

/** @brief Kick off one forced-mode conversion and return immediately. */
int bme280_start_measurement(void);

/**
 * @brief  1 if the conversion has finished, 0 if it is still running.
 *
 *         Two conditions, both required. At least BME280_CONVERSION_TIME_MS
 *         must have elapsed since bme280_start_measurement(), *and* the status
 *         register's measuring bit must read clear. The elapsed-time half is
 *         what stops a poll issued too early from mistaking "not started yet"
 *         for "already finished"; see the notes on BME280_CONVERSION_TIME_MS.
 *
 * @note   Without a registered time source the elapsed-time half cannot be
 *         evaluated and only the status bit is checked, which reintroduces the
 *         race. Call bme280_set_time_source() first.
 */
int bme280_measurement_ready(void);

/**
 * @brief  Burst-read and compensate the completed measurement.
 * @return 0 on success, -1 on a NULL argument.
 * @note   Only meaningful once bme280_measurement_ready() has returned 1.
 */
int bme280_fetch(bme280_reading_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BME280_SPI_H */
