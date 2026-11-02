/**
 ******************************************************************************
 * @file    ds3231.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.6: DS3231 real-time clock from its datasheet
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @sensor  Analog Devices DS3231 I2C RTC/TCXO, fixed 7-bit address 0x68.
 *
 * @details This exercise is the "interface an unfamiliar peripheral using only
 *          its datasheet" drill. The DS3231 is deliberately *unlike* the BME280
 *          and ADXL345 used earlier, in ways the reader must discover from the
 *          datasheet rather than from this chapter's prior examples:
 *
 *            - There is NO chip-ID register. Presence is confirmed by the slave
 *              ACKing its address, not by reading a magic byte.
 *            - Time fields are stored in BCD (binary-coded decimal), not plain
 *              binary, so every field needs conversion both ways.
 *            - Temperature lives in two registers (0x11 integer, 0x12 holds the
 *              0.25 C fraction in bits 7:6).
 *
 *          The register map and BCD encoding are taken from the DS3231 datasheet.
 ******************************************************************************
 */

#ifndef DS3231_H
#define DS3231_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DS3231_ADDR     (0x68U)    /* fixed 7-bit address */

/* 24-hour time/date. 'year' is the full year (e.g. 2026). */
typedef struct
{
    uint8_t  sec;     /* 0-59  */
    uint8_t  min;     /* 0-59  */
    uint8_t  hour;    /* 0-23  */
    uint8_t  dow;     /* 1-7, day of week (user-defined meaning) */
    uint8_t  date;    /* 1-31  */
    uint8_t  month;   /* 1-12  */
    uint16_t year;    /* full year, 2000-2099 */
} ds3231_time_t;

/**
 * @brief  Initialise the bus side and confirm the DS3231 ACKs its address.
 * @return 0 if the device acknowledges, -1 otherwise.
 */
int ds3231_init(void);

/** @brief Write all seven time/date registers (binary -> BCD internally). */
int ds3231_set_time(const ds3231_time_t *t);

/** @brief Read all seven time/date registers (BCD -> binary internally). */
int ds3231_get_time(ds3231_time_t *t);

/**
 * @brief  Read the on-chip temperature sensor.
 * @return Temperature in hundredths of a degree Celsius (resolution 0.25 C,
 *         so values step in multiples of 25).
 */
int32_t ds3231_read_temp_c100(void);

#ifdef __cplusplus
}
#endif

#endif /* DS3231_H */
