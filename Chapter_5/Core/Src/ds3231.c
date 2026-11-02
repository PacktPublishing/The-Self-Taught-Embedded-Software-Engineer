/**
 ******************************************************************************
 * @file    ds3231.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.6: DS3231 RTC (implementation, from datasheet)
 *
 * @details Register map used (DS3231 datasheet):
 *            0x00 seconds (BCD)      
 *            0x01 minutes (BCD)
 *            0x02 hours (bit 6 = 12/24; we use 24h, BCD in bits 5:0)
 *            0x03 day-of-week (1-7)  
 *            0x04 date (BCD)
 *            0x05 month (bit 7 = century, BCD in bits 4:0)
 *            0x06 year (BCD, 00-99)
 *            0x11 temp MSB (signed integer C)
 *            0x12 temp LSB (bits 7:6 = quarter-degree fraction)
 *
 *          Reuses the book's i2c_driver.c. Because the DS3231 has no ID
 *          register, ds3231_init() proves presence by issuing a zero-length-ish
 *          read of the seconds register and checking for an ACK (no NACK/timeout).
 ******************************************************************************
 */

#include "ds3231.h"
#include "i2c_driver.h"

#define REG_SECONDS     (0x00U)
#define REG_TEMP_MSB    (0x11U)

static uint8_t bcd2bin(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10U) + (bcd & 0x0FU));
}
static uint8_t bin2bcd(uint8_t bin)
{
    return (uint8_t)(((bin / 10U) << 4) | (bin % 10U));
}

int ds3231_init(void)
{
    /* The bus is NOT brought up here. i2c1_init() belongs to the caller, the
     * same way Exercise 5.4 does it, because I2C is a shared bus: the DS3231
     * and the ADXL345 sit on the same two wires, and a device driver that
     * silently re-initialises the bus would reset it underneath whatever else
     * is using it. A device layer configures its device; a bus layer
     * configures its bus.                                                    */

    /* No chip-ID register exists: confirm presence by reading one byte and
     * checking the transaction was acknowledged.
     *
     * The very first I2C transaction after peripheral enable can be unreliable
     * while the bus is still settling, so retry a few times before declaring
     * the device absent instead of failing on a single flaky START.           */
    for (int attempt = 0; attempt < 3; attempt++)
    {
        uint8_t s = 0U;
        if (i2c1_mem_read(DS3231_ADDR, REG_SECONDS, &s, 1U) == I2C_OK)
        {
            return 0;
        }
        for (volatile uint32_t d = 0U; d < 100000U; d++) { }  /* brief pause */
    }
    return -1;
}

int ds3231_set_time(const ds3231_time_t *t)
{
    if (t == 0)
    {
        return -1;
    }

    /* One transaction: register pointer 0x00 followed by 7 BCD bytes. */
    const uint8_t buf[8] =
    {
        REG_SECONDS,
        bin2bcd(t->sec),
        bin2bcd(t->min),
        bin2bcd(t->hour),                 /* bit6=0 -> 24-hour mode */
        (uint8_t)(t->dow & 0x07U),
        bin2bcd(t->date),
        bin2bcd(t->month),                /* century bit left 0 */
        bin2bcd((uint8_t)(t->year % 100U))
    };

    return (i2c1_write(DS3231_ADDR, buf, 8U, 1) == I2C_OK) ? 0 : -1;
}

int ds3231_get_time(ds3231_time_t *t)
{
    if (t == 0)
    {
        return -1;
    }

    uint8_t r[7];
    if (i2c1_mem_read(DS3231_ADDR, REG_SECONDS, r, 7U) != I2C_OK)
    {
        return -1;
    }

    t->sec   = bcd2bin(r[0] & 0x7FU);
    t->min   = bcd2bin(r[1] & 0x7FU);
    t->hour  = bcd2bin(r[2] & 0x3FU);     /* mask off the 12/24 control bit */
    t->dow   = (uint8_t)(r[3] & 0x07U);
    t->date  = bcd2bin(r[4] & 0x3FU);
    t->month = bcd2bin(r[5] & 0x1FU);     /* mask off the century bit */
    t->year  = (uint16_t)(2000U + bcd2bin(r[6]));
    return 0;
}

int32_t ds3231_read_temp_c100(void)
{
    uint8_t t[2] = { 0U, 0U };
    (void)i2c1_mem_read(DS3231_ADDR, REG_TEMP_MSB, t, 2U);

    const int32_t integer  = (int8_t)t[0];          /* signed whole degrees   */
    const int32_t quarters = (t[1] >> 6) & 0x03U;   /* 0..3 -> 0.00..0.75 C   */
    return (integer * 100) + (quarters * 25);
}
