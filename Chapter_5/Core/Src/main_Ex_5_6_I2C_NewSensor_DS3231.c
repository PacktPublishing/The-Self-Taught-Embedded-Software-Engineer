/**
 ******************************************************************************
 * @file    main_Ex_5_6_I2C_NewSensor_DS3231.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.6: Interface a new sensor from its datasheet
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   I2C1: PB8 SCL, PB9 SDA (4.7k pull-ups to 3V3)  ->  DS3231 @ 0x68
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details The capstone of the chapter's "read the datasheet" skill. Using only
 *          the DS3231 datasheet (no prior example for this part), the program:
 *            1. confirms the device ACKs at 0x68 (there is no ID register),
 *            2. sets the time once, and
 *            3. loops reading the time and the on-chip temperature.
 *
 *          The same i2c_driver from Exercise 5.4 is reused unchanged — the
 *          lesson is that a solid bus driver plus a datasheet is all you need
 *          to bring up a peripheral you have never seen before.
 *
 *          Set BUILD_SET_TIME to 1 for the first run to seed the clock, then
 *          rebuild with 0 so the running time is preserved across resets.
 ******************************************************************************
 */

#include "main.h"
#include "i2c_driver.h"
#include "ds3231.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define BUILD_SET_TIME   (1)   /* 1: seed the clock on this build; 0: just read */

static void crude_delay(volatile uint32_t loops)
{
    while (loops > 0UL)
    {
        loops--;
    }
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 5 - Exercise 5.6: DS3231 RTC from datasheet (I2C1)\r\n");

    /* Bring the shared bus up first, then the device on it. Same order as
     * Exercise 5.4: the bus belongs to the application, not to any one part. */
    i2c1_init();

    if (ds3231_init() != 0)
    {
        printf("ERROR: DS3231 did not ACK at 0x68. Check wiring/pull-ups/3V3.\r\n");
        Error_Handler();
    }
    printf("DS3231 present (ACK at 0x68; note: no chip-ID register exists).\r\n");

#if (BUILD_SET_TIME == 1)
    const ds3231_time_t seed =
    {
        .sec = 0U, .min = 30U, .hour = 9U,
        .dow = 1U, .date = 29U, .month = 6U, .year = 2026U
    };
    if (ds3231_set_time(&seed) == 0)
    {
        printf("Time seeded to 2026-06-29 09:30:00.\r\n");
    }
#endif

    while (1)
    {
        ds3231_time_t now;
        if (ds3231_get_time(&now) == 0)
        {
            const int32_t tc = ds3231_read_temp_c100();

            /* The DS3231 reads below freezing, and C truncates division toward
             * zero, so a naive tc/100 and tc%100 would print -5.25 C as
             * "-5.-25". Carry the sign once, in front, and take the fraction
             * from the magnitude. This is the kind of detail a datasheet-first
             * bring-up teaches you to look for: the register is documented as
             * signed, so the print path has to be too.                       */
            const char   sign = (tc < 0) ? '-' : '+';
            const int32_t mag = (tc < 0) ? -tc : tc;

            printf("%04u-%02u-%02u  %02u:%02u:%02u   T=%c%ld.%02ld C\r\n",
                   (unsigned)now.year, (unsigned)now.month, (unsigned)now.date,
                   (unsigned)now.hour, (unsigned)now.min, (unsigned)now.sec,
                   sign, (long)(mag / 100), (long)(mag % 100));
        }
        else
        {
            printf("read error\r\n");
        }
        crude_delay(4000000UL);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
