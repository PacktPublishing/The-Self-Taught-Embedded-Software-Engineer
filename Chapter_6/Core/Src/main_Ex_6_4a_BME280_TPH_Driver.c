/**
 ******************************************************************************
 * @file    main_Ex_6_4a_BME280_TPH_Driver.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.4a: print a full environmental record
 *          (temperature, pressure, humidity) from the finished BME280 driver
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   SPI2: PB13 SCK, PB15 MOSI, PB14 MISO, PB12 CS  ->  BME280 (mode 0)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details A focused check that the Chapter 6 upgrade to the BME280 driver
 *          works before it is dropped into the Section 6.4 pipeline. It brings
 *          SPI2 up at ~1 MHz (clean on the logic analyzer), confirms the chip
 *          ID, then loops reading all three quantities in one forced-mode
 *          conversion and printing them in human units. Breathe on the sensor
 *          and humidity climbs; the pressure reading should sit near your local
 *          sea-level-adjusted value.
 *
 *          Output goes over SWV/ITM (not UART) so this driver check needs no
 *          serial terminal - open the SWV ITM Data Console in the debugger.
 *
 *          The values are integer fixed-point out of the driver, so the print
 *          code splits each into whole and fractional parts by hand rather than
 *          pulling in floating-point formatting.
 ******************************************************************************
 */

#include "main.h"
#include "spi_driver.h"
#include "bme280_spi.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* A rough busy-wait used only to space out the prints so the console is
 * readable. `volatile` on the counter stops the compiler optimising the empty
 * loop away. This is not a calibrated delay - the pipeline exercise uses the
 * TIM3 tick for real timing; here we just want a visible pause.              */
static void crude_delay(volatile uint32_t loops)
{
    while (loops > 0UL)
    {
        loops--;
    }
}

int main(void)
{
    /* Unbuffered stdout so each printf reaches the ITM console immediately. */
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 6 - Exercise 6.4a: BME280 temperature, pressure, humidity over SPI2\r\n");

    /* SPI mode 0, br=4 -> 16 MHz / 32 = 500 kHz (<=1 MHz keeps captures clean). */
    const spi_config_t spi_cfg = { .mode = SPI_MODE0, .br = 4U };
    if (spi2_init(&spi_cfg) != SPI_OK)
    {
        Error_Handler();   /* bad prescaler: a programming error, not a wiring one */
    }

    /* Read the chip-ID register first: a quick, non-destructive presence test
     * that also proves the SPI wiring and mode before we trust any data.      */
    const uint8_t id = bme280_read_id();
    printf("BME280 chip ID = 0x%02X (expected 0x%02X)\r\n",
           (unsigned)id, (unsigned)BME280_CHIP_ID);

    /* Full init: verifies the ID, reads all factory calibration (T, P and H),
     * and programs humidity oversampling. A non-zero return is almost always
     * wiring, chip-select, or power.                                          */
    if (bme280_init() != 0)
    {
        printf("ERROR: BME280 not found. Check SPI2 wiring, CS on PB12, and 3V3.\r\n");
        Error_Handler();
    }

    while (1)
    {
        bme280_reading_t r;

        /* One forced-mode conversion returns all three quantities together. */
        if (bme280_read_all(&r) == 0)
        {
            /* Convert the driver's fixed-point integers to whole + fractional
             * parts for a float-free printf:
             *   temperature_c100 : 0.01 C  -> /100 whole C, %100 hundredths
             *   pressure_pa      : Pa      -> /100 whole hPa, %100 hundredths
             *   humidity_rh_1024 : Q22.10  -> /1024 whole %RH, remainder scaled */
            const int32_t  t_whole = r.temperature_c100 / 100;
            const int32_t  t_frac  = (r.temperature_c100 < 0 ?      /* |value| so a */
                                      -r.temperature_c100 : r.temperature_c100) % 100; /* sub-zero temp still prints 2 digits */
            const uint32_t p_whole = r.pressure_pa / 100U;      /* hPa */
            const uint32_t p_frac  = r.pressure_pa % 100U;      /* hundredths of hPa */
            const uint32_t h_whole = r.humidity_rh_1024 / 1024U;
            const uint32_t h_frac  = ((r.humidity_rh_1024 % 1024U) * 100U) / 1024U;

            printf("T=%ld.%02ld C   P=%lu.%02lu hPa   RH=%lu.%02lu %%\r\n",
                   (long)t_whole, (long)t_frac,
                   (unsigned long)p_whole, (unsigned long)p_frac,
                   (unsigned long)h_whole, (unsigned long)h_frac);
        }
        else
        {
            printf("read error\r\n");   /* conversion timed out */
        }
        crude_delay(4000000UL);         /* readable spacing between samples */
    }
}

/* Trap for unrecoverable errors: mask interrupts and spin so a debugger can
 * inspect the call stack. */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
