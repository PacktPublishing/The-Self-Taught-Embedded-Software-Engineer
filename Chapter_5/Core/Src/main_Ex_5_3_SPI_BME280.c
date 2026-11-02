/**
 ******************************************************************************
 * @file    main_Ex_5_3_SPI_BME280.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.3: Talk to an SPI sensor (BME280)
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   SPI2: PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS  ->  GY-BME280
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Demonstrates the SPI driver against a real device. The program:
 *            1. brings SPI2 up in mode 0 at ~500 kHz,
 *            2. reads the BME280 chip-ID register (expect 0x60) to prove the
 *               link works, and
 *            3. loops reading a forced-mode temperature and printing it.
 *
 *          Probe SCK/MOSI/MISO/CS on the logic analyzer and decode with the SPI
 *          decoder (mode 0, MSB first) to see the control byte (reg|0x80 for a
 *          read) followed by the returned data — exactly what the datasheet's
 *          timing diagram shows.
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
    printf("\r\nChapter 5 - Exercise 5.3: BME280 over SPI2\r\n");

    const spi_config_t spi_cfg = { .mode = SPI_MODE0, .br = 4U };   /* ~500 kHz */
    if (spi2_init(&spi_cfg) != SPI_OK)
    {
        printf("ERROR: bad SPI configuration.\r\n");
        Error_Handler();
    }

    const uint8_t id = bme280_read_id();
    printf("BME280 chip ID = 0x%02X (expected 0x%02X)\r\n",
           (unsigned)id, (unsigned)BME280_CHIP_ID);

    if (bme280_init() != 0)
    {
        printf("ERROR: BME280 not found. Check 3V3, CS=PB12, and SPI mode.\r\n");
        Error_Handler();
    }

    while (1)
    {
        const int32_t t = bme280_read_temperature_c100();
        printf("Temperature: %ld.%02ld C\r\n",
               (long)(t / 100), (long)(t < 0 ? -(t % 100) : (t % 100)));
        crude_delay(4000000UL);   /* ~human-readable cadence */
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
