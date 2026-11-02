/**
 ******************************************************************************
 * @file    bme280_spi.h
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.3: BME280 temperature over SPI
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @sensor  Bosch BME280 (GY-BME280 module) in 4-wire SPI mode, SPI mode 0.
 *
 * @details A deliberately small sensor layer that sits on top of spi_driver.c.
 *          It reads the chip-ID register to confirm the part is present, then
 *          performs a forced-mode temperature measurement and applies Bosch's
 *          published integer temperature-compensation formula. Pressure and
 *          humidity are intentionally left out to keep the SPI lesson focused;
 *          the same register-read pattern extends to them.
 *
 *          Authored from the Bosch BME280 datasheet (register map and the
 *          temperature compensation algorithm), not from any third-party code.
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
 * @brief  Probe the BME280, read its factory temperature calibration, and
 *         leave it in sleep mode ready for forced measurements.
 * @return 0 on success, -1 if the chip-ID register does not read 0x60.
 */
int bme280_init(void);

/** @brief Read the chip-ID register (0xD0); expected value is 0x60. */
uint8_t bme280_read_id(void);

/**
 * @brief  Trigger one forced-mode conversion and return the compensated
 *         temperature in hundredths of a degree Celsius (e.g. 2534 = 25.34 C).
 */
int32_t bme280_read_temperature_c100(void);

#ifdef __cplusplus
}
#endif

#endif /* BME280_SPI_H */
