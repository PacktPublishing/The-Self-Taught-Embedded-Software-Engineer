/**
 ******************************************************************************
 * @file    main_Ex_5_4_I2C_BusScan.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.4 (scanner build): who is actually on the bus
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   I2C1 on the ST morpho header: PB8 = SCL, PB9 = SDA, 4.7k pull-ups
 * @output  SWV ITM Data Console, Stimulus Port 0
 *
 * @details The first tool to reach for when an I2C device does not answer.
 *          It walks every 7-bit address the standard allows a device to use,
 *          attempts a one-byte write to each, and prints the addresses that
 *          acknowledged. Nothing else in this chapter answers the first
 *          question of any bring-up so quickly: is the part there at all, and
 *          is it at the address you think it is?
 *
 *          Reading the result:
 *            - a device at an address you did not expect usually means an
 *              address-select pin is strapped differently than you assumed;
 *            - a device that appears nowhere is a wiring, pull-up, or power
 *              problem, not a code problem;
 *            - MORE devices than you wired is the interesting case. Sensor
 *              breakouts frequently carry a second chip. The DS3231 module
 *              used in Section 5.6 is the standard example: the RTC answers
 *              at 0x68, and the EEPROM sitting beside it on the same board
 *              answers separately.
 *
 *          The scan range is 0x08 to 0x77. Addresses 0x00-0x07 and 0x78-0x7F
 *          are reserved by the I2C specification for general call, 10-bit
 *          addressing, and other special cases, so no ordinary device lives
 *          there and probing them can confuse a bus.
 *
 * @note    Build ONE main at a time. Keep either this file or
 *          main_Ex_5_4_I2C_ADXL345.c in the build and exclude the other
 *          (right-click -> Properties -> C/C++ Build -> Exclude from Build),
 *          since both define main().
 ******************************************************************************
 */

#include "main.h"
#include "i2c_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* First and last address the I2C specification leaves free for devices. */
#define SCAN_FIRST_ADDR   (0x08U)
#define SCAN_LAST_ADDR    (0x77U)

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\nChapter 5 - Exercise 5.4 (scanner): I2C bus scan on I2C1\r\n");

    i2c1_init();

    printf("I2C scan:\r\n");

    uint32_t found = 0U;
    for (uint8_t addr = SCAN_FIRST_ADDR; addr <= SCAN_LAST_ADDR; addr++)
    {
        /* One byte is enough. If a device is listening at this address it
         * acknowledges its address byte, and the driver returns I2C_OK; if
         * nothing is there the hardware raises AF and we get I2C_ERR_NACK.
         * Register 0x00 is a harmless target: it is read-only on every part
         * used in this chapter, so a stray write changes nothing.           */
        const uint8_t reg0 = 0x00U;

        if (i2c1_write(addr, &reg0, 1U, 1) == I2C_OK)
        {
            printf("  device found at 0x%02X\r\n", (unsigned)addr);
            found++;
        }
    }

    printf("scan done: %lu device(s)\r\n", (unsigned long)found);

    while (1)
    {
        /* trap so the SWV log stays on screen */
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
