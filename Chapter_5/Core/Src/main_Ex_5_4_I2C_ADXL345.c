/**
 ******************************************************************************
 * @file    main_Ex_5_4_I2C_ADXL345.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.4: Read a 3-axis accelerometer over I2C
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @board   I2C1: PB8 SCL, PB9 SDA (4.7k pull-ups to 3V3)  ->  ADXL345 @ 0x53
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Brings up I2C1 at 100 kHz, confirms the ADXL345 device ID (0xE5),
 *          configures +/-4 g full-resolution at 100 Hz, then loops reading and
 *          printing acceleration on X/Y/Z in milli-g. Tilt the board and the
 *          numbers change immediately — one axis reads ~1000 mg (1 g) when it
 *          points down, which makes "does my driver work?" obvious.
 *
 *          On the logic analyzer (CH0=SCL, CH1=SDA) the I2C decoder shows
 *          the register-pointer write, a repeated START, and the six
 *          auto-incremented data bytes of a burst read.
 *
 *          Note: for higher output rates, switch I2C1 to Fast Mode (400 kHz);
 *          at 100 kHz a six-byte burst sustains a few hundred Hz, which is
 *          plenty for this exercise.
 ******************************************************************************
 */

#include "main.h"
#include "i2c_driver.h"
#include "adxl345_i2c.h"
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
    printf("\r\nChapter 5 - Exercise 5.4: ADXL345 3-axis accelerometer over I2C1\r\n");

    i2c1_init();

    const uint8_t id = adxl345_read_id();
    printf("ADXL345 device ID = 0x%02X (expected 0x%02X)\r\n",
           (unsigned)id, (unsigned)ADXL_DEVICE_ID);

    const adxl_config_t cfg = {
        .range    = ADXL_RANGE_4G,
        .odr      = ADXL_ODR_100HZ,
        .full_res = 1U
    };

    const adxl_status_t st = adxl345_init(&cfg);
    if (st == ADXL_ERR_ID)
    {
        printf("ERROR: wrong/absent device ID. Check address 0x53 (SDO low) and 3V3.\r\n");
        Error_Handler();
    }
    else if (st != ADXL_OK)
    {
        printf("ERROR: I2C bus error during init. Check pull-ups and wiring.\r\n");
        Error_Handler();
    }

    while (1)
    {
        int32_t x_mg = 0, y_mg = 0, z_mg = 0;
        if (adxl345_read_mg(&x_mg, &y_mg, &z_mg) == ADXL_OK)
        {
            printf("X=%6ld mg  Y=%6ld mg  Z=%6ld mg\r\n",
                   (long)x_mg, (long)y_mg, (long)z_mg);
        }
        else
        {
            printf("read error\r\n");
        }
        crude_delay(4000000UL);
    }
}

/* The bus scanner that Section 5.4 recommends is a separate build:
 * main_Ex_5_4_I2C_BusScan.c in this folder. Exclude one main from the build
 * and include the other, the same way the two logic-analyzer builds work in
 * Exercise 5.2. */

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
