/**
 ******************************************************************************
 * @file    main_Ex_6_4_Blocking_Pipeline_VCP.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.4: the complete sensor-to-action pipeline,
 *          running the BME280 (SPI2) and ADXL345 (I2C1) concurrently in one
 *          cooperative super-loop [VCP / USART2 build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   SPI2 PB13/PB15/PB14 + CS PB12 -> BME280 |  I2C1 PB8/PB9 -> ADXL345 @0x53
 *          LD2 PA5 = actuator | PA6 = latency marker | B1 PC13 = threshold up
 *          USART2 PA2/PA3 -> ST-Link VCP
 * @output  A serial terminal on the ST-Link Virtual COM Port, 115200 8N1
 *
 * @details This is the chapter's payoff: every driver built so far, working
 *          together. One super-loop iteration services two independent buses -
 *          it samples the SPI environment sensor and the I2C accelerometer,
 *          filters a derived motion figure, compares it against a button-set
 *          threshold, drives LD2 as the actuator, and logs the result over
 *          UART. The pipeline reports its own end-to-end latency in
 *          microseconds, and mirrors it as a pulse on PA6 for the logic
 *          analyzer.
 *
 *          Running both buses in one loop is also the setup for Section 6.5:
 *          you can now feel how much of each iteration is spent waiting
 *          on the two sensors, which is exactly the cost that DMA (Chapter 7)
 *          and an RTOS (Chapter 8) go on to remove.
 *
 *          Scheduling (cooperative, 1 ms tick):
 *            - every  50 ms : run the pipeline; log immediately on any LD2 flip
 *            - every 1000 ms : print a full status line (both sensors + timing)
 *            - on B1 tap     : raise the motion threshold to the next preset
 *            - on B1 hold    : cycle blocking -> normal-mode -> split-phase
 ******************************************************************************
 */

#include "main.h"
#include "scheduler.h"
#include "exti_driver.h"
#include "spi_driver.h"
#include "bme280_spi.h"
#include "i2c_driver.h"
#include "adxl345_i2c.h"
#include "uart_driver_isr.h"
#include "pipeline.h"
#include "button_fsm.h"
#include "gpio_driver.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define BTN_PORT        GPIOC
#define BTN_PIN         (13U)   /* B1, active low; exti_pc13_init() pulls it up */

/* Raised by the B1 EXTI interrupt, cleared by the main loop. `volatile`
 * because it is written in ISR context and read in thread context.          */
static volatile uint8_t s_button_flag = 0U;

/* B1 interrupt callback. Kept to a single store so the ISR stays trivial;
 * the actual work (bump the threshold, log) happens later in the main loop. */
static void on_button_press(void)
{
    s_button_flag = 1U;               /* keep the ISR to a single store */
}

/* Absolute value for the fixed-point split below (freestanding). */
static int32_t abs_c100(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Build and send one full status line over UART.
 *
 * The driver returns fixed-point integers (temperature in 0.01 C, pressure in
 * Pa, humidity in Q22.10), so this splits each into whole and fractional parts
 * by hand and prints with %ld/%lu. That keeps the build off the floating-point
 * printf, which is both smaller and safe on the soft-float configuration.
 *   - temperature: value/100 gives whole C, |value|%100 gives hundredths.
 *   - pressure:    Pa/100 gives whole hPa, Pa%100 gives hundredths of hPa.
 *   - humidity:    value/1024 gives whole %RH, remainder scaled to hundredths.
 */
static void log_status(const pipeline_t *p)
{
    const int32_t  t_w = p->env.temperature_c100 / 100;
    const int32_t  t_f = abs_c100(p->env.temperature_c100) % 100;   /* |..| so a negative temp still prints two digits */
    const uint32_t p_w = p->env.pressure_pa / 100U;       /* hPa */
    const uint32_t p_f = p->env.pressure_pa % 100U;
    const uint32_t h_w = p->env.humidity_rh_1024 / 1024U;
    const uint32_t h_f = ((p->env.humidity_rh_1024 % 1024U) * 100U) / 1024U;

    char line[224];
    (void)snprintf(line, sizeof(line),
        "[%lu ms] T=%ld.%02ld C P=%lu.%02lu hPa RH=%lu.%02lu%% | "
        "motion=%ld thr=%ld LED=%s lat=%luus | mode=%s rxdrop=%lu\r\n",
        (unsigned long)millis(),                              /* uptime      */
        (long)t_w, (long)t_f,                                 /* temperature */
        (unsigned long)p_w, (unsigned long)p_f,               /* pressure    */
        (unsigned long)h_w, (unsigned long)h_f,               /* humidity    */
        (long)p->motion_filt_mg, (long)p->threshold_mg,       /* motion/thr  */
        (p->actuator_on != 0U) ? "ON" : "off",                /* actuator    */
        (unsigned long)p->last_latency_us,                    /* latency     */
        pipeline_mode_name(p->mode),                          /* sample path */
        (unsigned long)uart2_rx_overruns());                  /* bytes lost  */
    uart2_write_string(line);
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    /* --- bring up both buses and their sensors -------------------------
     * Order matters only in that the bus (SPI2 / I2C1) must be up before the
     * sensor that rides on it. UART is brought up early so the error paths
     * below can report a failed sensor bring-up. */
    const spi_config_t spi_cfg = { .mode = SPI_MODE0, .br = 4U };  /* ~500 kHz, clean on the analyzer */
    if (spi2_init(&spi_cfg) != SPI_OK)
    {
        Error_Handler();   /* bad prescaler: a programming error, not a wiring one */
    }

    i2c1_init();

    const uart_config_t uart = { .baud = 115200U };
    uart2_init(&uart);

    uart2_write_string("\r\nChapter 6 - Exercise 6.4: sensor-to-action pipeline (VCP)\r\n");

    /* BME280 on SPI2: confirms the chip ID and reads its factory calibration.
     * A failure here is almost always CS/wiring or power, so we say so. */
    if (bme280_init() != 0)
    {
        uart2_write_string("ERROR: BME280 not found on SPI2 (check CS PB12, 3V3).\r\n");
        Error_Handler();
    }

    /* ADXL345 on I2C1: +/-4 g, 100 Hz, full-resolution. A failure here is
     * usually the address (SDO wiring -> 0x53) or missing bus pull-ups. */
    const adxl_config_t acfg = {
        .range = ADXL_RANGE_4G, .odr = ADXL_ODR_100HZ, .full_res = 1U
    };
    if (adxl345_init(&acfg) != ADXL_OK)
    {
        uart2_write_string("ERROR: ADXL345 not responding on I2C1 (check 0x53, pull-ups).\r\n");
        Error_Handler();
    }

    /* --- time base, button, and the pipeline itself --------------------
     * The 1 ms tick (TIM3) drives sched_due(); B1 only sets a flag; the
     * pipeline owns LD2/PA6 and the DWT timing. */
    scheduler_init();                     /* 1 ms cooperative tick */
    exti_pc13_init(on_button_press);      /* B1 -> flag only */

    pipeline_t pipe;
    pipeline_init(&pipe);

    /* The sensor needs a real millisecond deadline for its bounded
     * blocking wait; hand it the super-loop's own time base. */
    bme280_set_time_source(millis);

    button_fsm_t button;
    button_fsm_init(&button);

    uart2_write_string("Tilt or shake the ADXL345 to cross the threshold. Tap B1 to raise it; hold B1 to change sampling mode.\r\n");

    /* Per-subsystem "last ran" timestamps for the cooperative scheduler. */
    uint32_t last_sample = 0U;
    uint32_t last_status = 0U;

    /* --- the cooperative super-loop -----------------------------------
     * Three independent, non-blocking jobs share the loop. None of them waits;
     * each runs when its own timer is due (or, for the button, when the flag
     * is set) and then returns, so all three stay responsive together. */
    while (1)
    {
        /* Job 1: run the pipeline at 20 Hz. Log immediately on any LD2 flip so
         * a threshold crossing is reported the instant it happens. */
        if (sched_due(&last_sample, 50U))
        {
            pipeline_step(&pipe);
            if (pipe.state_changed != 0U)
            {
                log_status(&pipe);
            }
        }

        /* Job 2: the button, reusing the five-state machine built in Exercise
         * 6.2. A tap raises the motion threshold; a hold cycles the sampling
         * strategy, so one flashed build can produce all three PA6 latency
         * captures without re-flashing between them. */
        uint8_t edge = 0U;
        if (s_button_flag != 0U)
        {
            s_button_flag = 0U;       /* consume the edge the ISR recorded */
            edge = 1U;
        }

        /* Poll the pin only once the machine has something to track. While it
         * sits in IDLE the EXTI edge is what wakes it, so an untouched button
         * costs the loop no GPIO reads at all.                              */
        uint8_t is_down = 0U;
        if ((button.state != BTN_IDLE) || (edge != 0U))
        {
            is_down = (gpio_read_pin(BTN_PORT, BTN_PIN) == GP_LOW) ? 1U : 0U;
        }
        const btn_event_t ev = button_fsm_update(&button, is_down, millis());

        if (ev == BTN_EVENT_SHORT)
        {
            pipeline_bump_threshold(&pipe);
            char msg[48];
            (void)snprintf(msg, sizeof(msg),
                           "threshold -> %ld mg\r\n", (long)pipe.threshold_mg);
            uart2_write_string(msg);
        }
        else if (ev == BTN_EVENT_LONG)
        {
            /* blocking -> normal-mode -> split-phase -> blocking ... */
            const pipe_sample_mode_t next =
                (pipe.mode == PIPE_SAMPLE_BLOCKING) ? PIPE_SAMPLE_NORMAL :
                (pipe.mode == PIPE_SAMPLE_NORMAL)   ? PIPE_SAMPLE_SPLIT  :
                                                      PIPE_SAMPLE_BLOCKING;
            pipeline_set_mode(&pipe, next);
            char msg[72];
            (void)snprintf(msg, sizeof(msg),
                           "sample mode -> %s (watch PA6 and rxdrop)\r\n",
                           pipeline_mode_name(pipe.mode));
            uart2_write_string(msg);
        }
        else
        {
            /* no button event this pass */
        }

        /* Job 3: once a second, print a full status line even if nothing
         * changed, so the terminal always shows a live heartbeat of readings. */
        if (sched_due(&last_status, 1000U))
        {
            log_status(&pipe);
        }
    }
}

/* Trap for unrecoverable errors (e.g. a sensor that never answered): mask
 * interrupts and spin so a debugger can inspect the call stack. */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
