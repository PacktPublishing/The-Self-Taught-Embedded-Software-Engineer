/**
 ******************************************************************************
 * @file    pipeline.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.4: sensor-to-action pipeline (implementation)
 *
 * @details The five stages are deliberately kept as separate static functions
 *          so the data flow reads top to bottom:
 *
 *              sample --> filter --> threshold --> action --> (log, by caller)
 *
 *          All arithmetic is integer: the motion figure is an L1-style distance
 *          from "flat and still" (|ax| + |ay| + |az - 1000 mg|), and the filter
 *          is a shift-based exponential moving average, so the whole pipeline
 *          stays free of floating point and runs in a few microseconds.
 *
 *          Latency is captured around the sample->action span with the DWT
 *          cycle counter (SYSCLK = 16 MHz, so microseconds = cycles / 16) and
 *          mirrored on the PA6 marker pin for an independent logic-analyzer
 *          measurement.
 *
 *          Ownership: this module owns the actuator (PA5) and marker (PA6)
 *          GPIO. The sensor drivers (BME280 on SPI2, ADXL345 on I2C1) must be
 *          initialised by the caller before the first pipeline_step().
 ******************************************************************************
 */

#include "pipeline.h"
#include "main.h"
#include "gpio_driver.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "adxl345_i2c.h"
#include <stddef.h>   /* NULL */

#define ACT_PORT        GPIOA
#define ACT_PIN         (5U)      /* LD2 actuator                 */
#define MARK_PORT       GPIOA
#define MARK_PIN        (6U)      /* latency marker for the analyzer */

/* Filter strength. The EMA weight is alpha = 1 / 2^EMA_SHIFT, so shift 3 means
 * each new sample contributes 1/8 and the previous average keeps 7/8. Larger
 * shift = smoother but slower to react; smaller = twitchier.                 */
#define EMA_SHIFT       (3U)      /* filter weight: alpha = 1/8   */

/* Trip levels (in milli-g of motion) that the B1 button cycles through. Resting
 * flat the motion figure sits near 0; a firm tilt or shake pushes it up into
 * these ranges, so a higher preset needs a bigger disturbance to trip.       */
static const int32_t k_thresholds_mg[] = { 300, 600, 1000, 1500 };
#define NUM_THRESHOLDS  (sizeof(k_thresholds_mg) / sizeof(k_thresholds_mg[0]))
static uint32_t s_threshold_index = 0U;   /* which preset is currently active */

/* Integer absolute value (freestanding; avoids pulling in <stdlib.h> abs). */
static int32_t iabs32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* ---- DWT cycle counter (Cortex-M core, always present on M4) -----------
 * The Data Watchpoint and Trace unit has a free-running 32-bit cycle counter
 * (CYCCNT). Enabling it here lets pipeline_step() time itself in CPU cycles
 * with zero overhead and no spare timer. Three steps: turn on the trace
 * subsystem (TRCENA), zero the counter, then enable CYCCNT.                  */
static void dwt_cyccnt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ---- stage 1: sample both buses ----------------------------------------
 * Read the SPI environment sensor and the I2C accelerometer in the same pass,
 * then reduce the three acceleration axes to a single "how disturbed is the
 * board" number. At rest, gravity gives roughly (0, 0, 1000 mg), so measuring
 * distance from that point makes a still, level board read ~0 and any tilt or
 * shake read larger. If an I2C read fails the previous axes are kept, so one
 * dropped sample does not corrupt the figure.                               */
static void stage_sample(pipeline_t *p)
{
    p->sample_fresh = 0U;

    switch (p->mode)
    {
        case PIPE_SAMPLE_BLOCKING:
            /* The original. One call, and it does not come back until the
             * sensor is done: roughly 8 ms of the 50 ms loop period spent
             * inside a status-bit spin. Everything else in the super-loop is
             * frozen for that whole time, which is what Section 6.4 measures
             * and what Section 6.5 then explains.                          */
            if (bme280_read_all(&p->env) == 0)
            {
                p->sample_fresh = 1U;
                p->conversions++;
            }
            break;

        case PIPE_SAMPLE_NORMAL:
            /* The cheap fix. The sensor was put into free-running mode at
             * setup, so there is nothing to trigger and nothing to wait for:
             * just take whatever it last wrote. The cost is that this sample
             * may be up to one standby interval old and we cannot tell.     */
            if (bme280_read_latest(&p->env) == 0)
            {
                p->sample_fresh = 1U;
                p->conversions++;
            }
            break;

        case PIPE_SAMPLE_SPLIT:
        default:
            /* The general fix: the wait belongs to the loop, not the driver.
             * Each pass does at most one short piece of work and returns, so
             * a conversion spans several iterations instead of blocking one.
             * This is the pattern for any device that cannot free-run.      */
            switch (p->sample_state)
            {
                case SAMPLE_IDLE:
                    (void)bme280_start_measurement();
                    p->sample_state = SAMPLE_WAIT;
                    break;

                case SAMPLE_WAIT:
                    /* One cheap status read, then straight back to the loop.
                     * No spinning: if it is not ready we simply try again on
                     * the next pass.                                        */
                    if (bme280_measurement_ready() != 0)
                    {
                        p->sample_state = SAMPLE_FETCH;
                    }
                    break;

                case SAMPLE_FETCH:
                    if (bme280_fetch(&p->env) == 0)
                    {
                        p->sample_fresh = 1U;
                        p->conversions++;
                    }
                    p->sample_state = SAMPLE_IDLE;
                    break;

                default:
                    p->sample_state = SAMPLE_IDLE;
                    break;
            }
            break;
    }

    /* The accelerometer is read every pass in all three modes. Its I2C
     * transaction is short and bounded, so it is not the problem here.     */
    int32_t x = 0, y = 0, z = 0;
    if (adxl345_read_mg(&x, &y, &z) == ADXL_OK)   /* I2C1: 3-axis, milli-g */
    {
        p->ax_mg = x;
        p->ay_mg = y;
        p->az_mg = z;
    }
    /* Distance from "flat and still": zero when the board rests level. */
    p->motion_mg = iabs32(p->ax_mg) + iabs32(p->ay_mg) + iabs32(p->az_mg - 1000);
}

/* ---- stage 2: filter ---------------------------------------------------
 * Smooth the raw motion figure with an exponential moving average so a single
 * noisy sample cannot flip the actuator. Written as an integer update:
 *     filt += (raw - filt) / 2^EMA_SHIFT
 * Using the signed difference means it converges from either direction, and
 * the shift is a cheap divide-by-power-of-two.                               */
static void stage_filter(pipeline_t *p)
{
    p->motion_filt_mg += (p->motion_mg - p->motion_filt_mg) >> EMA_SHIFT;
}

/* ---- stages 3 + 4: threshold and action --------------------------------
 * Compare the filtered motion against the active threshold and drive LD2
 * accordingly (this is the "actuator"). We remember the previous state so the
 * caller can be told whether this step *changed* the output - that lets the
 * main loop log only on transitions instead of every iteration.             */
static void stage_threshold_action(pipeline_t *p)
{
    const uint8_t was_on = p->actuator_on;
    p->actuator_on = (p->motion_filt_mg >= p->threshold_mg) ? 1U : 0U;

    gpio_write_pin(ACT_PORT, ACT_PIN,
                   (p->actuator_on != 0U) ? GP_HIGH : GP_LOW);

    p->state_changed = (p->actuator_on != was_on) ? 1U : 0U;
}

/**
 * @brief  Set up the actuator/marker GPIO, enable the DWT cycle counter, and
 *         clear the pipeline state to known defaults.
 *
 *         Does NOT initialise the sensor drivers - the caller brings up SPI2 /
 *         BME280 and I2C1 / ADXL345 first, then calls this. NULL-safe.
 */
void pipeline_init(pipeline_t *p)
{
    if (p == NULL)
    {
        return;
    }

    /* Actuator (LD2) and latency marker are plain push-pull outputs. */
    const gpio_config_t act = {
        .pin = ACT_PIN, .mode = GP_MODE_OUTPUT,
        .otype = GP_OTYPE_PUSHPULL, .pull = GP_PULL_NONE
    };
    const gpio_config_t mark = {
        .pin = MARK_PIN, .mode = GP_MODE_OUTPUT,
        .otype = GP_OTYPE_PUSHPULL, .pull = GP_PULL_NONE
    };
    gpio_init(ACT_PORT, &act);
    gpio_init(MARK_PORT, &mark);
    gpio_write_pin(ACT_PORT, ACT_PIN, GP_LOW);    /* actuator starts off  */
    gpio_write_pin(MARK_PORT, MARK_PIN, GP_LOW);  /* marker starts low    */

    dwt_cyccnt_enable();

    /* Clear all running state: sensors zeroed, filter cleared, first threshold
     * preset selected, actuator off, no latency measured yet.                */
    p->env.temperature_c100 = 0;
    p->env.pressure_pa      = 0U;
    p->env.humidity_rh_1024 = 0U;
    p->ax_mg = 0; p->ay_mg = 0; p->az_mg = 0;
    p->motion_mg      = 0;
    p->motion_filt_mg = 0;
    s_threshold_index = 0U;
    p->threshold_mg   = k_thresholds_mg[s_threshold_index];
    p->actuator_on    = 0U;
    p->state_changed  = 0U;
    p->last_latency_us = 0U;

    /* Start in the blocking mode on purpose: Section 6.4 opens by showing the
     * problem before either of the fixes.                                   */
    p->mode         = PIPE_SAMPLE_BLOCKING;
    p->sample_state = SAMPLE_IDLE;
    p->sample_fresh = 0U;
    p->conversions  = 0U;
}

void pipeline_set_mode(pipeline_t *p, pipe_sample_mode_t mode)
{
    if (p == NULL)
    {
        return;
    }

    p->mode = mode;
    p->sample_state = SAMPLE_IDLE;

    if (mode == PIPE_SAMPLE_NORMAL)
    {
        /* Hand the sensor its own schedule. A 62.5 ms standby with the IIR
         * filter off sits just above the pipeline's 50 ms period, so a fresh
         * conversion is usually waiting when we look.                       */
        (void)bme280_set_mode_normal(BME280_STANDBY_62M5, BME280_FILTER_OFF);
    }
    else
    {
        /* Both forced-mode paths want the sensor asleep between conversions
         * so that nothing measures unless we ask it to.                     */
        (void)bme280_set_mode_sleep();
    }
}

const char *pipeline_mode_name(pipe_sample_mode_t mode)
{
    switch (mode)
    {
        case PIPE_SAMPLE_BLOCKING: return "blocking";
        case PIPE_SAMPLE_NORMAL:   return "normal-mode";
        case PIPE_SAMPLE_SPLIT:    return "split-phase";
        default:                   return "?";
    }
}

/**
 * @brief  Run one full sample -> filter -> threshold -> action pass and record
 *         how long it took.
 *
 *         The PA6 marker goes high for the whole measured span, so a logic
 *         analyzer reads the pulse width directly as the pipeline latency; the
 *         DWT cycle count gives the same figure in microseconds for the log.
 *         Sets p->state_changed if the actuator flipped, so the caller can log
 *         only on transitions.
 */
void pipeline_step(pipeline_t *p)
{
    if (p == NULL)
    {
        return;
    }

    gpio_write_pin(MARK_PORT, MARK_PIN, GP_HIGH);   /* analyzer: latency start */
    const uint32_t c0 = DWT->CYCCNT;                /* timestamp: before work  */

    stage_sample(p);
    stage_filter(p);
    stage_threshold_action(p);

    const uint32_t c1 = DWT->CYCCNT;                /* timestamp: after work   */
    gpio_write_pin(MARK_PORT, MARK_PIN, GP_LOW);     /* analyzer: latency end   */

    /* cycles / (cycles-per-microsecond) = microseconds. The unsigned
     * subtraction (c1 - c0) is correct even if CYCCNT wrapped in between.     */
    p->last_latency_us = (c1 - c0) / (SYSTEM_CLOCK_HZ / 1000000UL);
}

/**
 * @brief  Advance the trip threshold to the next preset, wrapping around.
 *
 *         Called from the button handler. Cheap and I/O-free (just an index
 *         bump), so it is safe to invoke in response to a B1 press.
 */
void pipeline_bump_threshold(pipeline_t *p)
{
    if (p == NULL)
    {
        return;
    }
    s_threshold_index = (s_threshold_index + 1U) % NUM_THRESHOLDS;
    p->threshold_mg   = k_thresholds_mg[s_threshold_index];
}
