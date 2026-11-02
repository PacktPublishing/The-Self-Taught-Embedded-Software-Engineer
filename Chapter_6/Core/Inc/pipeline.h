/**
 ******************************************************************************
 * @file    pipeline.h
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.4: the sensor-to-action pipeline
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 *
 * @details This module is the heart of the mini integration project. It ties
 *          two independent buses and several drivers into one data flow:
 *
 *              sample  ->  filter  ->  threshold  ->  action  ->  log
 *
 *          - sample    : read the BME280 environment over SPI2 and the ADXL345
 *                        acceleration over I2C1, in the same iteration.
 *          - filter    : smooth a derived "motion" figure (how far the board is
 *                        from lying flat and still) with an integer EMA.
 *          - threshold : compare the filtered value against a level the user
 *                        can raise/lower with the B1 button.
 *          - action    : drive LD2 as the actuator when the level is crossed.
 *          - log       : hand a one-line record back to the caller to print.
 *
 *          It also measures its own end-to-end latency two ways: a cycle-count
 *          figure in microseconds (via the Cortex-M DWT counter) and a hardware
 *          marker pulse on PA6 that a logic analyzer can measure directly.
 *
 *          The module owns the actuator (PA5) and marker (PA6) GPIO. The
 *          sensor drivers must already be initialised by the caller before the
 *          first pipeline_step().
 ******************************************************************************
 */

#ifndef PIPELINE_H
#define PIPELINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bme280_spi.h"

/**
 * @brief How the pipeline gets its environmental sample.
 *
 * Section 6.4 builds the same pipeline three ways and measures each, so the
 * mode is a run-time choice rather than a compile-time one: the user flashes
 * once and compares three PA6 pulse widths on the same capture.
 */
typedef enum
{
    PIPE_SAMPLE_BLOCKING = 0,  /* bme280_read_all(): waits ~8 ms in the loop  */
    PIPE_SAMPLE_NORMAL,        /* sensor free-runs; the read never waits      */
    PIPE_SAMPLE_SPLIT          /* start / ready / fetch across loop passes    */
} pipe_sample_mode_t;

/**
 * @brief States of the split-phase sample stage.
 *
 * Only PIPE_SAMPLE_SPLIT uses these. The stage starts a conversion, returns to
 * the loop, polls cheaply on later passes, and fetches when the sensor says it
 * is done. Nothing here ever waits.
 */
typedef enum
{
    SAMPLE_IDLE = 0,   /* nothing in flight; ready to start a conversion  */
    SAMPLE_WAIT,       /* conversion running; poll the status bit         */
    SAMPLE_FETCH       /* conversion done; burst-read and compensate      */
} sample_state_t;

/** Running state of the pipeline, inspected by the logging stage. */
typedef struct
{
    bme280_reading_t env;          /* latest temperature/pressure/humidity  */
    int32_t  ax_mg, ay_mg, az_mg;  /* latest acceleration, milli-g          */
    int32_t  motion_mg;            /* derived: deviation from flat + still   */
    int32_t  motion_filt_mg;       /* EMA-smoothed motion                    */
    int32_t  threshold_mg;         /* current trip level (button-adjustable) */
    uint8_t  actuator_on;          /* current LD2 state (1 = on)             */
    uint8_t  state_changed;        /* set by the last step if LD2 flipped    */
    uint32_t last_latency_us;      /* sample->action time, microseconds      */

    /* --- sampling strategy and its state ------------------------------- */
    pipe_sample_mode_t mode;       /* which of the three sample paths runs   */
    sample_state_t     sample_state; /* split-phase stage position           */
    uint8_t  sample_fresh;         /* 1 when this step produced a new sample */
    uint32_t conversions;          /* completed conversions, for the log     */
} pipeline_t;

/**
 * @brief  Set up the actuator/marker GPIO, enable the DWT cycle counter, and
 *         initialise the pipeline state (filter cleared, default threshold,
 *         actuator off). Sensor drivers are NOT initialised here.
 */
void pipeline_init(pipeline_t *p);

/**
 * @brief  Select which of the three sampling strategies the pipeline uses, and
 *         reconfigure the sensor to match.
 *
 *         PIPE_SAMPLE_NORMAL puts the BME280 into free-running mode; the other
 *         two return it to sleep and drive it in forced mode. Safe to call at
 *         run time, which is what lets one flashed build produce all three
 *         latency captures for the Section 6.4 comparison.
 */
void pipeline_set_mode(pipeline_t *p, pipe_sample_mode_t mode);

/** @brief Human-readable name of a sampling mode, for the log line. */
const char *pipeline_mode_name(pipe_sample_mode_t mode);

/**
 * @brief  Run one full sample -> filter -> threshold -> action pass and record
 *         its latency. Sets @p p->state_changed if the actuator flipped, so the
 *         caller can log only on transitions. Does not print anything itself.
 */
void pipeline_step(pipeline_t *p);

/**
 * @brief  Advance the trip threshold to the next preset (wraps around). Call
 *         this from the button handler; it is cheap and does no I/O.
 */
void pipeline_bump_threshold(pipeline_t *p);

#ifdef __cplusplus
}
#endif

#endif /* PIPELINE_H */
