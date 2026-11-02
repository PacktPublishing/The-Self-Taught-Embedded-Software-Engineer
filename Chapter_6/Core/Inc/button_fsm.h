/**
 ******************************************************************************
 * @file    button_fsm.h
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.2: a non-blocking five-state machine for one
 *          push button, distinguishing a short press from a long press
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   B1 user button on PC13 (active low, EXTI13 falling edge)
 *
 * @details This module is the answer to a question Section 6.2 raises: when is
 *          a due-check enough, and when does a subsystem need real state?
 *
 *          Blinking an LED needs no state. It has one job, it happens on a
 *          period, and sched_due() covers it completely. A button is different.
 *          Whether a press is "short" or "long" is not visible in any single
 *          sample; it depends on what happened before and how long ago. The
 *          subsystem has to remember where it is. That is the definition of a
 *          state machine, and writing it as an explicit one is what stops it
 *          from becoming a pile of flags and timestamps.
 *
 *          The five states and what moves between them:
 *
 *              IDLE      --- edge seen ------------> DEBOUNCE
 *              DEBOUNCE  --- stable down for 25 ms -> PRESSED
 *              DEBOUNCE  --- stable up   for 25 ms -> IDLE       (bounce)
 *              DEBOUNCE  --- level changed --------> DEBOUNCE    (restart window)
 *              PRESSED   --- released before long --> IDLE       (SHORT event)
 *              PRESSED   --- held past long time --> HELD        (LONG event)
 *              HELD      --- released -------------> RELEASED
 *              RELEASED  --- next tick ------------> IDLE
 *
 *          Note what DEBOUNCE actually waits for. It is not "25 ms after the
 *          first edge, look once". A contact that is still chattering at that
 *          instant would decide the outcome by luck: a real press could be
 *          discarded, or a single bounce promoted to a press. Instead the state
 *          waits for the level to hold *steady* for 25 ms, restarting its
 *          window every time the level changes. Chatter therefore extends the
 *          wait rather than corrupting the decision.
 *
 *          Two design points worth noticing. The LONG event fires on entry to
 *          HELD, not on release, so the user gets feedback while still holding
 *          the button, which is what every phone and appliance does. And
 *          RELEASED exists as its own state rather than returning straight to
 *          IDLE so that releasing after a long press cannot be mistaken for a
 *          fresh short press.
 *
 *          Nothing here blocks, allocates, or touches a peripheral. The module
 *          is handed a "is the button down right now" reading and the current
 *          millisecond count, and it returns an event. That keeps it testable
 *          on the host and keeps the GPIO knowledge in the main file.
 ******************************************************************************
 */

#ifndef BUTTON_FSM_H
#define BUTTON_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Milliseconds the level must stay stable before a press is believed. */
#define BTN_DEBOUNCE_MS     (25U)

/** Milliseconds a press must be held before it counts as a long press. */
#define BTN_LONG_PRESS_MS   (800U)

/** The five states. Exposed so the caller can print the current one. */
typedef enum
{
    BTN_IDLE = 0,      /* nothing happening; waiting for an edge          */
    BTN_DEBOUNCE,      /* edge seen, waiting for the contact to settle    */
    BTN_PRESSED,       /* confirmed down, not yet long enough to be HELD  */
    BTN_HELD,          /* down past the long-press threshold              */
    BTN_RELEASED       /* let go after a long press; one tick of cleanup  */
} btn_state_t;

/** What the machine reports on a given tick. */
typedef enum
{
    BTN_EVENT_NONE = 0,  /* nothing to report this tick   */
    BTN_EVENT_SHORT,     /* pressed and released quickly  */
    BTN_EVENT_LONG       /* held past BTN_LONG_PRESS_MS   */
} btn_event_t;

/** Machine instance. One per button; the caller owns the storage. */
typedef struct
{
    btn_state_t state;       /* where the machine currently is           */
    uint32_t    t_mark;      /* millis() when the current window started */
    uint8_t     last_level;  /* level at the last sample, for debouncing */
} button_fsm_t;

/**
 * @brief  Put the machine into a known state. Call once before the loop.
 */
void button_fsm_init(button_fsm_t *b);

/**
 * @brief  Advance the machine by one super-loop iteration.
 *
 *         Cheap enough to call every pass: it is a switch on one enum plus at
 *         most one unsigned subtraction. It never waits for anything, so the
 *         blink and the UART echo keep running at full speed alongside it.
 *
 * @param  b         The machine instance.
 * @param  is_down   1 if the button is physically pressed right now, else 0.
 * @param  now_ms    Current millisecond count (the super-loop's millis()).
 * @return The event produced on this tick, or BTN_EVENT_NONE.
 */
btn_event_t button_fsm_update(button_fsm_t *b, uint8_t is_down, uint32_t now_ms);

/** @brief Human-readable name of a state, for logging the machine's path. */
const char *button_fsm_state_name(btn_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_FSM_H */
