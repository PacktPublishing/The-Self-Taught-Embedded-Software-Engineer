/**
 ******************************************************************************
 * @file    button_fsm.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.2: five-state button machine (implementation)
 *
 * @details The whole machine is one switch on b->state. That is deliberate and
 *          it is the shape Section 6.2 argues for: every state is visible in
 *          one place, every transition is one line, and a reader can check the
 *          diagram against the code without holding anything in their head.
 *
 *          Each state records when it was entered (b->t_mark) and compares
 *          against it with unsigned subtraction, the same trick sched_due()
 *          uses, so timing stays correct across the ~49 day millis() wrap.
 *
 *          Chapter 10 comes back to this file. When a machine has five states
 *          a switch is the clearest thing you can write; when it has twenty,
 *          the table-driven form in Section 10.3 wins. Seeing the switch first
 *          is what makes that later argument concrete.
 ******************************************************************************
 */

#include "button_fsm.h"
#include <stddef.h>   /* NULL */

/* Move to a new state and restart that state's timer. Every transition goes
 * through here so no state can be entered without its t_mark being set. */
static void enter(button_fsm_t *b, btn_state_t next, uint32_t now_ms)
{
    b->state  = next;
    b->t_mark = now_ms;
}

/* Milliseconds spent in the current state. Unsigned subtraction keeps this
 * correct even when the millisecond counter wraps past 2^32. */
static uint32_t elapsed(const button_fsm_t *b, uint32_t now_ms)
{
    return (uint32_t)(now_ms - b->t_mark);
}

void button_fsm_init(button_fsm_t *b)
{
    if (b == NULL)
    {
        return;
    }
    b->state      = BTN_IDLE;
    b->t_mark     = 0U;
    b->last_level = 0U;   /* button assumed released at startup */
}

btn_event_t button_fsm_update(button_fsm_t *b, uint8_t is_down, uint32_t now_ms)
{
    if (b == NULL)
    {
        return BTN_EVENT_NONE;
    }

    btn_event_t event = BTN_EVENT_NONE;

    switch (b->state)
    {
        case BTN_IDLE:
            /* Nothing is happening until the button goes down. No timer runs
             * in this state, so an idle button costs one comparison a pass. */
            if (is_down != 0U)
            {
                enter(b, BTN_DEBOUNCE, now_ms);
                b->last_level = is_down;
            }
            break;

        case BTN_DEBOUNCE:
            /* Mechanical contacts chatter for a few milliseconds. Rather than
             * delaying here (which would stall the blink and the echo), the
             * machine refuses to decide until the level has held steady for
             * BTN_DEBOUNCE_MS of loop time.
             *
             * The restart below is the important part. Waiting a fixed 25 ms
             * and then taking a single sample would let a contact that is
             * still bouncing at that exact instant decide the outcome. By
             * restarting the window on every change, chatter can only delay
             * the decision, never corrupt it.                              */
            if (is_down != b->last_level)
            {
                b->last_level = is_down;
                b->t_mark     = now_ms;              /* unstable: start over */
            }
            else if (elapsed(b, now_ms) >= BTN_DEBOUNCE_MS)
            {
                if (is_down != 0U)
                {
                    enter(b, BTN_PRESSED, now_ms);   /* a real press */
                }
                else
                {
                    enter(b, BTN_IDLE, now_ms);      /* it was bounce */
                }
            }
            else
            {
                /* stable so far, but not for long enough yet */
            }
            break;

        case BTN_PRESSED:
            /* Confirmed down. Two ways out: let go now and it was a short
             * press, or keep holding and it becomes a long one.            */
            if (is_down == 0U)
            {
                event = BTN_EVENT_SHORT;
                enter(b, BTN_IDLE, now_ms);
            }
            else if (elapsed(b, now_ms) >= BTN_LONG_PRESS_MS)
            {
                /* Report the long press on entry to HELD, not on release, so
                 * the user gets feedback while still holding the button.    */
                event = BTN_EVENT_LONG;
                enter(b, BTN_HELD, now_ms);
            }
            else
            {
                /* still deciding */
            }
            break;

        case BTN_HELD:
            /* The long press has already been reported. Wait for release
             * without emitting anything further, so holding the button does
             * not produce a stream of repeated events.                     */
            if (is_down == 0U)
            {
                enter(b, BTN_RELEASED, now_ms);
            }
            break;

        case BTN_RELEASED:
            /* One tick of cleanup. Going through a separate state rather than
             * straight back to IDLE is what stops the release of a long press
             * from being read as the start of a new short one.              */
            enter(b, BTN_IDLE, now_ms);
            break;

        default:
            /* Unreachable with the enum above, but a machine that finds
             * itself in an unknown state should recover, not wedge.        */
            enter(b, BTN_IDLE, now_ms);
            break;
    }

    return event;
}

const char *button_fsm_state_name(btn_state_t s)
{
    switch (s)
    {
        case BTN_IDLE:     return "IDLE";
        case BTN_DEBOUNCE: return "DEBOUNCE";
        case BTN_PRESSED:  return "PRESSED";
        case BTN_HELD:     return "HELD";
        case BTN_RELEASED: return "RELEASED";
        default:           return "?";
    }
}
