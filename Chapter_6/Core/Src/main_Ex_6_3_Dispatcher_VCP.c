/**
 ******************************************************************************
 * @file    main_Ex_6_3_Dispatcher_VCP.c
 * @author  Renato Soriano
 * @brief   Chapter 6 - Exercise 6.3: an interrupt-fed ring buffer driving a
 *          command dispatcher in the main loop   [VCP / USART2 build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @board   LD2 = PA5 (heartbeat / command target) | USART2 PA2/PA3 -> VCP
 * @output  A serial terminal on the ST-Link Virtual COM Port, 115200 8N1
 *
 * @details The receive side is fully asynchronous: every byte you type raises
 *          USART2_IRQn, the handler pushes it into a lock-free ring buffer and
 *          returns. The main loop drains that buffer at its own pace and feeds
 *          the bytes to the line assembler; a completed line is dispatched to a
 *          handler. Meanwhile a 1 ms TIM3 interrupt runs a background heartbeat
 *          on LD2, proving the two activities are genuinely independent - the
 *          heartbeat never stalls while you type, and typing is never dropped
 *          while the heartbeat runs.
 *
 *          Two contexts share LD2 and the heartbeat state: the TIM3 ISR (which
 *          toggles the LED) and the command handlers (which drive it directly).
 *          They coordinate through the `volatile` flag `s_blink_enabled`: a
 *          "led" command clears it to take manual control, a "blink" command
 *          sets it to hand control back to the timer. That flag is exactly the
 *          kind of short, safe ISR-to-main signal Section 6.3 is about.
 *
 *          Commands:
 *            led on | off | toggle   drive LD2 directly (stops the heartbeat)
 *            blink <ms>              resume the heartbeat at <ms> per half-cycle
 *            ping                    replies "pong"
 *            help                    lists the commands
 ******************************************************************************
 */

#include "main.h"
#include "gpio_driver.h"
#include "tim3_driver.h"
#include "uart_driver_isr.h"
#include "cmd_dispatcher.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#define LED_PORT        GPIOA
#define LED_PIN         (5U)          /* LD2, the on-board green LED */

/* Heartbeat state, shared between the main loop (writers) and the TIM3 ISR
 * (reader). `volatile` because it crosses the ISR/thread boundary:
 *   - s_blink_enabled : 1 = timer owns the LED, 0 = a command owns it.
 *   - s_blink_period  : half-cycle length in ms (LED toggles every period).  */
static volatile uint8_t  s_blink_enabled = 1U;
static volatile uint32_t s_blink_period  = 500U;   /* ms per half-cycle */

/* ---- port the dispatcher's reply sink to this build's UART -------------
 * cmd_dispatcher.c does not know which UART it is talking to; it just calls a
 * cmd_out_fn. This adapter binds that generic sink to USART2 for the VCP
 * build. The USART6 build supplies the same function over usart6_write_string,
 * so the command set is identical across both builds.                       */
static void out_uart(const char *s)
{
    uart2_write_string(s);
}

/* ---- tiny non-negative integer parse (freestanding) --------------------
 * Reads leading decimal digits from @p s and returns their value. Sets *ok to
 * 1 if at least one digit was seen, 0 otherwise (so "blink" with no number is
 * rejected). Deliberately minimal: no sign, no overflow check - inputs here
 * are small millisecond values typed by a human.                            */
static uint32_t parse_u32(const char *s, int *ok)
{
    uint32_t v = 0U;
    int seen = 0;
    while ((*s >= '0') && (*s <= '9'))
    {
        v = (v * 10U) + (uint32_t)(*s - '0');   /* shift left one decimal place */
        s++;
        seen = 1;
    }
    if (ok != NULL)
    {
        *ok = seen;
    }
    return v;
}

/* ---- command handlers --------------------------------------------------
 * Each handler matches the cmd_handler_t signature: it receives the argument
 * text and a reply buffer to fill. Returning text in `reply` (rather than
 * writing the UART directly) keeps the handlers transport-agnostic - the
 * dispatcher sends whatever they leave in the buffer.                        */

/* "led on|off|toggle": take manual control of LD2. Each branch first clears
 * s_blink_enabled so the timer stops fighting us for the LED, then drives the
 * pin. Only the first character(s) of the argument are inspected; args is
 * always null-terminated, so reading args[1] is safe even for a 1-char arg.  */
static void cmd_led(const char *args, char *reply, uint32_t n)
{
    if ((args[0] == 'o') && (args[1] == 'n'))          /* "on"     */
    {
        s_blink_enabled = 0U;
        gpio_write_pin(LED_PORT, LED_PIN, GP_HIGH);
        (void)snprintf(reply, n, "led on\r\n");
    }
    else if ((args[0] == 'o') && (args[1] == 'f'))     /* "off"    */
    {
        s_blink_enabled = 0U;
        gpio_write_pin(LED_PORT, LED_PIN, GP_LOW);
        (void)snprintf(reply, n, "led off\r\n");
    }
    else if (args[0] == 't')                           /* "toggle" */
    {
        s_blink_enabled = 0U;
        gpio_toggle_pin(LED_PORT, LED_PIN);
        (void)snprintf(reply, n, "led toggled\r\n");
    }
    else
    {
        (void)snprintf(reply, n, "usage: led on|off|toggle\r\n");
    }
}

/* "blink <ms>": set the heartbeat half-period and hand the LED back to the
 * timer. Rejects a missing or zero value so the ISR never divides by a zero
 * period.                                                                    */
static void cmd_blink(const char *args, char *reply, uint32_t n)
{
    int ok = 0;
    const uint32_t ms = parse_u32(args, &ok);
    if ((ok == 0) || (ms == 0U))
    {
        (void)snprintf(reply, n, "usage: blink <ms> (ms > 0)\r\n");
        return;
    }
    s_blink_period  = ms;      /* publish period first ... */
    s_blink_enabled = 1U;      /* ... then re-enable the timer's control    */
    (void)snprintf(reply, n, "heartbeat -> %lu ms\r\n", (unsigned long)ms);
}

/* "ping": a liveness check that simply answers "pong". */
static void cmd_ping(const char *args, char *reply, uint32_t n)
{
    (void)args;
    (void)snprintf(reply, n, "pong\r\n");
}

/* Forward declaration: the command table below references cmd_help, but the
 * handler itself needs the table (via cmd_get_table) to print it - so one of
 * the two has to be declared ahead of the other.                            */
static void cmd_help(const char *args, char *reply, uint32_t n);

/* The command table: one row per command {name, handler, help text}. This is
 * the single source of truth the dispatcher searches and that "help" prints,
 * so adding a command is a one-line change here plus its handler.           */
static const cmd_entry_t k_cmds[] = {
    { "led",   cmd_led,   "led on|off|toggle - drive LD2" },
    { "blink", cmd_blink, "blink <ms>       - heartbeat rate" },
    { "ping",  cmd_ping,  "ping             - replies pong" },
    { "help",  cmd_help,  "help             - this list" },
};
#define NUM_CMDS  (sizeof(k_cmds) / sizeof(k_cmds[0]))

/* "help": list every command's help string. This one writes to the UART
 * directly (via out_uart) instead of filling `reply`, because the output is
 * multi-line and longer than the reply buffer. It leaves `reply` empty, which
 * is why the dispatcher sends nothing extra afterwards.                      */
static void cmd_help(const char *args, char *reply, uint32_t n)
{
    (void)args;
    (void)reply;
    (void)n;
    uint32_t count = 0U;
    const cmd_entry_t *t = cmd_get_table(&count);
    out_uart("commands:\r\n");
    for (uint32_t i = 0U; i < count; i++)
    {
        out_uart("  ");
        out_uart(t[i].help);
        out_uart("\r\n");
    }
}

/* ---- 1 ms heartbeat, entirely inside the timer ISR ---------------------
 * Called from TIM3_IRQHandler every 1 ms. It is deliberately tiny: when the
 * timer owns the LED it counts milliseconds and toggles LD2 once per
 * s_blink_period. The `cnt` state is `static` so it persists between calls;
 * it is reset whenever a command takes manual control so the next "blink"
 * starts a fresh half-cycle rather than mid-count.                          */
static void on_tick(void)
{
    static uint32_t cnt = 0U;
    if (s_blink_enabled == 0U)
    {
        cnt = 0U;              /* a command owns the LED - stay out of its way */
        return;
    }
    cnt++;
    if (cnt >= s_blink_period)
    {
        cnt = 0U;
        gpio_toggle_pin(LED_PORT, LED_PIN);   /* BSRR write - atomic, ISR-safe */
    }
}

/* Configure PA5/LD2 as a push-pull output. */
static void led_init(void)
{
    const gpio_config_t led = {
        .pin = LED_PIN, .mode = GP_MODE_OUTPUT,
        .otype = GP_OTYPE_PUSHPULL, .pull = GP_PULL_NONE
    };
    gpio_init(LED_PORT, &led);
}

int main(void)
{
    /* Unbuffered stdout so any printf-style debug appears immediately. */
    setvbuf(stdout, NULL, _IONBF, 0);

    /* --- bring up the pieces, in dependency order --------------------- */
    led_init();
    tim3_periodic_init(1U, on_tick);            /* 1 ms background heartbeat */

    const uart_config_t uart = { .baud = 115200U };
    uart2_init(&uart);                          /* interrupt-driven RX + ring buffer */

    cmd_dispatcher_init(k_cmds, NUM_CMDS, out_uart);  /* bind table + reply sink */

    uart2_write_string("\r\nChapter 6 - Exercise 6.3: ring-buffer command dispatcher (VCP)\r\n");
    uart2_write_string("Type 'help' and press Enter.\r\n> ");

    /* --- the cooperative super-loop ----------------------------------- */
    while (1)
    {
        uint8_t rx;

        /* Drain one byte from the RX ring buffer if the ISR has queued any.
         * uart2_getchar() is non-blocking: it returns 0 when nothing is
         * waiting, so the loop never stalls on input.                       */
        if (uart2_getchar(&rx) != 0)
        {
            uart2_write_byte(rx);                 /* echo the keystroke back */
            if (rx == (uint8_t)'\r')
            {
                uart2_write_byte((uint8_t)'\n');  /* CR -> CRLF for the terminal */
            }

            cmd_feed_char(rx);                    /* assemble the line; dispatch on Enter */

            if ((rx == (uint8_t)'\r') || (rx == (uint8_t)'\n'))
            {
                uart2_write_string("> ");         /* reprint the prompt after a line */
            }
        }
        /* The main loop never blocks: if no byte is ready it simply spins,
         * free to do other cooperative work here in a larger application. */
    }
}

/* Trap for unrecoverable errors: mask interrupts and spin so a debugger can
 * inspect the call stack. The ITM line still works with a debugger attached. */
void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
