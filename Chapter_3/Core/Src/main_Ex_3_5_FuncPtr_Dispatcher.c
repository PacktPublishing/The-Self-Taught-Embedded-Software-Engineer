/**
 ******************************************************************************
 * @file    main_Ex_3_5_FuncPtr_Dispatcher.c
 * @author  Renato Soriano
 * @brief   Chapter 3 — Exercise 3.5: Function Pointers, Callbacks, Dispatch
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details A table-driven command dispatcher built entirely in C — no
 *          UART, no GPIO, no peripheral. UART arrives in Chapter 5; the
 *          dispatcher pattern shown here is exactly what you will
 *          plug a UART receiver into later.
 *
 *          Concept covered:
 *
 *          1. Function-pointer type definition
 *             `typedef void (*command_handler_t)(void);`
 *             Declares a named type for "pointer to a function that takes
 *             no arguments and returns nothing." Using a typedef rather
 *             than the raw declarator everywhere keeps the table
 *             declarations readable.
 *
 *          2. Table-driven dispatch
 *             An array of `(char command, command_handler_t handler)` pairs
 *             replaces a long chain of `if/else` or `switch` statements.
 *             Adding a new command is one line in the table — no
 *             control-flow edit, no risk of falling through.
 *
 *          3. A registered callback
 *             The dispatcher does not decide what happens when a command
 *             is not in the table. It calls whatever handler the
 *             application registered for that case, or falls back to a
 *             built-in message if nothing was registered. Registering a
 *             function and letting somebody else's code call it back is
 *             the whole idea behind a callback, and it is the same
 *             mechanism the UART driver in Chapter 5 uses to tell the
 *             application that a byte arrived.
 *
 *          4. Linear search vs O(1) lookup
 *             A linear walk is used because the table is short and the
 *             cost is dominated by the printed output. A real driver
 *             would either keep the table sorted and bsearch it or use
 *             the command character as an array index for O(1) — both
 *             trade-offs are noted inline so you can spot them later.
 *
 *          Sidebar topic — recursion in embedded:
 *          A recursive_demo() handler is included specifically as a
 *          warning. It self-limits to 3 levels so it does not actually
 *          overflow the stack; the comments explain why deep or
 *          unbounded recursion is banned in production firmware.
 ******************************************************************************
 */

#include "main.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>     /* NULL */

/* -------------------------------------------------------------------------
 * Function-pointer type: pointer to a void(void) function.
 *
 *   typedef void (*command_handler_t)(void);
 *
 * Read it inside-out: a pointer (`*`) named `command_handler_t`, to a
 * function taking `void` and returning `void`.
 * ------------------------------------------------------------------------- */
typedef void (*command_handler_t)(void);

/* -------------------------------------------------------------------------
 * Per-command record. The table below is an array of these.
 * ------------------------------------------------------------------------- */
typedef struct
{
    char              command_char;     /* The single-char command code     */
    const char       *description;      /* Human-readable description        */
    command_handler_t handler;          /* Pointer to the action to run      */
} command_entry_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void handle_status(void);
static void handle_help(void);
static void handle_version(void);
static void handle_recursive_demo(void);
static void handle_unknown_command(void);
static void register_unknown_handler(command_handler_t callback);
static void dispatch_command(char command);

/* -------------------------------------------------------------------------
 * The callback slot.
 *
 * One function pointer, initialised to NULL, holding the address of the
 * routine to run when a command does not appear in the table. NULL is the
 * "nobody has registered yet" state, and the dispatcher checks for it
 * before every call. An uninitialised or stale function pointer is one of
 * the few ways to make a Cortex-M jump to an address that is not code, so
 * the NULL check is not defensive clutter; it is the contract.
 *
 * `static` keeps the slot private to this file. The application changes it
 * only through register_unknown_handler(), which is the same shape as the
 * HAL_..._RegisterCallback() functions in ST's driver library.
 * ------------------------------------------------------------------------- */
static command_handler_t s_on_unknown_command = NULL;

/* -------------------------------------------------------------------------
 * Command lookup table — stored in Flash (.rodata) via the `static const`
 * qualifier. The whole table is read-only at runtime.
 *
 * MISRA-style note: an explicit sentinel ('\0', NULL, NULL) terminates the
 * table so the dispatcher does not depend on a separate length variable.
 * ------------------------------------------------------------------------- */
static const command_entry_t s_command_table[] =
{
    { 's', "Print system status",         handle_status         },
    { 'h', "Print this help",             handle_help           },
    { 'v', "Print firmware version",      handle_version        },
    { 'r', "Recursive demo (3 levels)",   handle_recursive_demo },
    { '\0', NULL, NULL }    /* sentinel */
};

static void handle_status(void)
{
    printf("  [status]  system is running, no faults latched.\r\n");
}

static void handle_help(void)
{
    printf("  [help]    available commands:\r\n");
    for (uint32_t i = 0U; s_command_table[i].handler != NULL; i++)
    {
        printf("              '%c' - %s\r\n",
               s_command_table[i].command_char,
               s_command_table[i].description);
    }
}

static void handle_version(void)
{
    printf("  [version] firmware 0.1 (Chapter 3 dispatcher demo)\r\n");
}

/* -------------------------------------------------------------------------
 * Recursive-demo handler.
 *
 * IMPORTANT — this is a sidebar example, not a recommendation. In
 * production firmware, recursion is almost always banned for two reasons:
 *
 *   1. The Cortex-M stack is small (a few KB) and a single deep recursion
 *      can overrun it before any HardFault watchdog catches it.
 *
 *   2. Static analysis tools (MISRA C:2012 Rule 17.2 forbids recursion)
 *      cannot bound the worst-case stack depth.
 *
 * The function below self-limits to a depth of 3 so it cannot actually
 * cause a stack overflow during this exercise. The depth check is the
 * lesson.
 * ------------------------------------------------------------------------- */
static void recursive_helper(uint32_t depth)
{
    printf("    recursive_helper at depth %lu\r\n", (unsigned long)depth);
    if (depth >= 3U)
    {
        return;
    }
    recursive_helper(depth + 1U);
}

static void handle_recursive_demo(void)
{
    printf("  [recurse] entering bounded recursion (max depth 3)\r\n");
    recursive_helper(0U);
    printf("  [recurse] returned safely.\r\n");
}

/**
 * @brief  Application-supplied reaction to an unrecognised command.
 *
 *         Nothing in the dispatcher knows this function exists. It becomes
 *         reachable only once main() hands its address to
 *         register_unknown_handler().
 */
static void handle_unknown_command(void)
{
    printf("  [callback] application handler: command rejected, logging it\r\n");
}

/**
 * @brief  Store the address of the routine to call on an unknown command.
 *
 *         Passing NULL clears the slot and returns the dispatcher to its
 *         built-in message. A registration function that accepts NULL as
 *         "unregister" saves the caller from needing a second function,
 *         and it is the convention most embedded driver APIs follow.
 */
static void register_unknown_handler(command_handler_t callback)
{
    s_on_unknown_command = callback;
}

/**
 * @brief  Linear-scan dispatcher. Walks the command table looking for an
 *         entry whose command_char matches `command`; calls its handler if
 *         found. If no entry matches, control goes to the registered
 *         callback, or to the built-in message when no callback is set.
 *
 *         The table-driven pattern means this function never has to know
 *         which commands exist — adding a new entry to s_command_table is
 *         the only change required to extend the system. The callback slot
 *         extends the same idea to behavior the dispatcher cannot know
 *         about at all: what this particular application wants to do when
 *         an input makes no sense. A logger might record it, a test rig
 *         might count it, a production build might ignore it. The
 *         dispatcher stays identical in all three.
 */
static void dispatch_command(char command)
{
    for (uint32_t i = 0U; s_command_table[i].handler != NULL; i++)
    {
        if (s_command_table[i].command_char == command)
        {
            s_command_table[i].handler();
            return;
        }
    }

    printf("  [error]   unknown command '%c'\r\n", command);

    /* Call back into the application, but only if it asked to be called. */
    if (s_on_unknown_command != NULL)
    {
        s_on_unknown_command();
    }
}

/**
 * @brief  The application entry point.
 *
 *         A short hard-coded sequence of commands is dispatched at startup
 *         to demonstrate the table walk. In Chapter 5 the same dispatcher
 *         is reused with a UART receive callback as the input source — at
 *         that point the command source becomes external, but the
 *         dispatcher itself does not change.
 *
 * @retval int
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\nChapter 3 - Exercise 3.5: Function-pointer command dispatcher\r\n");
    printf("--------------------------------------------------------------\r\n");

    /* Hard-coded command stream. Each line shows the dispatcher selecting
     * a different handler from the table.                                */
    const char demo_commands[] = { 'h', 's', 'v', 'r', 'x' };  /* 'x' = unknown */

    for (uint32_t i = 0U; i < sizeof(demo_commands); i++)
    {
        printf("\r\n> command '%c'\r\n", demo_commands[i]);
        dispatch_command(demo_commands[i]);
    }

    /* -------------------------------------------------------------------
     * Register a callback and repeat the command that failed.
     *
     * The dispatcher source does not change. The table does not change.
     * The only difference between the two attempts at 'x' is the value
     * sitting in one function pointer, and that value was written at
     * runtime by the application rather than at compile time by the
     * author of the dispatcher. That separation is what a callback buys
     * you, and it is exactly how the UART driver in Chapter 5 will hand
     * received bytes back to your code.
     * ------------------------------------------------------------------- */
    printf("\r\nRegistering an application callback for unknown commands.\r\n");
    register_unknown_handler(handle_unknown_command);

    printf("\r\n> command 'x' (again, now that a callback is registered)\r\n");
    dispatch_command('x');

    printf("\r\nDispatcher demo complete.\r\n");

    while (1)
    {
        /* Trap so the SWV log remains visible. */
    }

    return 0;
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: HAL init failed.\r\n");
    while (1)
    {
        /* trap */
    }
}
