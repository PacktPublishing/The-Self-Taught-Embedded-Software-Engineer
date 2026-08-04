/**
 ******************************************************************************
 * @file    main_Ex_3_4b_MultiFile_Static_Extern.c
 * @author  Renato Soriano
 * @brief   Chapter 3 — Exercise 3.4b: Multi-file project — `static` / `extern`
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details Three .c files cooperate in this exercise:
 *
 *            main.c             ← this file (the application)
 *            counter_module.c   ← all-static internal state + public API
 *            build_info.c       ← external-linkage globals
 *
 *          What you will practice:
 *
 *          - Calling functions whose implementation is in a separate .c
 *            file, through a shared .h header.
 *
 *          - Declaring `extern` globals defined in another translation
 *            unit (build_info.c) and using them safely.
 *
 *          - Confirming that file-scope `static` variables in
 *            counter_module.c are NOT reachable from main.c — the line
 *            below that would do so is left commented out and will fail
 *            to link if uncommented. That failure is part of the lesson.
 ******************************************************************************
 */

#include "main.h"
#include "itm_debug.h"
#include "counter_module.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* -------------------------------------------------------------------------
 * `extern` declarations matching the definitions in build_info.c.
 *
 * The compiler is told "these symbols exist somewhere else"; the linker
 * resolves them at link time. If we forgot to compile build_info.c, the
 * link would fail with an "undefined reference" error — which is exactly
 * what `extern` is for: a deliberate cross-file contract.
 * ------------------------------------------------------------------------- */
extern const char     g_build_version[];
extern const uint32_t g_build_year;

/* -------------------------------------------------------------------------
 * Negative example — uncommenting this line will cause an "undefined
 * reference" at link time, because `s_counter` has internal linkage in
 * counter_module.c.
 *
 *     extern uint32_t s_counter;   // <- intentionally illegal: do NOT use
 * ------------------------------------------------------------------------- */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\nChapter 3 - Exercise 3.4b: Multi-file static / extern\r\n");
    printf("------------------------------------------------------------\r\n");

    /* Read externally-defined globals from build_info.c ---------------- */
    printf("Build version (from build_info.c) : %s\r\n", g_build_version);
    printf("Build year    (from build_info.c) : %lu\r\n",
           (unsigned long)g_build_year);

    /* Exercise the counter module's public API ------------------------- */
    printf("\r\nUsing counter_module public API:\r\n");
    counter_reset();
    (void)counter_increment();
    (void)counter_increment();
    (void)counter_increment();
    printf("Final value via counter_get() : %lu\r\n",
           (unsigned long)counter_get());

    /* If you try to read counter_module.c's s_counter directly from here,
     * the link will fail — that is the whole point of internal linkage.  */

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
