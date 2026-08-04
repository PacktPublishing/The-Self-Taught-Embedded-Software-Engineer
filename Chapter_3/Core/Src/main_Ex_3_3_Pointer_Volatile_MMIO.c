/**
 ******************************************************************************
 * @file    main_Ex_3_3_Pointer_Volatile_MMIO.c
 * @author  Renato Soriano
 * @brief   Chapter 3 — Exercise 3.3: Pointers, `volatile`, and Memory-Mapped I/O
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details This exercise demonstrates memory-mapped I/O at its safest possible
 *          touch point — the read-only Cortex-M4 device-ID register `DBGMCU_IDCODE`
 *          at `0xE0042000`. The register requires no clock setup, no GPIO
 *          configuration, and no mode bits: it is always present and always
 *          readable.
 *
 *          DBGMCU_IDCODE layout (per STM32F446RE Reference Manual RM0390):
 *              bits [11:0]  DEV_ID   — device identifier (0x421 for F446)
 *              bits [15:12] reserved
 *              bits [31:16] REV_ID   — silicon revision
 *
 *          Three C concepts in one exercise:
 *
 *          1. Integer-to-pointer cast:
 *               `(volatile uint32_t *) 0xE0042000UL`
 *             This is how every embedded driver eventually reaches a
 *             register from C. The compiler does not care that this address
 *             is "special" — only that we treat it as a 32-bit memory cell.
 *
 *          2. The `volatile` qualifier:
 *             Without it the compiler is free to cache or eliminate
 *             register reads. With it, every dereference is emitted as a
 *             real load instruction.
 *
 *          3. `const` placement in Flash:
 *             The two `device_label_*` arrays below are declared `const`
 *             so the linker places them in `.rodata` (Flash) instead of
 *             RAM.
 ******************************************************************************
 */

#include "main.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* -------------------------------------------------------------------------
 * Memory-mapped address of the Cortex-M4 device-ID register.
 *
 * The literal carries a UL suffix because the address is wider than the
 * minimum 16-bit `int` guaranteed by the C standard — same lesson as the
 * 3.2 sidebar on integer promotion, applied to an address rather than a
 * mask.
 * ------------------------------------------------------------------------- */
#define DBGMCU_IDCODE_ADDR      (0xE0042000UL)

/* Field masks per RM0390 Section 33.6.1 -------------------------------- */
#define DBGMCU_IDCODE_DEV_ID_MSK    (0x00000FFFUL)
#define DBGMCU_IDCODE_DEV_ID_POS    (0U)
#define DBGMCU_IDCODE_REV_ID_MSK    (0xFFFF0000UL)
#define DBGMCU_IDCODE_REV_ID_POS    (16U)

/* -------------------------------------------------------------------------
 * Const lookup table — placed in Flash (.rodata).
 *
 * The reader can confirm this with `arm-none-eabi-nm` on the .elf:
 * the symbol resolves to an address inside the Flash region defined in
 * the linker script, not inside SRAM. Marking the array `const` is what
 * tells the linker to do so.
 * ------------------------------------------------------------------------- */
static const char device_label_flash[]  = "STM32F446RE (DEV_ID = 0x421)";
static const char volatile_lesson[]     = "volatile prevents the compiler from caching this read";

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static uint32_t read_dbgmcu_idcode(void);
static void     print_idcode_decoded(uint32_t idcode_raw);

/**
 * @brief  Read the 32-bit DBGMCU_IDCODE register via a `volatile` pointer
 *         cast.
 *
 *         This is the canonical "address-to-pointer" pattern. The literal
 *         0xE0042000UL is reinterpreted as a pointer-to-volatile-uint32,
 *         then dereferenced once. The `volatile` keyword is what stops the
 *         compiler from constant-folding the read away or hoisting it out
 *         of a loop.
 *
 *         Why this is safe to do in Chapter 3:
 *         DBGMCU_IDCODE is part of the ARM CoreSight debug block, which is
 *         always powered and clocked when the core is running. No RCC
 *         enable bit is required, no GPIO is touched.
 */
static uint32_t read_dbgmcu_idcode(void)
{
    volatile uint32_t * const p_idcode = (volatile uint32_t *) DBGMCU_IDCODE_ADDR;
    return *p_idcode;
}

/**
 * @brief  Decode the two fields of DBGMCU_IDCODE and print them.
 *
 *         The shift/mask pattern is exactly the bit-manipulation idiom
 *         introduced in Exercise 3.2 — same operators, same care with UL
 *         literals, just applied to a field extraction instead of a SET
 *         or CLEAR. That continuity is deliberate.
 */
static void print_idcode_decoded(uint32_t idcode_raw)
{
    const uint32_t dev_id = (idcode_raw & DBGMCU_IDCODE_DEV_ID_MSK)
                            >> DBGMCU_IDCODE_DEV_ID_POS;
    const uint32_t rev_id = (idcode_raw & DBGMCU_IDCODE_REV_ID_MSK)
                            >> DBGMCU_IDCODE_REV_ID_POS;

    printf("DBGMCU_IDCODE raw   : 0x%08lX\r\n", (unsigned long)idcode_raw);
    printf("  DEV_ID  [11:0]    : 0x%03lX  (expected 0x421 for STM32F446)\r\n",
           (unsigned long)dev_id);
    printf("  REV_ID  [31:16]   : 0x%04lX  (silicon revision)\r\n",
           (unsigned long)rev_id);
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
     HAL_Init();
    SystemClock_Config();

    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\nChapter 3 - Exercise 3.3: Pointers, volatile, Memory-Mapped I/O\r\n");
    printf("-----------------------------------------------------------------\r\n");
    printf("%s\r\n", device_label_flash);
    printf("%s\r\n\r\n", volatile_lesson);

    /* Demonstrate the pointer-cast + volatile read pattern. */
    const uint32_t idcode = read_dbgmcu_idcode();
    print_idcode_decoded(idcode);

    /* Show that Flash-resident strings live above 0x08000000 and RAM
     * variables live above 0x20000000 — a quick visual proof of `const`
     * placement.                                                         */
    uint32_t ram_variable = 0xCAFEBABEUL;
    printf("\r\nAddress of Flash-const string : %p\r\n", (void *)device_label_flash);
    printf("Address of RAM uint32_t : %p\r\n", (void *)&ram_variable);

    while (1)
    {
        /* Trap so the SWV console keeps the lines visible.               */
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
