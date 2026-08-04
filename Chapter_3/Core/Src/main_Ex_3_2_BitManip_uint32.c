/**
 ******************************************************************************
 * @file    main_Ex_3_2_BitManip_uint32.c
 * @author  Renato Soriano
 * @brief   Chapter 3 — Exercise 3.2: Bitwise Operations for Bit Manipulation
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @output  SWV ITM Data Console (Stimulus Port 0) — set up in Chapter 2.5
 *
 * @details You will see how AND, OR, XOR, NOT, and shifts express the
 *          three primitive bit operations:
 *
 *            - SET    bit n   →   value |=  (1UL << n)
 *            - CLEAR  bit n   →   value &= ~(1UL << n)
 *            - TOGGLE bit n   →   value ^=  (1UL << n)
 *
 *          Each operation prints the before/after state in hex and binary so
 *          you can match what the code did to what the bits look like.
 *          The same pattern is reused later in Chapter 4 against real
 *          GPIO_MODER / GPIOA_ODR registers.
 *
 *          Sidebar topic — integer-promotion trap:
 *          `1 << 31` is undefined behavior on a 32-bit `int` because the
 *          literal `1` has type `int`. Always write `1UL << n` (or use a
 *          `uint32_t` mask variable) when the operand is wider than 16 bits.
 *          The mask helpers below take that approach.
 ******************************************************************************
 */

#include "main.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static uint32_t set_bit(uint32_t value, uint8_t bit_pos);
static uint32_t clear_bit(uint32_t value, uint8_t bit_pos);
static uint32_t toggle_bit(uint32_t value, uint8_t bit_pos);
static void print_state(const char *label, uint32_t value);
static void print_binary_uint32(uint32_t value);

/**
 * @brief  Set bit `bit_pos` of `value` to 1.
 *
 * @note   The mask uses 1UL (not 1) to guarantee a 32-bit shift result and
 *         avoid the integer-promotion trap when `bit_pos >= 16`.
 */
static uint32_t set_bit(uint32_t value, uint8_t bit_pos)
{
    return value | (1UL << bit_pos);
}

/**
 * @brief  Clear bit `bit_pos` of `value` to 0 (leaves all other bits intact).
 *
 *         The two-step idiom — build a positive mask, invert it, AND it in —
 *         is the canonical clear-bit pattern in embedded C and reappears in
 *         every peripheral driver later in the book.
 */
static uint32_t clear_bit(uint32_t value, uint8_t bit_pos)
{
    return value & ~(1UL << bit_pos);
}

/**
 * @brief  Flip bit `bit_pos` of `value` (1 → 0 or 0 → 1).
 *
 *         XOR with a 1-hot mask is the standard toggle pattern; the
 *         hardware-level equivalent is the BSRR register pattern used in
 *         Chapter 4.
 */
static uint32_t toggle_bit(uint32_t value, uint8_t bit_pos)
{
    return value ^ (1UL << bit_pos);
}

/**
 * @brief  Print a 32-bit value in hex and binary, prefixed with a label.
 *
 *         Binary output is shown MSB-first with a separator every 4 bits so
 *         the reader can spot which bit moved at a glance — much easier than
 *         counting 32 contiguous digits.
 */
static void print_state(const char *label, uint32_t value)
{
    printf("%s :\r 0x%08lX  |  ", label, (unsigned long)value);
    print_binary_uint32(value);
    printf("\r\n");
}

static void print_binary_uint32(uint32_t value)
{
    for (int8_t bit = 31; bit >= 0; bit--)
    {
        printf("%c", ((value >> (uint8_t)bit) & 1UL) ? '1' : '0');
        if ((bit % 4) == 0 && bit != 0)
        {
            printf("_");
        }
    }
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    uint32_t reg_image  = 0x00000000UL;   /* "register image" — a uint32_t we will manipulate */
    const uint8_t target_bit_a = 5U;       /* mimic the position of PA5 / LD2 for familiarity */
    const uint8_t target_bit_b = 17U;      /* a higher bit to exercise the >=16 path          */

    HAL_Init();
    SystemClock_Config();

    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\nChapter 3 - Exercise 3.2: Bitwise Operations on uint32_t\r\n");
    printf("------------------------------------------------------------\r\n");

    print_state("Initial value", reg_image);

    /* SET bit 5 -------------------------------------------------------- */
    reg_image = set_bit(reg_image, target_bit_a);
    print_state("After SET bit 5", reg_image);

    /* SET bit 17 (exercises the integer-promotion path) ---------------- */
    reg_image = set_bit(reg_image, target_bit_b);
    print_state("After SET bit 17", reg_image);

    /* TOGGLE bit 5 (now goes 1 -> 0) ----------------------------------- */
    reg_image = toggle_bit(reg_image, target_bit_a);
    print_state("After TOGGLE bit 5", reg_image);

    /* CLEAR bit 17 ----------------------------------------------------- */
    reg_image = clear_bit(reg_image, target_bit_b);
    print_state("After CLEAR bit 17", reg_image);

    printf("------------------------------------------------------------\r\n");
    printf("Done. Compare each line's binary column with the operation.\r\n");

    /* Infinite loop */
    while (1)
    {
        /* All work happens once at startup; trap here so the SWV console
         * keeps its log visible while the reader inspects the output.    */
    }

    return 0;
}

/**
 * @brief  System Clock Configuration — HSI 16 MHz default, sufficient for SWV.
 */
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
