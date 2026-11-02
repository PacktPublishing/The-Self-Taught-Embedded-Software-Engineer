/**
 ******************************************************************************
 * @file    main_Ex_7_1_Memory_Map.c
 * @author  Renato Soriano
 * @brief   Chapter 7 - Exercise 7.1: where code, data, and the stack live,
 *          and what the startup code did before main()   [SWV ITM build]
 *
 * @target  STM32F446RE - Nucleo-F446RE (ARM Cortex-M4)
 * @output  SWV ITM Data Console, Stimulus Port 0, core clock 16000000 Hz
 *
 * @details Chapter 3 printed two addresses to prove that a const string sits
 *          in Flash and a local variable sits in SRAM. This exercise reads
 *          the whole map instead, from the same symbols the linker script
 *          defines and the startup file uses, and then hands the reader the
 *          addresses to type into the STM32CubeIDE Memory Browser.
 *
 *          It shows four things:
 *            1. The section boundaries the linker chose (_sidata, _sdata,
 *               _edata, _sbss, _ebss, _end, _estack, _Min_Stack_Size).
 *            2. Which section each kind of C object lands in: a function, a
 *               const table, an initialized global, a zeroed global, a local.
 *            3. Evidence of what ran before main(): the first two words of the
 *               vector table, the .data copy from Flash, the zeroed .bss.
 *            4. How much stack main() is using at this point.
 *
 *          The .data comparison runs first, before any printf(). The C
 *          library keeps its own state in .data (the stdout stream lives
 *          there), so the first printf() changes RAM and the two copies stop
 *          matching. That is not a bug: the Flash copy holds starting values
 *          only, and .data is ordinary live memory from then on.
 *
 *          Every address printed here comes from the build, so your numbers
 *          will differ from the book's whenever your build differs.
 ******************************************************************************
 */

#include "main.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* ---- Symbols defined by STM32F446RETX_FLASH.ld and the startup file ----
 * A linker symbol is an address with no storage behind it. Declaring it as
 * a uint32_t and taking its address is the conventional way to read it. */
extern uint32_t _etext;          /* end of .text in Flash                     */
extern uint32_t _sidata;         /* load address of .data (in Flash)          */
extern uint32_t _sdata;          /* run address of .data (in SRAM)            */
extern uint32_t _edata;          /* end of .data, one past its last byte      */
extern uint32_t _sbss;           /* start of .bss, zeroed by Reset_Handler    */
extern uint32_t _ebss;           /* end of .bss, one past its last byte       */
extern uint32_t _end;            /* heap start, 8-byte aligned after .bss     */
extern uint32_t _estack;         /* top of SRAM: the initial stack pointer    */
extern uint32_t _Min_Stack_Size; /* reserved stack, 0x400 by default          */
extern uint32_t _Min_Heap_Size;  /* reserved heap, 0x200 by default           */
extern const uint32_t g_pfnVectors[];   /* the vector table, startup file     */
extern void Reset_Handler(void);

#define FLASH_BASE_ADDR   (0x08000000UL)
#define FLASH_END_ADDR    (0x08080000UL)   /* 512 KB */
#define PERIPH_BASE_ADDR  (0x40000000UL)   /* APB1, APB2, AHB1, AHB2 (Table 3.1) */
#define PERIPH_END_ADDR   (0x60000000UL)
#define SRAM_BASE_ADDR    (0x20000000UL)
#define ALIVE_LOOPS       (1600000UL)      /* roughly 1 s per pass at 16 MHz */

/* One object of each kind, so each section has something of ours in it. */
static const uint16_t k_lookup[8] = { 0U, 1U, 4U, 9U, 16U, 25U, 36U, 49U }; /* .rodata */
static uint32_t s_alive = 0xC0FFEE00UL;   /* .data: initial value copied from Flash */
static uint32_t s_zero_block[16];         /* .bss: zeroed by the startup code       */

static uint32_t addr_of(const void *p)
{
    return (uint32_t)p;
}

/* Name the region an address falls in, using the linker's own boundaries. */
static const char *region_of(uint32_t a)
{
    if ((a >= FLASH_BASE_ADDR) && (a < addr_of(&_etext)))       { return "Flash .text";   }
    if ((a >= addr_of(&_etext)) && (a < FLASH_END_ADDR))        { return "Flash .rodata"; }
    if ((a >= addr_of(&_sdata)) && (a < addr_of(&_edata)))      { return "SRAM .data";    }
    if ((a >= addr_of(&_sbss))  && (a < addr_of(&_ebss)))       { return "SRAM .bss";     }
    if ((a >= addr_of(&_estack) - addr_of(&_Min_Stack_Size)) &&
        (a < addr_of(&_estack)))                                { return "SRAM stack";    }
    if ((a >= addr_of(&_end)) && (a < addr_of(&_estack)))       { return "SRAM free (heap grows up, stack grows down)"; }
    if ((a >= PERIPH_BASE_ADDR) && (a < PERIPH_END_ADDR))       { return "peripheral registers"; }
    return "outside Flash/SRAM";
}

/* Count the words of .data whose RAM value differs from the Flash copy.
 * The walk uses integer addresses, as the startup code does, because the
 * linker symbols are separate C objects as far as the compiler knows. */
static uint32_t data_words_differing(void)
{
    const uint32_t words = (addr_of(&_edata) - addr_of(&_sdata)) / 4U;
    const volatile uint32_t *flash_copy = (const volatile uint32_t *)addr_of(&_sidata);
    const volatile uint32_t *ram_copy   = (const volatile uint32_t *)addr_of(&_sdata);
    uint32_t diff = 0U;

    for (uint32_t i = 0U; i < words; i++)
    {
        if (ram_copy[i] != flash_copy[i])
        {
            diff++;
        }
    }
    return diff;
}

static uint32_t bss_block_is_zero(void)
{
    for (uint32_t i = 0U; i < (sizeof(s_zero_block) / sizeof(s_zero_block[0])); i++)
    {
        if (s_zero_block[i] != 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

static void crude_delay(uint32_t loops)
{
    for (volatile uint32_t i = 0U; i < loops; i++)
    {
        /* pacing only */
    }
}

int main(void)
{
    /* Before anything else touches .data, compare it with its Flash copy. */
    const uint32_t data_diff_at_entry = data_words_differing();
    const uint32_t bss_zero_at_entry  = bss_block_is_zero();
    const uint32_t sp_at_entry        = __get_MSP();
    uint32_t local_var = 0x1234ABCDUL;          /* lives in main()'s stack frame */

    setvbuf(stdout, NULL, _IONBF, 0);

    /* Clock GPIOC so its input register reads the real pin. This exercise
     * carries no GPIO driver, so the one RCC write is done directly; PC13
     * already resets as an input with B1's external pull-up on the Nucleo. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    const uint32_t data_bytes = addr_of(&_edata) - addr_of(&_sdata);
    const uint32_t bss_bytes  = addr_of(&_ebss)  - addr_of(&_sbss);
    const uint32_t flash_used = (addr_of(&_sidata) + data_bytes) - FLASH_BASE_ADDR;
    const uint32_t stack_rsv  = addr_of(&_Min_Stack_Size);
    const uint32_t stack_lim  = addr_of(&_estack) - stack_rsv;

    printf("\r\nChapter 7 - Exercise 7.1: where code, data, and the stack live\r\n");

    printf("\r\n-- Linker script boundaries --\r\n");
    printf("vector table (g_pfnVectors) 0x%08lX\r\n", (unsigned long)addr_of(g_pfnVectors));
    printf("end of .text   (_etext)     0x%08lX\r\n", (unsigned long)addr_of(&_etext));
    printf(".data in Flash (_sidata)    0x%08lX\r\n", (unsigned long)addr_of(&_sidata));
    printf(".data in SRAM  (_sdata)     0x%08lX  %lu bytes\r\n",
           (unsigned long)addr_of(&_sdata), (unsigned long)data_bytes);
    printf(".bss           (_sbss)      0x%08lX  %lu bytes\r\n",
           (unsigned long)addr_of(&_sbss), (unsigned long)bss_bytes);
    printf("heap start     (_end)       0x%08lX  min %lu bytes\r\n",
           (unsigned long)addr_of(&_end), (unsigned long)addr_of(&_Min_Heap_Size));
    printf("stack limit                 0x%08lX  %lu bytes reserved\r\n",
           (unsigned long)stack_lim, (unsigned long)stack_rsv);
    printf("stack top      (_estack)    0x%08lX\r\n", (unsigned long)addr_of(&_estack));
    printf("Flash used %lu of 524288 bytes, static SRAM %lu of 131072 bytes\r\n",
           (unsigned long)flash_used, (unsigned long)(data_bytes + bss_bytes));

    printf("\r\n-- Where each object landed --\r\n");
    printf("main()        0x%08lX  %s\r\n", (unsigned long)(uint32_t)&main,
           region_of((uint32_t)&main & ~1UL));
    printf("k_lookup[]    0x%08lX  %s\r\n", (unsigned long)addr_of(k_lookup),
           region_of(addr_of(k_lookup)));
    printf("s_alive       0x%08lX  %s\r\n", (unsigned long)addr_of(&s_alive),
           region_of(addr_of(&s_alive)));
    printf("s_zero_block  0x%08lX  %s\r\n", (unsigned long)addr_of(s_zero_block),
           region_of(addr_of(s_zero_block)));
    printf("local_var     0x%08lX  %s\r\n", (unsigned long)addr_of(&local_var),
           region_of(addr_of(&local_var)));
    printf("GPIOC->IDR    0x%08lX  %s\r\n", (unsigned long)(uint32_t)&GPIOC->IDR,
           region_of((uint32_t)&GPIOC->IDR));

    printf("\r\n-- Evidence of what ran before main() --\r\n");
    printf("vector[0] initial SP  0x%08lX  equals _estack: %s\r\n",
           (unsigned long)g_pfnVectors[0],
           (g_pfnVectors[0] == addr_of(&_estack)) ? "yes" : "no");
    printf("vector[1] reset entry 0x%08lX  Reset_Handler 0x%08lX\r\n",
           (unsigned long)g_pfnVectors[1], (unsigned long)(uint32_t)&Reset_Handler);
    /* At reset VTOR is 0. With BOOT0 low the chip aliases Flash at address 0,
     * so 0x00000000 here still means "the table at 0x08000000". */
    printf("VTOR                  0x%08lX%s\r\n", (unsigned long)SCB->VTOR,
           (SCB->VTOR == 0UL) ? "  (Flash aliased at address 0 by BOOT0)" : "");
    printf(".data matched its Flash copy at entry: %s (%lu words differed)\r\n",
           (data_diff_at_entry == 0U) ? "yes" : "no", (unsigned long)data_diff_at_entry);
    printf(".data words differing now, after printf: %lu\r\n",
           (unsigned long)data_words_differing());
    printf(".bss block was zero at entry: %s\r\n", (bss_zero_at_entry != 0U) ? "yes" : "no");
    printf("stack in use at entry to main(), its own frame included: %lu of %lu bytes\r\n",
           (unsigned long)(addr_of(&_estack) - sp_at_entry), (unsigned long)stack_rsv);

    printf("\r\n-- Open Window > Show View > Memory Browser and visit --\r\n");
    printf("0x%08lX  vector table: initial SP, then Reset_Handler\r\n",
           (unsigned long)addr_of(g_pfnVectors));
    printf("0x%08lX  k_lookup[] in Flash (0, 1, 4, 9, ...)\r\n", (unsigned long)addr_of(k_lookup));
    printf("0x%08lX  s_alive in SRAM, watch it count\r\n", (unsigned long)addr_of(&s_alive));
    printf("0x%08lX  the live stack (the SP register in the Registers view)\r\n",
           (unsigned long)__get_MSP());
    printf("0x%08lX  GPIOC_IDR: bit 13 reads 0 while you hold B1\r\n",
           (unsigned long)(uint32_t)&GPIOC->IDR);

    while (1)
    {
        s_alive++;              /* changes once per pass: watch it in the browser */
        local_var ^= k_lookup[s_alive & 0x7UL];
        crude_delay(ALIVE_LOOPS);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    itm_send_string("ERROR: trapped in Error_Handler.\r\n");
    while (1)
    {
    }
}
