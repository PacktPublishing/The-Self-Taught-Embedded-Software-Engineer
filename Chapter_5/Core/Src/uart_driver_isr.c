/**
 ******************************************************************************
 * @file    uart_driver_isr.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2b: USART2 interrupt-driven RX (implementation)
 *
 * @details Adds to the 5.2a register set:
 *
 *              USART_CR1 RXNEIE  §25.6.4   RXNE interrupt enable
 *              USART_SR  RXNE    §25.6.1   read DR in the ISR to clear it
 *              NVIC (USART2_IRQn)          enable the vector
 *
 *          The ISR is deliberately tiny: read the byte, push it to the ring
 *          buffer, optionally call the user hook. Keeping ISRs short is a rule
 *          the book repeats — heavy work belongs in the main loop that drains
 *          the buffer. Overrun (ORE) is cleared by the SR-then-DR read sequence.
 ******************************************************************************
 */

#include "uart_driver_isr.h"
#include "gpio_driver.h" /* the Chapter 4 driver, reused for pin setup */
#include "main.h"
#include "ring_buffer.h"
#include <stddef.h>

/* Pin map on GPIOA for the USART2 VCP lines. */
#define UART_TX_PIN     (2U)    /* PA2 */
#define UART_RX_PIN     (3U)    /* PA3 */

/* Bounded spin budget so a missing/stuck peripheral cannot hang forever. */
#define UART_FLAG_BUDGET    (1000000UL)

/* RX queue + optional hook, shared between ISR and main loop. */
static ring_buffer_t s_rx_rb;
static uart_rx_callback_t s_rx_cb = NULL;

/* Bytes the hardware dropped because the previous one had not been read yet.
 * Counting them costs one increment in the ISR and turns a silent data loss
 * into something the application can print. */
static volatile uint32_t s_rx_overruns = 0U;

/* =========================================================================
 * uart2_init
 * ========================================================================= */
uart_status_t uart2_init(const uart_config_t *cfg)
{
    if ((cfg == NULL) || (cfg->baud == 0UL))
    {
        return UART_ERR_PARAM;
    }

    rb_init(&s_rx_rb);

    /* Peripheral clock: USART2 on APB1. gpio_init() enables the GPIOA gate
     * as part of configuring the pins.                                      */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    /* PA2 and PA3 to USART2 (AF7) through the Chapter 4 GPIO driver. */
    const gpio_config_t tx_cfg = {
        .pin = UART_TX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF7_USART2, .speed = GP_SPEED_LOW
    };
    const gpio_config_t rx_cfg = {
        .pin = UART_RX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_UP, .af = GP_AF7_USART2, .speed = GP_SPEED_LOW
    };
    gpio_init(GPIOA, &tx_cfg);
    gpio_init(GPIOA, &rx_cfg);

    /* Disable the peripheral while (re)configuring it. */
    USART2->CR1 = 0U;

    /* Baud rate: integer-rounded APB1 / baud into BRR (oversampling 16). */
    USART2->BRR = (APB1_CLOCK_HZ + (cfg->baud / 2U)) / cfg->baud;

    /* 8N1: M = 0 (8 data bits), PCE = 0 (no parity). CR2 stop bits = 00
     *    means 1 stop bit, which is the reset value, so CR2 is left clear.   */
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    /* Enable TX, RX, and the RXNE interrupt before switching the USART on. */
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);

    /* Enable the USART2 interrupt in the NVIC. Priority left at default; the
     * ISR is short and there are no nested-priority requirements yet.        */
    NVIC_EnableIRQ(USART2_IRQn);

    /* Enable the peripheral itself. */
    USART2->CR1 |= USART_CR1_UE;

    return UART_OK;
}

/* =========================================================================
 * uart2_write_byte
 * ========================================================================= */
uart_status_t uart2_write_byte(uint8_t byte)
{
    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART2->SR & USART_SR_TXE) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    USART2->DR = (uint32_t)byte & 0x1FFUL;  /* 9-bit field; we use 8 */
    return UART_OK;
}

/* =========================================================================
 * uart2_write
 * ========================================================================= */
uart_status_t uart2_write(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) && (len > 0UL))
    {
        return UART_ERR_PARAM;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        const uart_status_t st = uart2_write_byte(data[i]);
        if (st != UART_OK)
        {
            return st;
        }
    }

    /* Wait for the last byte to clear the shift register so the caller knows
     * the line is idle (important before, e.g., disabling the peripheral).  */
    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART2->SR & USART_SR_TC) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    return UART_OK;
}

/* =========================================================================
 * uart2_write_string
 * ========================================================================= */
uart_status_t uart2_write_string(const char *str)
{
    if (str == NULL)
    {
        return UART_ERR_PARAM;
    }
    uint32_t len = 0U;
    while (str[len] != '\0')
    {
        len++;
    }
    return uart2_write((const uint8_t *)str, len);
}

/* =========================================================================
 * uart2_getchar
 * ========================================================================= */
int uart2_getchar(uint8_t *out)
{
    return rb_get(&s_rx_rb, out);
}

/* =========================================================================
 * uart2_set_rx_callback
 * ========================================================================= */
void uart2_set_rx_callback(uart_rx_callback_t cb)
{
    s_rx_cb = cb;
}

/* =========================================================================
 * USART2_IRQHandler — vector defined in the CMSIS startup file.
 *
 * Kept minimal: on RXNE, read DR (which also clears RXNE and any ORE that the
 * preceding SR read latched), queue the byte, and optionally notify. No
 * printing, no blocking, no long work in interrupt context.
 * ========================================================================= */
void USART2_IRQHandler(void)
{
    /* Snapshot SR once. Reading it here is also the first half of the
     * read-SR-then-read-DR sequence that clears ORE.                        */
    const uint32_t sr = USART2->SR;

    if ((sr & USART_SR_RXNE) != 0U)
    {
        const uint8_t byte = (uint8_t)(USART2->DR & 0xFFUL);

        /* ORE means the hardware received a byte before we read the previous
         * one, so one byte was lost. Count it rather than discard the fact
         * silently: an application that sees this climbing knows the loss is
         * in the peripheral, not in the ring buffer.                        */
        if ((sr & USART_SR_ORE) != 0U)
        {
            s_rx_overruns++;
        }

        (void)rb_put(&s_rx_rb, byte);
        if (s_rx_cb != NULL)
        {
            s_rx_cb(byte);
        }
    }
}

/* =========================================================================
 * uart2_rx_overruns
 * ========================================================================= */
uint32_t uart2_rx_overruns(void)
{
    return s_rx_overruns;
}

/* =========================================================================
 * USART6 — header-native UART on PC6 (TX) / PC7 (RX), interrupt-driven RX.
 *
 * Mirrors the USART2 path with three differences: clock gate on APB2
 * (RCC_APB2ENR, USART6EN), baud divider from APB2_CLOCK_HZ, and PC6/PC7 on
 * alternate function AF8 in GPIOC AFR[0]. It has its own ring buffer and
 * callback so it is fully independent of USART2.
 * ========================================================================= */
#define U6_TX_PIN   (6U)    /* PC6 */
#define U6_RX_PIN   (7U)    /* PC7 */

static ring_buffer_t      s_rx_rb6;
static uart_rx_callback_t s_rx_cb6 = NULL;
static volatile uint32_t  s_rx_overruns6 = 0U;

uart_status_t usart6_init(const uart_config_t *cfg)
{
    if ((cfg == NULL) || (cfg->baud == 0UL))
    {
        return UART_ERR_PARAM;
    }

    rb_init(&s_rx_rb6);

    /* Peripheral clock: USART6 lives on APB2. gpio_init() enables GPIOC. */
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    (void)RCC->APB2ENR;

    /* PC6/PC7 to USART6 (AF8) through the same GPIO driver. */
    const gpio_config_t u6_tx = {
        .pin = U6_TX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF8_USART6, .speed = GP_SPEED_LOW
    };
    const gpio_config_t u6_rx = {
        .pin = U6_RX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_UP, .af = GP_AF8_USART6, .speed = GP_SPEED_LOW
    };
    gpio_init(GPIOC, &u6_tx);
    gpio_init(GPIOC, &u6_rx);

    USART6->CR1 = 0U;
    USART6->BRR = (APB2_CLOCK_HZ + (cfg->baud / 2U)) / cfg->baud;   /* APB2! */
    USART6->CR2 = 0U;
    USART6->CR3 = 0U;
    USART6->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
    NVIC_EnableIRQ(USART6_IRQn);
    USART6->CR1 |= USART_CR1_UE;
    return UART_OK;
}

uart_status_t usart6_write_byte(uint8_t byte)
{
    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART6->SR & USART_SR_TXE) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    USART6->DR = (uint32_t)byte & 0x1FFUL;
    return UART_OK;
}

uart_status_t usart6_write(const uint8_t *data, uint32_t len)
{
    if ((data == NULL) && (len > 0UL))
    {
        return UART_ERR_PARAM;
    }
    for (uint32_t i = 0U; i < len; i++)
    {
        const uart_status_t st = usart6_write_byte(data[i]);
        if (st != UART_OK)
        {
            return st;
        }
    }
    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART6->SR & USART_SR_TC) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    return UART_OK;
}

uart_status_t usart6_write_string(const char *str)
{
    if (str == NULL)
    {
        return UART_ERR_PARAM;
    }
    uint32_t len = 0U;
    while (str[len] != '\0')
    {
        len++;
    }
    return usart6_write((const uint8_t *)str, len);
}

int usart6_getchar(uint8_t *out)
{
    return rb_get(&s_rx_rb6, out);
}

void usart6_set_rx_callback(uart_rx_callback_t cb)
{
    s_rx_cb6 = cb;
}

/* =========================================================================
 * USART6_IRQHandler — vector defined in the CMSIS startup file.
 * ========================================================================= */
void USART6_IRQHandler(void)
{
    const uint32_t sr = USART6->SR;

    if ((sr & USART_SR_RXNE) != 0U)
    {
        const uint8_t byte = (uint8_t)(USART6->DR & 0xFFUL);
        if ((sr & USART_SR_ORE) != 0U)
        {
            s_rx_overruns6++;
        }
        (void)rb_put(&s_rx_rb6, byte);
        if (s_rx_cb6 != NULL)
        {
            s_rx_cb6(byte);
        }
    }
}

uint32_t usart6_rx_overruns(void)
{
    return s_rx_overruns6;
}
