/**
 ******************************************************************************
 * @file    uart_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.2a: Register-level USART2 driver (polling)
 *
 * @details Registers used (STM32F446RE Reference Manual RM0390, USART chapter):
 *
 *              RCC_APB1ENR  §6.3.13   USART2 clock gate (USART2EN, bit 17)
 *              USART_BRR    §25.6.3   baud rate divider
 *              USART_CR1    §25.6.4   UE, TE, RE, word length, parity
 *              USART_SR     §25.6.1   TXE, TC, RXNE status flags
 *              USART_DR     §25.6.2   data register
 *
 *          Pin setup (MODER, OTYPER, OSPEEDR, PUPDR, AFRL) is delegated to the
 *          Chapter 4 gpio_driver rather than written inline here, so the same
 *          two lines of configuration serve the UART, SPI and I2C drivers.
 *
 *          Baud divider (oversampling by 16): USARTDIV = f_APB1 / baud.
 *          Writing the integer round of that ratio to BRR is accurate enough
 *          for standard baud rates; e.g. 16 MHz / 115200 = 138.9 -> 139 (0x8B),
 *          an error of ~0.08 %, far inside the UART's tolerance.
 ******************************************************************************
 */

#include "uart_driver.h"
#include "gpio_driver.h" /* the Chapter 4 driver, reused for pin setup */
#include "main.h"        /* APB1_CLOCK_HZ */
#include <stddef.h>      /* NULL */

/* Pin map on GPIOA for the USART2 VCP lines. */
#define UART_TX_PIN     (2U)    /* PA2 */
#define UART_RX_PIN     (3U)    /* PA3 */

/* Bounded spin budget so a missing/stuck peripheral cannot hang forever. */
#define UART_FLAG_BUDGET    (1000000UL)

/* =========================================================================
 * uart2_init
 * ========================================================================= */
uart_status_t uart2_init(const uart_config_t *cfg)
{
    if ((cfg == NULL) || (cfg->baud == 0UL))
    {
        return UART_ERR_PARAM;
    }

    /* 1. Peripheral clock: USART2 on APB1. The GPIOA gate is handled by
     *    gpio_init() below, which enables a port's clock as part of
     *    configuring a pin on it.                                           */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    /* 2. Hand PA2 and PA3 to USART2 through the Chapter 4 GPIO driver. Two
     *    struct fills replace six register read-modify-writes, and the pin
     *    setup now reads the same way in every driver in this chapter.
     *    Output speed is left at GP_SPEED_LOW: at 115200 baud each bit is
     *    8.68 us wide, which is roughly a thousand times slower than the
     *    edge the pad produces even on its slowest setting.                  */
    const gpio_config_t tx_cfg = {
        .pin = UART_TX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF7_USART2, .speed = GP_SPEED_LOW
    };
    const gpio_config_t rx_cfg = {
        /* A pull-up on RX keeps the line at the idle level while nothing is
         * driving it, so a disconnected cable does not look like a stream of
         * start bits and framing errors.                                     */
        .pin = UART_RX_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_UP, .af = GP_AF7_USART2, .speed = GP_SPEED_LOW
    };
    gpio_init(GPIOA, &tx_cfg);
    gpio_init(GPIOA, &rx_cfg);

    /* 4. Disable the peripheral while (re)configuring it. */
    USART2->CR1 = 0U;

    /* 5. Baud rate: integer-rounded APB1 / baud into BRR (oversampling 16). */
    USART2->BRR = (APB1_CLOCK_HZ + (cfg->baud / 2U)) / cfg->baud;

    /* 6. 8N1: M = 0 (8 data bits), PCE = 0 (no parity). CR2 stop bits = 00
     *    means 1 stop bit, which is the reset value, so CR2 is left clear.   */
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;

    /* 7. Enable transmitter and receiver, then the peripheral itself. */
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
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

    USART2->DR = (uint32_t)byte & 0x1FFUL;   /* 9-bit field; we use 8 */
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
 * uart2_read_byte
 * ========================================================================= */
uart_status_t uart2_read_byte(uint8_t *out)
{
    if (out == NULL)
    {
        return UART_ERR_PARAM;
    }

    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART2->SR & USART_SR_RXNE) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }

    /* Reading DR clears RXNE. Mask to 8 bits for our 8N1 frame. */
    *out = (uint8_t)(USART2->DR & 0xFFUL);
    return UART_OK;
}

/* =========================================================================
 * uart2_rx_ready
 * ========================================================================= */
int uart2_rx_ready(void)
{
    return ((USART2->SR & USART_SR_RXNE) != 0U) ? 1 : 0;
}

/* =========================================================================
 * USART6 — header-native UART on PC6 (TX) / PC7 (RX), for logic-analyzer use.
 *
 * Identical programming model to USART2, with three differences:
 *   - clock gate is on APB2 (RCC_APB2ENR, USART6EN) instead of APB1,
 *   - the baud divider uses APB2_CLOCK_HZ,
 *   - PC6/PC7 use alternate function AF8 (USART6) in GPIOC AFR[0].
 *
 * Registers used (RM0390): RCC_AHB1ENR (GPIOCEN), RCC_APB2ENR (USART6EN),
 * GPIOx_MODER/AFRL, USART_BRR, USART_CR1, USART_SR, USART_DR.
 * ========================================================================= */
#define U6_TX_PIN   (6U)    /* PC6 */
#define U6_RX_PIN   (7U)    /* PC7 */

uart_status_t usart6_init(const uart_config_t *cfg)
{
    if ((cfg == NULL) || (cfg->baud == 0UL))
    {
        return UART_ERR_PARAM;
    }

    /* Peripheral clock: USART6 lives on APB2, not APB1. gpio_init() enables
     * the GPIOC gate when it configures the pins.                           */
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
    USART6->CR1 |= (USART_CR1_TE | USART_CR1_RE);
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

uart_status_t usart6_read_byte(uint8_t *out)
{
    if (out == NULL)
    {
        return UART_ERR_PARAM;
    }
    uint32_t budget = UART_FLAG_BUDGET;
    while ((USART6->SR & USART_SR_RXNE) == 0U)
    {
        if (--budget == 0UL)
        {
            return UART_ERR_TIMEOUT;
        }
    }
    *out = (uint8_t)(USART6->DR & 0xFFUL);
    return UART_OK;
}

int usart6_rx_ready(void)
{
    return ((USART6->SR & USART_SR_RXNE) != 0U) ? 1 : 0;
}
