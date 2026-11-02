/**
 ******************************************************************************
 * @file    spi_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.3: Register-level SPI2 master (implementation)
 *
 * @details Registers used (STM32F446RE Reference Manual RM0390, SPI chapter):
 *
 *              RCC_APB1ENR  §6.3.13   SPI2 clock gate (SPI2EN, bit 14)
 *              SPI_CR1      §26.7.1   BR, CPOL, CPHA, MSTR, SSM, SSI, SPE
 *              SPI_SR       §26.7.3   TXE, RXNE, BSY status flags
 *              SPI_DR       §26.7.4   data register (write=TX, read=RX)
 *
 *          Pin configuration (MODER, OTYPER, OSPEEDR, PUPDR, AFRH) is done
 *          through the Chapter 4 gpio_driver rather than inline here.
 *
 *          Software NSS: with SSM=1 and SSI=1 the peripheral treats its
 *          internal slave-select as high, which a single master needs to avoid
 *          a spurious mode fault. The *real* chip select is an ordinary GPIO
 *          (PB12) toggled around each transaction by the sensor layer.
 ******************************************************************************
 */

#include "spi_driver.h"
#include "gpio_driver.h"
#include <stddef.h>

#define SPI_SCK_PIN     (13U)
#define SPI_MISO_PIN    (14U)
#define SPI_MOSI_PIN    (15U)
#define SPI_CS_PIN      (12U)

/* Bounded spin budget, sized like the UART and I2C drivers so all three
 * behave the same way when a device is missing. */
#define SPI_FLAG_BUDGET (1000000UL)

spi_status_t spi2_init(const spi_config_t *cfg)
{
    spi_config_t local = { SPI_MODE0, 4U };   /* safe defaults: mode 0, /32 */
    if (cfg != NULL)
    {
        if (cfg->br > 7U)
        {
            return SPI_ERR_PARAM;
        }
        local = *cfg;
    }

    /* 1. Peripheral clock: SPI2 on APB1. gpio_init() enables the GPIOB gate
     *    when it configures the first pin on that port.                      */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;

    /* 2. Hand SCK, MISO and MOSI to SPI2 (AF5).
     *
     *    OSPEEDR is set here, and this is the first place in the book where it
     *    is a real decision. Output speed does not set a data rate; it sets how
     *    hard the pad drives the line, and therefore how fast each edge rises
     *    and falls. A slow edge on a fast clock eats into the receiver's setup
     *    window, which is why a bus is where the field starts to matter and a
     *    blinking LED is not.
     *
     *    The numbers make the choice concrete. Table 58 of the STM32F446
     *    datasheet (DS10693) rates the low setting at 4 MHz maximum with edges
     *    up to 100 ns, at VDD >= 2.7 V into a 50 pF load. At the 500 kHz used
     *    here a bit is 2 us wide, so a 100 ns edge costs 5 percent of it and
     *    the low setting would honestly be enough. It is set to
     *    GP_SPEED_MEDIUM anyway because the BME280 accepts a 10 MHz clock,
     *    where a bit is 100 ns and a 100 ns edge is the entire bit. The medium
     *    setting is rated at 25 MHz with 10 ns edges, which leaves room for the
     *    user who lowers the prescaler to see how fast the bus will go.
     *    The temptation to reach straight for GP_SPEED_HIGH is worth resisting:
     *    a harder driver means more ringing, more overshoot, and more radiated
     *    noise, especially on a breadboard with long jumper wires. The rule is
     *    the slowest setting that still gives a clean edge on the analyzer, not
     *    the fastest setting the pad offers.
     *
     *    MISO is an input as far as the MCU is concerned, so its speed field
     *    does nothing; it gets a pull-up so the line reads high rather than
     *    floating when no slave is driving it.                               */
    const gpio_config_t sck_cfg = {
        .pin = SPI_SCK_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF5_SPI2, .speed = GP_SPEED_MEDIUM
    };
    const gpio_config_t mosi_cfg = {
        .pin = SPI_MOSI_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF5_SPI2, .speed = GP_SPEED_MEDIUM
    };
    const gpio_config_t miso_cfg = {
        .pin = SPI_MISO_PIN, .mode = GP_MODE_ALT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_UP, .af = GP_AF5_SPI2, .speed = GP_SPEED_LOW
    };
    gpio_init(GPIOB, &sck_cfg);
    gpio_init(GPIOB, &mosi_cfg);
    gpio_init(GPIOB, &miso_cfg);

    /* 3. PB12 as a plain push-pull output for software chip select. Same
     *    driver, different mode — which is the point of having one.          */
    const gpio_config_t cs_cfg = {
        .pin = SPI_CS_PIN, .mode = GP_MODE_OUTPUT, .otype = GP_OTYPE_PUSHPULL,
        .pull = GP_PULL_NONE, .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    gpio_init(GPIOB, &cs_cfg);
    gpio_write_pin(GPIOB, SPI_CS_PIN, GP_HIGH);   /* idle deasserted */

    /* 4. Configure SPI2. Start from a clean CR1. */
    uint32_t cr1 = 0U;
    cr1 |= ((uint32_t)(local.br & 0x7U) << SPI_CR1_BR_Pos);   /* baud prescaler */
    if (((uint32_t)local.mode & 0x1U) != 0U) { cr1 |= SPI_CR1_CPHA; }
    if (((uint32_t)local.mode & 0x2U) != 0U) { cr1 |= SPI_CR1_CPOL; }
    cr1 |= SPI_CR1_MSTR;                 /* master mode                       */
    cr1 |= (SPI_CR1_SSM | SPI_CR1_SSI);  /* software slave management, NSS hi */
    /* DFF=0 (8-bit), LSBFIRST=0 (MSB first): both are the reset state.       */

    SPI2->CR1 = cr1;
    SPI2->CR2 = 0U;
    SPI2->CR1 |= SPI_CR1_SPE;            /* enable the peripheral             */

    return SPI_OK;
}

void spi2_cs_low(void)
{
    gpio_write_pin(GPIOB, SPI_CS_PIN, GP_LOW);    /* assert   */
}

void spi2_cs_high(void)
{
    gpio_write_pin(GPIOB, SPI_CS_PIN, GP_HIGH);   /* deassert */
}

spi_status_t spi2_transfer(uint8_t tx, uint8_t *rx)
{
    /* Wait for the TX buffer to be free, then load the byte. The budget is
     * what turns an absent or unpowered slave into a reported error rather
     * than a program that silently stops responding.                        */
    uint32_t budget = SPI_FLAG_BUDGET;
    while ((SPI2->SR & SPI_SR_TXE) == 0U)
    {
        if (--budget == 0UL)
        {
            return SPI_ERR_TIMEOUT;
        }
    }
    *(volatile uint8_t *)&SPI2->DR = tx;         /* 8-bit access for DFF=0 */

    /* Wait for the simultaneously-received byte, then read it. */
    budget = SPI_FLAG_BUDGET;
    while ((SPI2->SR & SPI_SR_RXNE) == 0U)
    {
        if (--budget == 0UL)
        {
            return SPI_ERR_TIMEOUT;
        }
    }

    const uint8_t in = *(volatile uint8_t *)&SPI2->DR;
    if (rx != NULL)
    {
        *rx = in;
    }
    return SPI_OK;
}

spi_status_t spi2_transfer_buf(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
    if ((tx == NULL) && (rx == NULL) && (len > 0UL))
    {
        return SPI_ERR_PARAM;   /* nothing to send and nowhere to put it */
    }

    for (uint32_t i = 0U; i < len; i++)
    {
        const uint8_t out = (tx != NULL) ? tx[i] : 0xFFU;
        uint8_t in = 0U;

        const spi_status_t st = spi2_transfer(out, &in);
        if (st != SPI_OK)
        {
            return st;
        }
        if (rx != NULL)
        {
            rx[i] = in;
        }
    }

    /* Ensure the last frame has fully shifted out before CS is released. */
    uint32_t budget = SPI_FLAG_BUDGET;
    while ((SPI2->SR & SPI_SR_BSY) != 0U)
    {
        if (--budget == 0UL)
        {
            return SPI_ERR_TIMEOUT;
        }
    }

    return SPI_OK;
}
