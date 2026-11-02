/**
 ******************************************************************************
 * @file    i2c_driver.c
 * @author  Renato Soriano
 * @brief   Chapter 5 — Exercise 5.4: Register-level I2C1 master (implementation)
 *
 * @details Registers used (STM32F446RE Reference Manual RM0390, I2C chapter):
 *
 *              RCC_APB1ENR  §6.3.13   I2C1 clock gate (I2C1EN, bit 21)
 *              I2C_CR1      §24.6.1   PE, START, STOP, ACK, SWRST
 *              I2C_CR2      §24.6.2   FREQ = APB1 clock in MHz
 *              I2C_CCR      §24.6.8   clock control (SCL period)
 *              I2C_TRISE    §24.6.9   max rise time + 1
 *              I2C_SR1/SR2  §24.6.6-7 SB, ADDR, TXE, RXNE, BTF, AF flags
 *              I2C_DR       §24.6.5   data register
 *
 *          Standard-mode 100 kHz from a 16 MHz APB1:
 *              FREQ  = 16
 *              CCR   = APB1 / (2 * 100 kHz) = 16e6 / 200e3 = 80
 *              TRISE = (1000 ns / (1/16 MHz)) + 1 = 16 + 1 = 17
 *
 *          The F4 I2C address-phase handshake (clear ADDR by reading SR1 then
 *          SR2) is fiddly, so the flag waits are wrapped with timeouts that
 *          return an explicit error instead of hanging the program.
 ******************************************************************************
 */

#include "i2c_driver.h"
#include "gpio_driver.h"   /* the Chapter 4 driver, reused for pin setup */
#include "main.h"
#include <stddef.h>

#define I2C_SCL_PIN     (8U)
#define I2C_SDA_PIN     (9U)
#define I2C_BUDGET      (1000000UL)

/* ---- bounded flag waits ------------------------------------------------ */

/* =========================================================================
 * wait_sr1_set — spin until an SR1 status flag is set, or fail.
 *
 * The I2C sequence is a series of "wait for this event, then act" steps, and
 * every event surfaces as a bit in status register 1 (SB, ADDR, TXE, RXNE,
 * BTF, ...). This helper centralises that wait so each call site stays a single
 * readable line, and it folds in the two things that must never be forgotten:
 *
 *   - NACK detection: if the slave does not acknowledge, hardware sets AF
 *     (acknowledge failure). We must clear AF (write 0) and report the error;
 *     otherwise the caller would spin forever waiting for an event that will
 *     never come.
 *   - A timeout: a missing, unpowered, or held-down device could stretch SCL
 *     indefinitely. The bounded budget turns that into I2C_ERR_TIMEOUT instead
 *     of a firmware lock-up.
 *
 * @param  mask  One (or more) SR1 flag bit(s) to wait for.
 * @return I2C_OK when the flag is set; I2C_ERR_NACK on AF; I2C_ERR_TIMEOUT if
 *         the budget expires first.
 * ========================================================================= */
static i2c_status_t wait_sr1_set(uint32_t mask)
{
    uint32_t budget = I2C_BUDGET;              /* loop guard, decremented below */
    while ((I2C1->SR1 & mask) == 0U)           /* not there yet — keep polling  */
    {
        /* Slave failed to ACK: clear AF (write-0-to-clear) and bail out. */
        if ((I2C1->SR1 & I2C_SR1_AF) != 0U)
        {
            I2C1->SR1 &= ~I2C_SR1_AF;
            return I2C_ERR_NACK;
        }
        /* Nothing after I2C_BUDGET iterations -> treat the bus as stuck. */
        if (--budget == 0UL)
        {
            return I2C_ERR_TIMEOUT;
        }
    }
    return I2C_OK;                              /* flag observed */
}

/* =========================================================================
 * wait_busy_clear — spin until the bus is idle, or time out.
 *
 * BUSY (in SR2) is set on a START and cleared on the matching STOP; it reflects
 * activity from ANY master on the bus, not just us. A new transfer must not
 * begin while BUSY is set, so each public entry point calls this first. The
 * same bounded budget protects against a line that is wedged low (for example
 * a mis-wired bus with no pull-ups), returning an error rather than hanging.
 * ========================================================================= */
static i2c_status_t wait_busy_clear(void)
{
    uint32_t budget = I2C_BUDGET;
    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U)    /* bus in use — wait it out */
    {
        if (--budget == 0UL)
        {
            return I2C_ERR_TIMEOUT;             /* stuck bus -> give up safely */
        }
    }
    return I2C_OK;                              /* bus is free */
}

/* Settle / bit-bang timing — crude busy-loops sized for the 16 MHz HSI core. */
#define I2C_SETTLE_LOOPS  (20000UL)   /* ~ a few ms post-enable bus settle */
#define I2C_BIT_LOOPS     (40UL)      /* ~ 10 us bit-bang half period       */

static void i2c_delay(volatile uint32_t loops)
{
    while (loops > 0UL) { loops--; }
}

/* =========================================================================
 * i2c1_bus_recover — free a hung bus BEFORE the pins are handed to the I2C
 * peripheral. If a slave was reset mid-byte it can hold SDA low indefinitely,
 * and a peripheral SWRST does NOT clock it out. Drive PB8 (SCL) as an
 * open-drain GPIO and issue up to nine clock pulses with SDA released, so the
 * stuck slave finishes its byte and lets go, then generate a manual STOP. This
 * also settles the lines high so the very first real START is clean.
 * ========================================================================= */
static void i2c1_bus_recover(void)
{
    /* PB8/PB9 as open-drain GPIO outputs with pull-ups, both released high.
     * Note that "released" and "driven high" are the same thing on an
     * open-drain pin: the transistor turns off and the pull-up does the rest,
     * which is exactly why a shared bus uses open-drain in the first place. */
    const gpio_config_t bitbang = {
        .pin = I2C_SCL_PIN, .mode = GP_MODE_OUTPUT,
        .otype = GP_OTYPE_OPENDRAIN, .pull = GP_PULL_UP,
        .af = GP_AF_NONE, .speed = GP_SPEED_LOW
    };
    gpio_config_t scl_cfg = bitbang;
    gpio_config_t sda_cfg = bitbang;
    sda_cfg.pin = I2C_SDA_PIN;

    gpio_init(GPIOB, &scl_cfg);
    gpio_init(GPIOB, &sda_cfg);
    gpio_write_pin(GPIOB, I2C_SCL_PIN, GP_HIGH);
    gpio_write_pin(GPIOB, I2C_SDA_PIN, GP_HIGH);
    i2c_delay(I2C_BIT_LOOPS);

    /* Up to nine SCL pulses; stop early once SDA has been released high. */
    for (uint32_t p = 0U; p < 9U; p++)
    {
        if (gpio_read_pin(GPIOB, I2C_SDA_PIN) == GP_HIGH) { break; }  /* freed */
        gpio_write_pin(GPIOB, I2C_SCL_PIN, GP_LOW);
        i2c_delay(I2C_BIT_LOOPS);
        gpio_write_pin(GPIOB, I2C_SCL_PIN, GP_HIGH);
        i2c_delay(I2C_BIT_LOOPS);
    }

    /* Manual STOP: SDA low while SCL is high, then SDA released high. */
    gpio_write_pin(GPIOB, I2C_SDA_PIN, GP_LOW);
    i2c_delay(I2C_BIT_LOOPS);
    gpio_write_pin(GPIOB, I2C_SCL_PIN, GP_HIGH);
    i2c_delay(I2C_BIT_LOOPS);
    gpio_write_pin(GPIOB, I2C_SDA_PIN, GP_HIGH);   /* -> STOP */
    i2c_delay(I2C_BIT_LOOPS);
}

void i2c1_init(void)
{
    /* 1. Peripheral clock: I2C1 on APB1. gpio_init() enables GPIOB. */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    /* 1b. Recover a possibly-hung bus BEFORE the pins become I2C signals. */
    i2c1_bus_recover();

    /* 2. PB8/PB9 -> alternate function AF4, open-drain, internal pull-up.
     *    This is the configuration that shows why the GPIO driver needed an
     *    otype field: I2C is the one bus in this chapter that cannot use a
     *    push-pull output, because two devices driving the same line in
     *    opposite directions would be a short circuit.
     *
     *    External 4.7k pull-ups are still required. The internal ones are tens
     *    of kilohms, which is far too weak to pull a loaded bus high inside
     *    the rise-time budget; they are enabled only so the line has a defined
     *    level before the externals are fitted.
     *
     *    Output speed stays LOW: an open-drain pin only ever drives its falling
     *    edge, and the rising edge is set by the pull-up resistor and the bus
     *    capacitance, not by OSPEEDR.                                        */
    const gpio_config_t i2c_pin = {
        .pin = I2C_SCL_PIN, .mode = GP_MODE_ALT,
        .otype = GP_OTYPE_OPENDRAIN, .pull = GP_PULL_UP,
        .af = GP_AF4_I2C1, .speed = GP_SPEED_LOW
    };
    gpio_config_t scl_af = i2c_pin;
    gpio_config_t sda_af = i2c_pin;
    sda_af.pin = I2C_SDA_PIN;

    gpio_init(GPIOB, &scl_af);
    gpio_init(GPIOB, &sda_af);

    /* 3. Reset the peripheral, then program timing while PE = 0. */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0U;

    const uint32_t apb1_mhz = APB1_CLOCK_HZ / 1000000UL;   /* = 16 */
    I2C1->CR2   = apb1_mhz;                                /* FREQ field */
    I2C1->CCR   = (uint16_t)(APB1_CLOCK_HZ / (2UL * 100000UL));  /* 80 -> 100 kHz Sm */
    I2C1->TRISE = (uint16_t)(apb1_mhz + 1U);              /* 17 */

    /* 4. Enable the peripheral, THEN enable ACK. The ACK bit is held cleared by
     *    hardware while PE = 0, so it only takes effect if set after PE.        */
    I2C1->CR1 |= I2C_CR1_PE;
    I2C1->CR1 |= I2C_CR1_ACK;

    /* 5. Let the bus and peripheral settle before the first START. This absorbs
     *    the unreliable first-transaction-after-enable behaviour that otherwise
     *    made a cold run fail until a bus scan "warmed up" the lines.           */
    i2c_delay(I2C_SETTLE_LOOPS);
}

/* =========================================================================
 * i2c1_write — master transmit: START, address+W, N data bytes, optional STOP.
 *
 * This walks the STM32F4 I2C "event" sequence from RM0390 (the events the
 * reference manual labels EV5, EV6, EV8, EV8_2). Every wait is bounded by a
 * timeout so a missing or stuck slave returns an error instead of hanging.
 * ========================================================================= */
i2c_status_t i2c1_write(uint8_t addr7, const uint8_t *data, uint32_t len, int stop)
{
    /* A NULL buffer is only acceptable when there is nothing to send. */
    if ((data == NULL) && (len > 0UL))
    {
        return I2C_ERR_PARAM;
    }

    /* Wait for the bus to be idle. BUSY (SR2) stays set from a START until the
     * matching STOP — ours or another master's. Starting while BUSY risks a
     * collision, so block (with timeout) until the bus is free.               */
    i2c_status_t st = wait_busy_clear();
    if (st != I2C_OK) { return st; }

    /* EV5 — START. Setting the START bit emits a start condition and puts the
     * peripheral into master mode; hardware then sets SB in SR1. SB is cleared
     * by the SR1 read inside wait_sr1_set() followed by the DR write below.    */
    I2C1->CR1 |= I2C_CR1_START;
    st = wait_sr1_set(I2C_SR1_SB);
    if (st != I2C_OK) { return st; }

    /* EV6 — address phase. Bits [7:1] carry the 7-bit address; bit 0 = 0 marks
     * a WRITE. Writing DR shifts the address out. The addressed slave must ACK;
     * if it does not, hardware sets AF and wait_sr1_set() returns I2C_ERR_NACK.
     * On success hardware sets ADDR when the address byte has been sent.       */
    I2C1->DR = (uint32_t)(addr7 << 1) | 0U;     /* write */
    st = wait_sr1_set(I2C_SR1_ADDR);
    if (st != I2C_OK) { I2C1->CR1 |= I2C_CR1_STOP; return st; }

    /* Clear ADDR with the mandatory read-SR1-then-read-SR2 sequence (the SR1
     * read already happened in the wait). While ADDR is set the peripheral
     * holds SCL low (clock stretch), so no data can move until it is cleared.  */
    (void)I2C1->SR2;                            /* clear ADDR (read SR1 then SR2) */

    /* EV8 — data. For each byte, wait for TXE (data register free), then write
     * it. Hardware shifts the previous byte onto the bus in the background
     * while the next one is loaded.                                           */
    for (uint32_t i = 0U; i < len; i++)
    {
        st = wait_sr1_set(I2C_SR1_TXE);
        if (st != I2C_OK) { I2C1->CR1 |= I2C_CR1_STOP; return st; }
        I2C1->DR = data[i];
    }

    /* EV8_2 — drain. BTF set together with TXE means both DR and the shift
     * register are empty: the final byte has physically left the MCU. Waiting
     * here guarantees we never truncate the last byte, and it leaves the bus in
     * a clean state to either STOP or (if stop == 0) issue a repeated START.   */
    st = wait_sr1_set(I2C_SR1_BTF);             /* last byte fully shifted */
    if (st != I2C_OK) { I2C1->CR1 |= I2C_CR1_STOP; return st; }

    /* STOP releases the bus (SDA rises while SCL is high). Skipping it holds
     * the bus so the caller can send a repeated START — exactly what
     * i2c1_mem_read() does to turn the bus around from write to read.          */
    if (stop != 0)
    {
        I2C1->CR1 |= I2C_CR1_STOP;
    }
    return I2C_OK;
}

/* =========================================================================
 * i2c1_read — master receive of N bytes, then STOP.
 *
 * Receiving on the STM32F4 I2C is trickier than transmitting because the
 * master must decide when to ACK (keep receiving) versus NACK (tell the slave
 * to stop) — and the NACK for the *final* byte has to be armed before that byte
 * arrives. RM0390 gives two different recipes: one for a single byte and one
 * for two-or-more bytes. Both are implemented below.
 * ========================================================================= */
/* Core receiver. wait_idle = 1 for a standalone read (START issued from an idle
 * bus); wait_idle = 0 for the read half of a repeated-start transfer
 * (i2c1_mem_read), where WE are deliberately holding the bus BUSY from the
 * preceding pointer write. Waiting for BUSY to clear in that case would time out
 * forever, because the only thing that clears BUSY is a STOP we never send. */
static i2c_status_t i2c1_read_core(uint8_t addr7, uint8_t *data, uint32_t len,
                                   int wait_idle)
{
    /* A read must have a destination and move at least one byte. */
    if ((data == NULL) || (len == 0UL))
    {
        return I2C_ERR_PARAM;
    }

    i2c_status_t st = I2C_OK;
    if (wait_idle != 0)
    {
        st = wait_busy_clear();
        if (st != I2C_OK) { return st; }
    }

    /* Enable ACK so the master acknowledges each received byte (that ACK is how
     * the slave is told to send the next one). We clear ACK later to NACK the
     * last byte. START then waits for SB, same EV5 as the write path.          */
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 &= ~I2C_CR1_POS;   /* POS is only used by the 2-byte case below */
    I2C1->CR1 |= I2C_CR1_START;
    st = wait_sr1_set(I2C_SR1_SB);
    if (st != I2C_OK) { return st; }

    /* EV6 — address phase with bit 0 = 1 (READ). The slave ACKs its address or
     * AF -> I2C_ERR_NACK. ADDR is set on success but is deliberately NOT cleared
     * yet: clearing ADDR starts the receiver clocking bytes in, so the precise
     * clear sequence differs between the 1-byte and N-byte cases below.        */
    I2C1->DR = (uint32_t)(addr7 << 1) | 1U;     /* read */
    st = wait_sr1_set(I2C_SR1_ADDR);
    if (st != I2C_OK) { I2C1->CR1 |= I2C_CR1_STOP; return st; }

    if (len == 1U)
    {
        /* Single-byte recipe (RM0390): the NACK and STOP must be set up BEFORE
         * ADDR is cleared, otherwise the master would ACK its only byte and
         * begin a second, unwanted read. Order matters:
         *   1) ACK = 0   -> the one incoming byte will be NACKed,
         *   2) read SR2  -> clears ADDR, so reception of that byte begins,
         *   3) STOP      -> queued now so it fires right after the byte.       */
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2;
        I2C1->CR1 |= I2C_CR1_STOP;

        /* Wait for the byte to land in DR (RXNE), then read it out. */
        st = wait_sr1_set(I2C_SR1_RXNE);
        if (st != I2C_OK) { return st; }
        data[0] = (uint8_t)I2C1->DR;
        return I2C_OK;
    }

    /* ---- N == 2: RM0390 two-byte recipe (requires the POS bit) -------------
     * The NACK for the SECOND byte has to be armed before EITHER byte is read.
     * Setting POS=1 with ACK=0 makes the (N)ACK apply to the byte currently in
     * the shift register, so byte 2 is NACKed and no third byte is clocked.    */
    if (len == 2U)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;              /* NACK ...                     */
        I2C1->CR1 |=  I2C_CR1_POS;              /* ... the second byte          */
        (void)I2C1->SR2;                        /* clear ADDR, reception begins */

        st = wait_sr1_set(I2C_SR1_BTF);         /* byte1 in DR, byte2 in shift  */
        if (st != I2C_OK)
        {
            I2C1->CR1 |= I2C_CR1_STOP;
            I2C1->CR1 &= ~I2C_CR1_POS;
            return st;
        }

        I2C1->CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)I2C1->DR;
        data[1] = (uint8_t)I2C1->DR;
        I2C1->CR1 &= ~I2C_CR1_POS;              /* restore for later transfers  */
        return I2C_OK;
    }

    /* ---- N >= 3: NACK exactly the last byte, clock no spare ----------------
     * Read normally until three bytes remain, then run the documented
     * BTF / ACK / STOP dance so the FINAL byte is NACKed and the slave is never
     * told to send an extra one. The previous version NACKed one byte too late,
     * clocked a spare byte, and left RXNE/DR dirty — which broke the NEXT read
     * (the "first read works, the rest error" symptom).                        */
    (void)I2C1->SR2;                            /* clear ADDR, ACK stays 1      */

    uint32_t i = 0U;
    while ((len - i) > 3U)                       /* everything but the last 3    */
    {
        st = wait_sr1_set(I2C_SR1_RXNE);
        if (st != I2C_OK) { return st; }
        data[i++] = (uint8_t)I2C1->DR;
    }

    /* Exactly three bytes remain: DataN-2 / DataN-1 / DataN. */
    st = wait_sr1_set(I2C_SR1_BTF);             /* DataN-2 in DR, DataN-1 in shift */
    if (st != I2C_OK) { return st; }
    I2C1->CR1 &= ~I2C_CR1_ACK;                  /* NACK the byte that follows (DataN) */
    data[i++] = (uint8_t)I2C1->DR;              /* read DataN-2                 */

    st = wait_sr1_set(I2C_SR1_BTF);             /* DataN-1 in DR, DataN in shift */
    if (st != I2C_OK) { return st; }
    I2C1->CR1 |= I2C_CR1_STOP;
    data[i++] = (uint8_t)I2C1->DR;              /* read DataN-1                 */

    st = wait_sr1_set(I2C_SR1_RXNE);            /* DataN now in DR              */
    if (st != I2C_OK) { return st; }
    data[i++] = (uint8_t)I2C1->DR;              /* read DataN                   */
    return I2C_OK;
}

/* Public standalone read: wait for an idle bus, then START + receive + STOP. */
i2c_status_t i2c1_read(uint8_t addr7, uint8_t *data, uint32_t len)
{
    return i2c1_read_core(addr7, data, len, 1);
}

/* Register read: write the slave's register pointer, then repeated-START read len bytes. */
i2c_status_t i2c1_mem_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint32_t len)
{
    /* Phase 1: write the register pointer WITHOUT a STOP. This deliberately
     * leaves the bus held (BUSY = 1) so phase 2 can issue a repeated START
     * without releasing the line to another master.                          */
    i2c_status_t st = i2c1_write(addr7, &reg, 1U, 0);
    if (st != I2C_OK)
    {
        return st;
    }
    /* Phase 2: repeated START + receive. Pass wait_idle = 0 — the bus is BUSY
     * by design here, so the receiver must NOT wait for it to clear.          */
    return i2c1_read_core(addr7, data, len, 0);
}
