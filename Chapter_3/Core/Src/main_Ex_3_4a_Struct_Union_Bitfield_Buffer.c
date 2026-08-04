/**
 ******************************************************************************
 * @file    main_Ex_3_4a_Struct_Union_Bitfield_Buffer.c
 * @author  Renato Soriano
 * @brief   Chapter 3 — Exercise 3.4a: Structs, Unions, Bitfields on a buffer
 *
 * @target  STM32F446RE — Nucleo-F446RE (ARM Cortex-M4)
 * @output  SWV ITM Data Console (Stimulus Port 0)
 *
 * @details The struct overlay is demonstrated against a plain printable buffer
 *          rather than a peripheral struct.
 *
 *          Three language ideas covered side-by-side:
 *
 *          1a. Packed struct overlay on a `uint8_t` buffer.
 *             A small "wire-format" message header (8 bytes) is laid down
 *             into a buffer, then read back via a typed struct pointer. The
 *             `__attribute__((packed))` qualifier prevents the compiler from
 *             inserting padding bytes that would silently corrupt the on-
 *             the-wire layout — the same trick every UART/CAN protocol
 *             parser uses.
 *
 *          1b. The layout mismatch, demonstrated for real.
 *             Encoding and decoding through the same struct proves nothing
 *             about packing: whatever layout the compiler picks, both ends
 *             agree and the round trip succeeds. The interesting case is
 *             the one a protocol parser actually faces, where the bytes
 *             arrived from somewhere else and the local struct is the only
 *             thing that decides where each field begins. `wire_frame[]`
 *             below stands in for those received bytes, and it is decoded
 *             twice: once through the packed struct and once through the
 *             padded one. Only the packed decode recovers the fields.
 *
 *          2. Union for type-punning.
 *             A single union exposes the same 4 bytes as either a
 *             `uint32_t` word or four `uint8_t` bytes. This is the standard
 *             little-endian visibility trick used in protocol parsers and
 *             CRC code.
 *
 *          3. Bitfields for flag packing.
 *             A 1-byte status field is split into four bit-level flags so
 *             the reader can see how named bit positions replace shift/mask
 *             arithmetic when the layout is fixed.
 *
 *          Sidebar topic — `sizeof(struct)` surprises:
 *          Without `packed`, the same struct occupies 12 bytes instead of 8,
 *          because the compiler inserts two bytes of padding to land the
 *          `uint32_t` on a 4-byte boundary and two more at the tail to keep
 *          the struct itself 4-byte aligned in an array. The exercise prints
 *          both sizes so you see the RAM cost of forgetting the qualifier,
 *          and then decodes `wire_frame[]` both ways so you see the far
 *          worse cost: fields read from the wrong offsets, silently.
 ******************************************************************************
 */

#include "main.h"
#include "itm_debug.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/* -------------------------------------------------------------------------
 * Wire-format message header. 8 bytes when packed.
 *
 * Field layout (LSB first):
 *   offset 0  : msg_id     uint8
 *   offset 1  : version    uint8
 *   offset 2..5 : payload  uint32
 *   offset 6..7 : checksum uint16
 * ------------------------------------------------------------------------- */
typedef struct __attribute__((packed))
{
    uint8_t  msg_id;
    uint8_t  version;
    uint32_t payload;
    uint16_t checksum;
} message_header_t;

/* -------------------------------------------------------------------------
 * Non-packed counterpart — used only to print the size difference.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t  msg_id;
    uint8_t  version;
    uint32_t payload;
    uint16_t checksum;
} message_header_padded_t;

/* -------------------------------------------------------------------------
 * Union — same 4 bytes viewed two ways.
 * ------------------------------------------------------------------------- */
typedef union
{
    uint32_t as_word;
    uint8_t  as_bytes[4];
} word_view_t;

/* -------------------------------------------------------------------------
 * Bitfield-packed status flags. The whole thing fits in a single byte.
 * ------------------------------------------------------------------------- */
typedef struct
{
    uint8_t ready    : 1;
    uint8_t error    : 1;
    uint8_t busy     : 1;
    uint8_t mode     : 2;
    uint8_t reserved : 3;
} status_flags_t;

/* -------------------------------------------------------------------------
 * A frame "received from another device", written byte by byte exactly as it
 * would arrive on a UART or CAN link:
 *
 *   offset 0    A5           msg_id
 *   offset 1    01           version
 *   offset 2..5 EF BE AD DE  payload  = 0xDEADBEEF, little-endian
 *   offset 6..7 34 12        checksum = 0x1234, little-endian
 *
 * Twelve bytes of storage back an 8-byte frame. The four trailing zeros are
 * not part of the message; they exist so that decoding through the padded
 * struct, which believes the record is 12 bytes long, stays inside the
 * array. Reading past the end of an object is undefined behavior.
 *
 * The `aligned(4)` attribute guarantees the array starts on a word boundary,
 * which is what the padded struct expects of any object it is overlaid on.
 * ------------------------------------------------------------------------- */
static const uint8_t wire_frame[12] __attribute__((aligned(4))) =
{
    0xA5U, 0x01U,
    0xEFU, 0xBEU, 0xADU, 0xDEU,
    0x34U, 0x12U,
    0x00U, 0x00U, 0x00U, 0x00U
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void encode_message(uint8_t *buffer, uint8_t msg_id, uint8_t version,
                           uint32_t payload, uint16_t checksum);
static void decode_and_print(const uint8_t *buffer);
static void decode_wire_frame_both_ways(void);
static void demonstrate_union_view(uint32_t value);
static void demonstrate_bitfield_status(void);

/**
 * @brief  Encode a message header into a raw byte buffer by overlaying a
 *         packed-struct pointer on top of it.
 */
static void encode_message(uint8_t *buffer,
                           uint8_t  msg_id,
                           uint8_t  version,
                           uint32_t payload,
                           uint16_t checksum)
{
    message_header_t *hdr = (message_header_t *)buffer;
    hdr->msg_id   = msg_id;
    hdr->version  = version;
    hdr->payload  = payload;
    hdr->checksum = checksum;
}

/**
 * @brief  Decode the same buffer by overlaying a const-qualified packed
 *         struct pointer and printing each field.
 */
static void decode_and_print(const uint8_t *buffer)
{
    const message_header_t *hdr = (const message_header_t *)buffer;
    printf("Decoded message:\r\n");
    printf("  msg_id   = 0x%02X\r\n", hdr->msg_id);
    printf("  version  = 0x%02X\r\n", hdr->version);
    printf("  payload  = 0x%08lX\r\n", (unsigned long)hdr->payload);
    printf("  checksum = 0x%04X\r\n", hdr->checksum);
}

/**
 * @brief  Decode the same received frame through both struct layouts and
 *         print the two results side by side.
 *
 *         The packed struct places `payload` at offset 2, which is where the
 *         sender put it, so the field comes back as 0xDEADBEEF. The padded
 *         struct places `payload` at offset 4, because the compiler inserted
 *         two bytes of alignment padding after `version`, so it reads four
 *         bytes starting in the middle of the payload and returns a value
 *         that is wrong but perfectly plausible. `checksum` slides out to
 *         offset 8 and reads the trailing zeros.
 *
 *         Nothing warns you. No compiler diagnostic fires, no fault is
 *         raised, and the decode succeeds. That silence is the reason
 *         `packed` matters on any struct that describes bytes you did not
 *         lay out yourself.
 */
static void decode_wire_frame_both_ways(void)
{
    const message_header_t        *packed_view = (const message_header_t *)wire_frame;
    const message_header_padded_t *padded_view = (const message_header_padded_t *)wire_frame;

    printf("\r\nSame received frame, decoded through two layouts:\r\n");
    printf("  packed [8 B]  payload = 0x%08lX  checksum = 0x%04X\r\n",
           (unsigned long)packed_view->payload, packed_view->checksum);
    printf("  padded [12 B] payload = 0x%08lX  checksum = 0x%04X\r\n",
           (unsigned long)padded_view->payload, padded_view->checksum);
    printf("  the sender transmitted payload = 0xDEADBEEF, checksum = 0x1234\r\n");
}

/**
 * @brief  Show the same 4 bytes from two viewpoints via a union.
 *
 *         On a little-endian Cortex-M4, the lowest-address byte of
 *         as_bytes[] is the LSB of as_word. Printing both makes that
 *         endianness explicit.
 */
static void demonstrate_union_view(uint32_t value)
{
    word_view_t view;
    view.as_word = value;
    printf("\r\nUnion type-punning (little-endian view):\r\n");
    printf("  as_word    = 0x%08lX\r\n", (unsigned long)view.as_word);
    printf("  as_bytes[] = { 0x%02X, 0x%02X, 0x%02X, 0x%02X }\r\n",
           view.as_bytes[0], view.as_bytes[1],
           view.as_bytes[2], view.as_bytes[3]);
}

/**
 * @brief  Pack four flags into a single byte using bitfields.
 *
 *         Note how the assignment looks just like a struct field write — no
 *         shift/mask arithmetic is needed. The compiler does it for us, at
 *         the cost of some ABI portability (bitfield order is
 *         implementation-defined for cross-toolchain code).
 */
static void demonstrate_bitfield_status(void)
{
    status_flags_t flags = {0};
    flags.ready = 1U;
    flags.error = 0U;
    flags.busy  = 1U;
    flags.mode  = 2U;   /* 2 bits, range 0..3 */

    printf("\r\nBitfield status flags (sizeof = %u byte):\r\n",
           (unsigned int)sizeof(flags));
    printf("  ready=%u  error=%u  busy=%u  mode=%u\r\n",
           flags.ready, flags.error, flags.busy, flags.mode);
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

    uint8_t buffer[16] = {0};

    printf("\r\nChapter 3 - Exercise 3.4a: Structs, Unions, Bitfields on a buffer\r\n");
    printf("-----------------------------------------------------------------\r\n");

    /* Size comparison — what the sidebar promises -------------------- */
    printf("sizeof(message_header_t) [packed] = %u bytes\r\n",
           (unsigned int)sizeof(message_header_t));
    printf("sizeof(message_header_padded_t) [normal] = %u bytes\r\n",
           (unsigned int)sizeof(message_header_padded_t));

    /* Encode/decode via struct overlay ------------------------------- */
    encode_message(buffer, 0xA5U, 0x01U, 0xDEADBEEFUL, 0x1234U);

    printf("\r\nRaw buffer bytes (after encode):\r\n  ");
    for (uint32_t i = 0U; i < sizeof(message_header_t); i++)
    {
        printf("%02X ", buffer[i]);
    }
    printf("\r\n");

    decode_and_print(buffer);

    /* Same frame, two layouts — where packing actually earns its keep -- */
    decode_wire_frame_both_ways();

    /* Union view ----------------------------------------------------- */
    demonstrate_union_view(0xDEADBEEFUL);

    /* Bitfield status ----------------------------------------------- */
    demonstrate_bitfield_status();

    while (1)
    {
        /* Trap so the SWV console retains the log. */
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
