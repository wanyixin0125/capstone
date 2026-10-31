/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Capstone firmware (stm#2) - bare-metal, no HAL
 *
 * Chain:  FSR/piezo sensors -> ADS8688 (external 16-bit ADC, SPI1)
 *         -> STM32F103C8T6 -> USART2 (PA2/PA3, 3.3V TTL) -> PC
 *
 * Pin map (per Nathan's spec):
 *   PA4 = ADS8688 CS        PA5 = SPI1 SCLK (<=17MHz, we use 9MHz)
 *   PA6 = SPI1 MISO         PA7 = SPI1 MOSI
 *   PA1 = ADS8688 RST/PD (active low)
 *   PB6 = DS18B20 data (4.7k pull-up to 3.3V, externally powered)
 *   PA2 = USART2 TX         PA3 = USART2 RX
 *   PA13/PA14 = SWD (keep free)
 *
 * ADS8688 register/command facts (from datasheet SBAS680):
 *   - RST command      = 0x8500 (resets program registers)
 *   - MAN_Ch_n command = 0xC000 | (n << 10)
 *   - Register write   = 16-bit frame: [15:9]=addr, [8]=1, [7:0]=data
 *   - Register read    = same but [8]=0, data clocked out after bit 16
 *   - CHn_RANGE reg    = 0x05 + n ; value 0x06 = 0..1.25*VREF = 0..5.12V
 *   - Pipeline: the conversion result is shifted out in the NEXT frame
 *     (result read in frame N belongs to the channel commanded in N-1).
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>

/* ================================================================== */
/* Register definitions (STM32F103C8T6)                                */
/* ================================================================== */

#define PERIPH_BASE      0x40000000UL
#define RCC_BASE         (PERIPH_BASE + 0x21000UL)
#define GPIOA_BASE       (PERIPH_BASE + 0x10800UL)
#define GPIOB_BASE       (PERIPH_BASE + 0x10C00UL)
#define USART2_BASE      (PERIPH_BASE + 0x4400UL)
#define SPI1_BASE        (PERIPH_BASE + 0x13000UL)
#define FLASH_BASE_R     0x40022000UL

typedef volatile uint32_t *reg32_t;

#define REG32(addr)      (*(reg32_t)(addr))

/* RCC */
#define RCC_CR           REG32(RCC_BASE + 0x00)
#define RCC_CFGR         REG32(RCC_BASE + 0x04)
#define RCC_APB2ENR      REG32(RCC_BASE + 0x18)
#define RCC_APB1ENR      REG32(RCC_BASE + 0x1C)

#define RCC_CR_HSEON     (1UL << 16)
#define RCC_CR_HSERDY    (1UL << 17)
#define RCC_CR_PLLON     (1UL << 24)
#define RCC_CR_PLLRDY    (1UL << 25)

#define RCC_CFGR_SW_PLL  (2UL << 0)
#define RCC_CFGR_SWS_PLL (2UL << 2)
#define RCC_CFGR_PPRE1_DIV2  (4UL << 8)   /* APB1 = HCLK/2 = 36 MHz */
#define RCC_CFGR_PLLSRC_HSE  (1UL << 16)
#define RCC_CFGR_PLLMUL9     (7UL << 18)  /* x9 -> 72 MHz */

#define RCC_APB2ENR_IOPA  (1UL << 2)
#define RCC_APB2ENR_IOPB  (1UL << 3)
#define RCC_APB2ENR_AFIO  (1UL << 0)
#define RCC_APB2ENR_SPI1  (1UL << 12)
#define RCC_APB1ENR_USART2 (1UL << 17)

/* Flash: latency */
#define FLASH_ACR        REG32(FLASH_BASE_R + 0x00)
#define FLASH_ACR_LAT2   (2UL << 0)
#define FLASH_ACR_PRFTBE (1UL << 4)

/* GPIO */
#define GPIOA_CRL        REG32(GPIOA_BASE + 0x00)
#define GPIOB_CRL        REG32(GPIOB_BASE + 0x00)
#define GPIOA_BSRR       REG32(GPIOA_BASE + 0x10)
#define GPIOB_BSRR       REG32(GPIOB_BASE + 0x10)
#define GPIOA_IDR        REG32(GPIOA_BASE + 0x08)
#define GPIOB_IDR        REG32(GPIOB_BASE + 0x08)

/* SPI1 */
#define SPI1_CR1         REG32(SPI1_BASE + 0x00)
#define SPI1_SR          REG32(SPI1_BASE + 0x08)
#define SPI1_DR          REG32(SPI1_BASE + 0x0C)

#define SPI1_CR1_SPE     (1UL << 6)
#define SPI1_CR1_MSTR    (1UL << 2)
#define SPI1_CR1_SSM     (1UL << 9)
#define SPI1_CR1_SSI     (1UL << 8)
#define SPI1_CR1_BR_DIV8 (4UL << 3)       /* 72/8 = 9 MHz <= 17 MHz limit */

#define SPI_SR_TXE       (1UL << 1)
#define SPI_SR_RXNE      (1UL << 0)
#define SPI_SR_BSY       (1UL << 7)

/* USART2 (APB1, 36 MHz) */
#define USART2_SR        REG32(USART2_BASE + 0x00)
#define USART2_DR        REG32(USART2_BASE + 0x04)
#define USART2_BRR       REG32(USART2_BASE + 0x08)
#define USART2_CR1       REG32(USART2_BASE + 0x0C)

#define USART_SR_TXE     (1UL << 7)
#define USART_CR1_UE     (1UL << 13)
#define USART_CR1_TE     (1UL << 3)
#define USART_CR1_RE     (1UL << 2)
/* 36e6 / 230400 = 156.25 -> mantissa 156, fraction 0.25*16 = 4 */
#define USART2_BRR_230400 (((156UL) << 4) | 4UL)

/* Core debug / DWT cycle counter for microsecond timing */
#define DEMCR            REG32(0xE000EDFCUL)
#define DWT_CTRL         REG32(0xE0001000UL)
#define DWT_CYCCNT       REG32(0xE0001004UL)
#define DEMCR_TRCENA     (1UL << 24)

/* SysTick: 1 ms tick, 32-bit ms counter (wraps after ~49.7 days;
 * extended by the PC side if needed - see Nathan's DR1 note) */
#define SYST_CSR         REG32(0xE000E010UL)
#define SYST_RVR         REG32(0xE000E014UL)
#define SYST_CVR         REG32(0xE000E018UL)

/* ================================================================== */
/* Pins (GPIOA / GPIOB bit numbers)                                    */
/* ================================================================== */

#define PIN_ADC_RST      1UL   /* PA1, active low */
#define PIN_ADC_CS       4UL   /* PA4, active low */
#define PIN_DS18B20      6UL   /* PB6 */

/* ================================================================== */
/* ADS8688 constants                                                   */
/* ================================================================== */

#define ADS8688_CMD_RST          0x8500UL
#define ADS8688_CMD_MAN_CH(n)    (0xC000UL | ((uint16_t)(n) << 10))
#define ADS8688_REG_AUTO_SEQ_EN  0x01UL
#define ADS8688_REG_CH_PWR_DOWN  0x02UL
#define ADS8688_REG_CH0_RANGE    0x05UL

#define ADS_RANGE_UNIP_1P25      0x06UL  /* 0..1.25*VREF = 0..5.12 V */

#define ADS_N_ACTIVE_CH          3UL     /* AIN0..AIN2 */

/* ================================================================== */
/* Data protocol (binary, little-endian bytes on the wire)             */
/*   [0]=0xAA [1]=0x55 [2:3]=seq(u16) [4:7]=t_ms(u32) [8]=type [9]=len */
/*   [10..]=payload [last]=checksum (u8 sum of bytes 2..last-1)        */
/*   type 0x01 = ADC frame, payload = N_ACTIVE_CH * u16 raw counts     */
/*   type 0x02 = temperature frame, payload = i16 (x16 degC)           */
/* ================================================================== */

#define PKT_TYPE_ADC     0x01
#define PKT_TYPE_TEMP    0x02

/* ================================================================== */
/* Low-level timing                                                    */
/* ================================================================== */

static volatile uint32_t g_ms = 0;
static uint32_t SystemCoreClockVal = 72000000UL;

void SysTick_Handler(void)
{
    g_ms++;
}

static void dwt_init(void)
{
    DEMCR |= DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1UL;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t ticks = us * (SystemCoreClockVal / 1000000UL);
    while ((DWT_CYCCNT - start) < ticks) {
        /* busy wait */
    }
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms;
    while ((g_ms - start) < ms) {
        /* busy wait */
    }
}

/* ================================================================== */
/* Clock: HSE 8 MHz -> PLL x9 -> 72 MHz, APB1 36 MHz                   */
/* ================================================================== */

static void clock_init(void)
{
    /* Start HSE */
    RCC_CR |= RCC_CR_HSEON;
    while (!(RCC_CR & RCC_CR_HSERDY)) {
        /* wait */
    }

    /* Flash latency for 72 MHz + prefetch */
    FLASH_ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LAT2;

    /* PLL: HSE x9 = 72 MHz, APB2 = 72 MHz, APB1 = 36 MHz */
    RCC_CFGR = RCC_CFGR_PLLMUL9 | RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PPRE1_DIV2;
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY)) {
        /* wait */
    }

    /* Switch system clock to PLL */
    RCC_CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC_CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL) {
        /* wait */
    }
}

/* ================================================================== */
/* GPIO / SPI1 / USART2 init                                           */
/* ================================================================== */

static void gpio_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPA | RCC_APB2ENR_IOPB | RCC_APB2ENR_AFIO;

    /* GPIOA CRL, one nibble per pin PA0..PA7:
     * PA0 in-float(4) PA1 out-PP(3) PA2 AF-PP(B) PA3 in-float(4)
     * PA4 out-PP(3) PA5 AF-PP(B) PA6 in-float(4) PA7 AF-PP(B)   */
    GPIOA_CRL = 0xB4B34B34UL;

    /* PB6: open-drain output 50 MHz (external 4.7k pull-up), idle high */
    GPIOB_CRL = (GPIOB_CRL & ~(0xFUL << 24)) | (0x7UL << 24);
    GPIOB_BSRR = (1UL << PIN_DS18B20);   /* release the bus high */

    /* Idle levels */
    GPIOA_BSRR = (1UL << PIN_ADC_CS);          /* CS high (inactive) */
    GPIOA_BSRR = (1UL << PIN_ADC_RST);         /* RST high (out of PD) */
}

static void spi1_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_SPI1;
    /* Mode 0 (CPOL=0, CPHA=0), MSB first, software NSS.
     * NOTE: if MISO data looks shifted/garbled on the bench, change to
     * mode 1 by setting CPHA (bit 0) in CR1 - one-line change. */
    SPI1_CR1 = SPI1_CR1_SPE | SPI1_CR1_MSTR | SPI1_CR1_SSM |
               SPI1_CR1_SSI | SPI1_CR1_BR_DIV8;
}

static inline void spi_cs_low(void)  { GPIOA_BSRR = (1UL << (PIN_ADC_CS + 16)); }
static inline void spi_cs_high(void) { GPIOA_BSRR = (1UL << PIN_ADC_CS); }

static uint8_t spi_xfer_byte(uint8_t tx)
{
    while (!(SPI1_SR & SPI_SR_TXE)) {
        /* wait */
    }
    SPI1_DR = tx;
    while (!(SPI1_SR & SPI_SR_RXNE)) {
        /* wait */
    }
    return (uint8_t)SPI1_DR;
}

static uint16_t spi_xfer16(uint16_t tx)
{
    uint16_t rx;
    spi_cs_low();
    rx  = (uint16_t)(spi_xfer_byte((uint8_t)(tx >> 8)) << 8);
    rx |= spi_xfer_byte((uint8_t)(tx & 0xFFUL));
    spi_cs_high();
    return rx;
}

/* ================================================================== */
/* USART2                                                              */
/* ================================================================== */

static void usart2_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_USART2;
    USART2_BRR = USART2_BRR_230400;
    USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart_send_buf(const uint8_t *p, uint32_t n)
{
    while (n--) {
        while (!(USART2_SR & USART_SR_TXE)) {
            /* wait */
        }
        USART2_DR = *p++;
    }
}

static void uart_puts(const char *s)
{
    while (*s) {
        while (!(USART2_SR & USART_SR_TXE)) {
            /* wait */
        }
        USART2_DR = (uint8_t)*s++;
    }
}

static void uart_put_hex16(uint16_t v)
{
    static const char hexd[] = "0123456789ABCDEF";
    char b[5];
    b[0] = hexd[(v >> 12) & 0xF];
    b[1] = hexd[(v >> 8) & 0xF];
    b[2] = hexd[(v >> 4) & 0xF];
    b[3] = hexd[v & 0xF];
    b[4] = '\0';
    uart_puts(b);
}

/* ================================================================== */
/* ADS8688 driver                                                      */
/* ================================================================== */

static void ads8688_rst_pulse(void)
{
    /* PA1 low >= 400 ns, then release; wait >= 15 ms for the internal
     * reference to recover before any configuration. */
    GPIOA_BSRR = (1UL << (PIN_ADC_RST + 16));   /* low  */
    delay_us(10);                               /* >> 400 ns */
    GPIOA_BSRR = (1UL << PIN_ADC_RST);          /* high */
    delay_ms(20);                               /* > 15 ms */
}

static void ads8688_reg_write(uint8_t addr, uint8_t val)
{
    /* Write frame: [15:9]=addr, [8]=1, [7:0]=data.
     * 24 SCLKs minimum are required -> send 16-bit frame + 1 dummy byte
     * so the device can clock out its write echo. */
    spi_cs_low();
    (void)spi_xfer_byte((uint8_t)(((addr & 0x7FUL) << 1) | 0x01UL));
    (void)spi_xfer_byte(val);
    (void)spi_xfer_byte(0x00);   /* clocks for the echo */
    spi_cs_high();
}

static uint8_t ads8688_reg_read(uint8_t addr)
{
    /* Read frame: [15:9]=addr, [8]=0, [7:0]=don't care.
     * Register data is clocked out from the 16th SCLK falling edge
     * onwards -> 16-bit frame + 1 dummy byte, data is the 3rd byte. */
    uint8_t data;
    spi_cs_low();
    (void)spi_xfer_byte((uint8_t)(((addr & 0x7FUL) << 1) | 0x00UL));
    (void)spi_xfer_byte(0x00);
    data = spi_xfer_byte(0x00);
    spi_cs_high();
    return data;
}

static uint16_t ads8688_read_channel(uint8_t ch)
{
    /* Manual mode with pipeline: the conversion result of the channel
     * commanded in the previous frame is shifted out during the current
     * frame. Sending the same MAN_Ch_n command in both frames makes the
     * second frame's result unambiguously belong to channel n. */
    (void)spi_xfer16(ADS8688_CMD_MAN_CH(ch));
    return spi_xfer16(ADS8688_CMD_MAN_CH(ch));
}

/* Returns 0 if all readbacks match, nonzero otherwise (bitmask). */
static uint32_t ads8688_init(void)
{
    uint32_t err = 0;
    uint8_t n;

    ads8688_rst_pulse();

    for (n = 0; n < ADS_N_ACTIVE_CH; n++) {
        ads8688_reg_write((uint8_t)(ADS8688_REG_CH0_RANGE + n),
                          ADS_RANGE_UNIP_1P25);
    }
    /* Enable ch0..2 in auto sequence; power down the unused inputs */
    ads8688_reg_write(ADS8688_REG_AUTO_SEQ_EN, 0x07);
    ads8688_reg_write(ADS8688_REG_CH_PWR_DOWN, 0xF8);

    /* Read back and verify (Nathan: "Read back the configuration.") */
    for (n = 0; n < ADS_N_ACTIVE_CH; n++) {
        uint8_t rb = ads8688_reg_read((uint8_t)(ADS8688_REG_CH0_RANGE + n));
        if (rb != ADS_RANGE_UNIP_1P25) {
            err |= (1UL << n);
        }
    }
    if (ads8688_reg_read(ADS8688_REG_AUTO_SEQ_EN) != 0x07) {
        err |= (1UL << 8);
    }
    return err;
}

/* ================================================================== */
/* DS18B20 (PB6, externally powered, 4.7k pull-up)                     */
/* Non-blocking usage: start conversion, then read after >= 750 ms.    */
/* ================================================================== */

#define DS_CMD_SKIP_ROM    0xCC
#define DS_CMD_CONVERT_T   0x44
#define DS_CMD_READ_SP     0xBE
#define DS_TEMP_INVALID    0x7FFF

static inline void ds_out_low(void)
{
    GPIOB_BSRR = (1UL << (PIN_DS18B20 + 16));
}

static inline void ds_release(void)
{
    GPIOB_BSRR = (1UL << PIN_DS18B20);
}

static inline uint32_t ds_read_pin(void)
{
    return (GPIOB_IDR >> PIN_DS18B20) & 1UL;
}

static uint8_t ds_reset(void)
{
    uint8_t presence;
    ds_out_low();
    delay_us(480);                 /* reset pulse >= 480 us */
    ds_release();
    delay_us(70);                  /* wait for presence pulse */
    presence = (uint8_t)(ds_read_pin() == 0U);
    delay_us(420);                 /* wait out the presence window */
    return presence;               /* 1 = a device answered */
}

static void ds_write_bit(uint8_t b)
{
    if (b) {
        ds_out_low();
        delay_us(6);
        ds_release();
        delay_us(64);
    } else {
        ds_out_low();
        delay_us(60);
        ds_release();
        delay_us(10);
    }
}

static uint8_t ds_read_bit(void)
{
    uint8_t b;
    ds_out_low();
    delay_us(3);
    ds_release();
    delay_us(10);
    b = (uint8_t)ds_read_pin();
    delay_us(50);
    return b;
}

static void ds_write_byte(uint8_t b)
{
    uint8_t i;
    for (i = 0; i < 8; i++) {
        ds_write_bit((uint8_t)(b & 1UL));
        b >>= 1;
    }
}

static uint8_t ds_read_byte(void)
{
    uint8_t i, b = 0;
    for (i = 0; i < 8; i++) {
        b >>= 1;
        if (ds_read_bit()) {
            b |= 0x80;
        }
    }
    return b;
}

static uint8_t ds_crc8(const uint8_t *p, uint32_t n)
{
    uint8_t crc = 0;
    uint32_t i;
    uint8_t bit;
    for (i = 0; i < n; i++) {
        uint8_t b = p[i];
        for (bit = 0; bit < 8; bit++) {
            uint8_t mix = (uint8_t)((crc ^ b) & 1UL);
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            b >>= 1;
        }
    }
    return crc;
}

/* Start one temperature conversion (non-blocking: the DS18B20 keeps
 * converting for up to 750 ms by itself; we come back later). */
static uint8_t ds_start_convert(void)
{
    if (!ds_reset()) {
        return 0;
    }
    ds_write_byte(DS_CMD_SKIP_ROM);
    ds_write_byte(DS_CMD_CONVERT_T);
    return 1;
}

/* Read scratchpad; returns temperature x16 degC, or DS_TEMP_INVALID. */
static int16_t ds_read_temp(void)
{
    uint8_t sp[9];
    uint32_t i;
    if (!ds_reset()) {
        return (int16_t)DS_TEMP_INVALID;
    }
    ds_write_byte(DS_CMD_SKIP_ROM);
    ds_write_byte(DS_CMD_READ_SP);
    for (i = 0; i < 9; i++) {
        sp[i] = ds_read_byte();
    }
    if (ds_crc8(sp, 8) != sp[8]) {
        return (int16_t)DS_TEMP_INVALID;
    }
    return (int16_t)((uint16_t)sp[0] | ((uint16_t)sp[1] << 8));
}

/* ================================================================== */
/* Packet builder                                                      */
/* ================================================================== */

static uint16_t g_seq = 0;

static void send_packet(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[32];
    uint8_t sum = 0;
    uint32_t i;
    uint32_t pos = 0;

    buf[pos++] = 0xAA;
    buf[pos++] = 0x55;
    buf[pos++] = (uint8_t)(g_seq & 0xFFU);
    buf[pos++] = (uint8_t)(g_seq >> 8);
    buf[pos++] = (uint8_t)(g_ms & 0xFFU);
    buf[pos++] = (uint8_t)((g_ms >> 8) & 0xFFU);
    buf[pos++] = (uint8_t)((g_ms >> 16) & 0xFFU);
    buf[pos++] = (uint8_t)((g_ms >> 24) & 0xFFU);
    buf[pos++] = type;
    buf[pos++] = len;
    for (i = 0; i < len; i++) {
        buf[pos++] = payload[i];
    }
    for (i = 2; i < pos; i++) {          /* checksum over everything else */
        sum = (uint8_t)(sum + buf[i]);
    }
    buf[pos++] = sum;

    uart_send_buf(buf, pos);
    g_seq++;
}

/* ================================================================== */
/* main                                                                */
/* ================================================================== */

int main(void)
{
    uint32_t next_sample_ms = 0;
    uint32_t next_ds_event_ms = 0;
    uint32_t ds_state = 0;       /* 0 = need to start conversion,
                                    1 = waiting to read result */
    uint32_t err;

    clock_init();
    dwt_init();
    gpio_init();
    usart2_init();

    /* SysTick: 1 ms @ 72 MHz */
    SYST_RVR = 71999UL;
    SYST_CVR = 0;
    SYST_CSR = 7UL;              /* ENABLE | TICKINT | CLKSOURCE */

    uart_puts("\r\n[capstone_fw] boot 72MHz, uart 230400\r\n");

    spi1_init();
    err = ads8688_init();

    if (err != 0UL) {
        uart_puts("[ads8688] config readback MISMATCH mask=0x");
        uart_put_hex16((uint16_t)err);
        uart_puts("\r\n");
    } else {
        uart_puts("[ads8688] init ok, ch0-2 range=0x06 (0-5.12V)\r\n");
    }

    for (;;) {
        uint32_t now = g_ms;

        /* --- ADC sampling: >= 200 SPS target; 1 kHz paced by SysTick --- */
        if ((int32_t)(now - next_sample_ms) >= 0) {
            uint8_t ch;
            uint8_t payload[8];

            next_sample_ms = now + 1;    /* 1 kHz frame rate */

            for (ch = 0; ch < ADS_N_ACTIVE_CH; ch++) {
                uint16_t raw = ads8688_read_channel(ch);
                payload[(uint32_t)ch * 2UL] = (uint8_t)(raw >> 8);
                payload[((uint32_t)ch * 2UL) + 1UL] = (uint8_t)(raw & 0xFFUL);
            }
            send_packet(PKT_TYPE_ADC, payload,
                        (uint8_t)(ADS_N_ACTIVE_CH * 2UL));
        }

        /* --- Temperature state machine (non-blocking) --- */
        if ((int32_t)(now - next_ds_event_ms) >= 0) {
            if (ds_state == 0UL) {
                if (ds_start_convert()) {
                    ds_state = 1;
                    next_ds_event_ms = now + 800;  /* > 750 ms conv time */
                } else {
                    next_ds_event_ms = now + 1000; /* retry in 1 s */
                }
            } else {
                int16_t t16 = ds_read_temp();
                uint8_t payload[2];
                payload[0] = (uint8_t)((uint16_t)t16 & 0xFFUL);
                payload[1] = (uint8_t)(((uint16_t)t16 >> 8) & 0xFFUL);
                send_packet(PKT_TYPE_TEMP, payload, 2);
                ds_state = 0;
                next_ds_event_ms = now + 1000;     /* 1 Hz temperature */
            }
        }
    }
}
