// STM32F411 I2S sine wave — volume control via USER button (PA0)
// Button cycles: 100% → 25% → 5% → mute → 100% ...
// PA8 = XSMT unmute, PC13 = LED (blinks = muted, solid = playing)
// I2S2: PB12(WS), PB13(CK), PB15(SD) → PCM5102 DAC

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef short int16_t;
typedef int int32_t;

#define RCC_BASE       0x40023800
#define GPIOA_BASE     0x40020000
#define GPIOB_BASE     0x40020400
#define GPIOC_BASE     0x40020800
#define SPI2_BASE      0x40003800

#define RCC_CR         (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_AHB1ENR    (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR    (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_PLLI2SCFGR (*(volatile uint32_t *)(RCC_BASE + 0x84))

#define GPIOA_MODER    (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_IDR      (*(volatile uint32_t *)(GPIOA_BASE + 0x10))
#define GPIOA_BSRR     (*(volatile uint32_t *)(GPIOA_BASE + 0x18))
#define GPIOA_PUPDR    (*(volatile uint32_t *)(GPIOA_BASE + 0x0C))

#define GPIOB_MODER    (*(volatile uint32_t *)(GPIOB_BASE + 0x00))
#define GPIOB_AFRH     (*(volatile uint32_t *)(GPIOB_BASE + 0x24))

#define GPIOC_MODER    (*(volatile uint32_t *)(GPIOC_BASE + 0x00))
#define GPIOC_BSRR     (*(volatile uint32_t *)(GPIOC_BASE + 0x18))

#define SPI2_SR        (*(volatile uint32_t *)(SPI2_BASE + 0x08))
#define SPI2_DR        (*(volatile uint32_t *)(SPI2_BASE + 0x0C))
#define SPI2_I2SCFGR   (*(volatile uint32_t *)(SPI2_BASE + 0x1C))
#define SPI2_I2SPR     (*(volatile uint32_t *)(SPI2_BASE + 0x20))

static const int16_t sine_table[256] = {
        0,   804,  1608,  2410,  3212,  4011,  4808,  5602,
     6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285,
    32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683,
    27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
    23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868,
    18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279,
    12539, 11793, 11039, 10278,  9512,  8739,  7962,  7179,
     6393,  5602,  4808,  4011,  3212,  2410,  1608,   804,
        0,  -804, -1608, -2410, -3212, -4011, -4808, -5602,
    -6393, -7179, -7962, -8739, -9512,-10278,-11039,-11793,
   -12539,-13279,-14010,-14732,-15446,-16151,-16846,-17530,
   -18204,-18868,-19519,-20159,-20787,-21403,-22005,-22594,
   -23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,
   -27245,-27683,-28105,-28510,-28898,-29268,-29621,-29956,
   -30273,-30571,-30852,-31113,-31356,-31580,-31785,-31971,
   -32137,-32285,-32412,-32521,-32609,-32678,-32728,-32757,
   -32767,-32757,-32728,-32678,-32609,-32521,-32412,-32285,
   -32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,
   -30273,-29956,-29621,-29268,-28898,-28510,-28105,-27683,
   -27245,-26790,-26319,-25832,-25329,-24811,-24279,-23731,
   -23170,-22594,-22005,-21403,-20787,-20159,-19519,-18868,
   -18204,-17530,-16846,-16151,-15446,-14732,-14010,-13279,
   -12539,-11793,-11039,-10278, -9512, -8739, -7962, -7179,
    -6393, -5602, -4808, -4011, -3212, -2410, -1608,  -804,
};

// Volume levels: 256=100%, 64=25%, 13=5%, 0=mute (shift right by 8)
static const uint16_t vol_levels[] = { 256, 64, 13, 0 };
#define NUM_LEVELS 4

static void delay(volatile uint32_t n) { while (n--) __asm__("nop"); }

static void init_gpio(void) {
    RCC_AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 2);
    delay(16);

    // PA0 = input with pull-up (USER button, active low)
    GPIOA_MODER &= ~(3u << 0);
    GPIOA_PUPDR = (GPIOA_PUPDR & ~(3u << 0)) | (1u << 0); // pull-up

    // PA8 = output (XSMT unmute)
    GPIOA_MODER = (GPIOA_MODER & ~(3u << 16)) | (1u << 16);
    GPIOA_BSRR = (1 << 8); // HIGH = unmute

    // PC13 = output (LED)
    GPIOC_MODER = (GPIOC_MODER & ~(3u << 26)) | (1u << 26);

    // PB12, PB13, PB15 = AF5 (I2S2)
    GPIOB_MODER = (GPIOB_MODER & ~((3u << 24) | (3u << 26) | (3u << 30)))
                | (2u << 24) | (2u << 26) | (2u << 30);
    GPIOB_AFRH = (GPIOB_AFRH & ~((0xFu << 16) | (0xFu << 20) | (0xFu << 28)))
               | (5u << 16) | (5u << 20) | (5u << 28);
}

static void init_i2s(void) {
    RCC_APB1ENR |= (1 << 14);
    delay(16);

    RCC_PLLI2SCFGR = (192u << 6) | (2u << 28);
    RCC_CR |= (1 << 26);
    while (!(RCC_CR & (1 << 27)));

    SPI2_I2SCFGR = (1u << 11) | (2u << 8);
    SPI2_I2SPR = (31u << 0) | (1u << 8);
    SPI2_I2SCFGR |= (1u << 10);
}

static inline void i2s_write(int16_t sample) {
    while (!(SPI2_SR & (1u << 1)));
    SPI2_DR = (uint16_t)sample;
}

int main(void) {
    init_gpio();

    // Blink 2x = startup
    for (int i = 0; i < 2; i++) {
        GPIOC_BSRR = (1u << (13 + 16));
        delay(300000);
        GPIOC_BSRR = (1u << 13);
        delay(300000);
    }

    init_i2s();

    uint32_t phase = 0;
    const uint32_t step = 606; // 440 Hz
    uint8_t vol_idx = 2;       // start at 5% (quiet!)
    uint8_t btn_was_pressed = 0;
    uint32_t debounce = 0;
    uint32_t led_counter = 0;

    // LED on = playing
    GPIOC_BSRR = (1u << (13 + 16));

    while (1) {
        uint8_t idx = (phase >> 8) & 0xFF;
        int32_t raw = sine_table[idx];
        int16_t sample = (int16_t)((raw * vol_levels[vol_idx]) >> 8);

        i2s_write(sample);
        i2s_write(sample);
        phase += step;

        // Button check (every ~1000 samples for debounce)
        if (++debounce >= 1000) {
            debounce = 0;
            uint8_t btn_now = !(GPIOA_IDR & 1); // active low

            if (btn_now && !btn_was_pressed) {
                vol_idx = (vol_idx + 1) % NUM_LEVELS;
            }
            btn_was_pressed = btn_now;

            // LED: solid = playing, blink = muted
            led_counter++;
            if (vol_levels[vol_idx] == 0) {
                if (led_counter & 8)
                    GPIOC_BSRR = (1u << (13 + 16));
                else
                    GPIOC_BSRR = (1u << 13);
            } else {
                GPIOC_BSRR = (1u << (13 + 16)); // LED on
            }
        }
    }
}

__attribute__((section(".isr_vector")))
const void *vector_table[] = {
    (void *)0x20020000,
    (void *)main,
};
