// MIDI Synth — STM32F411 BlackPill
// USB MIDI IN → I2S DMA → PCM5102 DAC
// 4-voice polyphony, proper DDS synthesis, linear interpolation, soft clipping
// Lock-free: main loop pushes MIDI events, DMA ISR consumes them

#include "tusb.h"
#include "stm32f4xx.h"

#define BOARD_TUD_RHPORT 0

uint32_t SystemCoreClock = 84000000;

// --- Audio config ---
#define SAMPLE_RATE    48000
#define NUM_VOICES     4
#define DMA_BUF_SIZE   512
#define ENV_MAX        4096
#define ATTACK_INC     32

static int16_t dma_buf[DMA_BUF_SIZE];

// --- MIDI event ring buffer (main → ISR, lock-free SPSC) ---
#define MIDI_RING_SIZE 32
typedef struct {
    uint8_t type;
    uint8_t note;
    uint8_t vel;
} MidiEvent;

static volatile MidiEvent midi_ring[MIDI_RING_SIZE];
static volatile uint8_t midi_ring_head = 0;
static volatile uint8_t midi_ring_tail = 0;

static void midi_ring_push(uint8_t type, uint8_t note, uint8_t vel) {
    uint8_t next = (midi_ring_head + 1) % MIDI_RING_SIZE;
    if (next == midi_ring_tail) return;
    midi_ring[midi_ring_head].type = type;
    midi_ring[midi_ring_head].note = note;
    midi_ring[midi_ring_head].vel = vel;
    __DMB();
    midi_ring_head = next;
}

// --- Sine lookup table (256 samples, full scale) ---
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

// --- Proper DDS: pre-computed phase_inc for all 128 MIDI notes ---
// phase_inc = freq * 2^32 / SAMPLE_RATE
// Computed from base octave 0 values (exact for A0=13.75Hz), shifted for higher octaves
static uint32_t midi_phase_inc[128];

// Base phase_inc for notes 0-11 (C0 through B0)
// Computed as: freq * 4294967296 / 48000
static const uint32_t base_phase_inc_dds[12] = {
     731573,  //  C0 =  8.17580 Hz
     775065,  // C#0 =  8.66196 Hz
     821122,  //  D0 =  9.17702 Hz
     869944,  // D#0 =  9.72272 Hz
     921640,  //  E0 = 10.30086 Hz
     976438,  //  F0 = 10.91338 Hz
    1034493,  // F#0 = 11.56234 Hz
    1096022,  //  G0 = 12.24986 Hz
    1161166,  // G#0 = 12.97827 Hz
    1230329,  //  A0 = 13.75000 Hz
    1303463,  // A#0 = 14.56762 Hz
    1381008,  //  B0 = 15.43385 Hz
};

static void init_phase_table(void) {
    for (int note = 0; note < 128; note++) {
        int octave = note / 12;
        int semi = note % 12;
        midi_phase_inc[note] = base_phase_inc_dds[semi] << octave;
    }
}

typedef struct {
    uint8_t note;
    uint8_t velocity;
    uint32_t phase;
    uint32_t phase_inc;
    uint8_t active;
    uint16_t envelope;
    uint8_t releasing;
} Voice;

static Voice voices[NUM_VOICES];

static void voice_note_on(uint8_t note, uint8_t velocity) {
    int slot = -1;
    for (int i = 0; i < NUM_VOICES; i++) {
        if (!voices[i].active) { slot = i; break; }
    }
    if (slot < 0) {
        for (int i = 0; i < NUM_VOICES; i++) {
            if (voices[i].releasing) { slot = i; break; }
        }
    }
    if (slot < 0) slot = 0;

    voices[slot].note = note;
    voices[slot].velocity = velocity;
    voices[slot].phase = 0;
    voices[slot].phase_inc = midi_phase_inc[note];
    voices[slot].active = 1;
    voices[slot].envelope = 0;
    voices[slot].releasing = 0;
}

static void voice_note_off(uint8_t note) {
    for (int i = 0; i < NUM_VOICES; i++) {
        if (voices[i].active && voices[i].note == note) {
            voices[i].releasing = 1;
            break;
        }
    }
}

#define RELEASE_DECAY 255  // envelope *= 255/256 per sample (~20ms release)

static void process_midi_events(void) {
    while (midi_ring_tail != midi_ring_head) {
        MidiEvent ev = midi_ring[midi_ring_tail];
        __DMB();
        midi_ring_tail = (midi_ring_tail + 1) % MIDI_RING_SIZE;

        if (ev.type == 0x90 && ev.vel > 0) {
            voice_note_on(ev.note, ev.vel);
        } else {
            voice_note_off(ev.note);
        }
    }
}

// Soft clipping
static inline int32_t soft_clip(int32_t x) {
    if (x > 30000)
        return 30000 + ((x - 30000) >> 3);
    if (x < -30000)
        return -30000 + ((x + 30000) >> 3);
    return x;
}

static void fill_buffer(int16_t *buf, int len) {

    for (int s = 0; s < len; s += 2) {
        process_midi_events();
        int32_t mix = 0;

        for (int i = 0; i < NUM_VOICES; i++) {
            if (!voices[i].active) continue;

            // Envelope (0..ENV_MAX)
            if (!voices[i].releasing && voices[i].envelope < ENV_MAX) {
                voices[i].envelope += ATTACK_INC;
                if (voices[i].envelope > ENV_MAX) voices[i].envelope = ENV_MAX;
            }
            if (voices[i].releasing) {
                voices[i].envelope = (voices[i].envelope * RELEASE_DECAY) >> 8;
                if (voices[i].envelope < 4) {
                    voices[i].envelope = 0;
                    voices[i].active = 0;
                    continue;
                }
            }

            // DDS: full 32-bit phase accumulator
            voices[i].phase += voices[i].phase_inc;

            // Linear interpolation between sine table entries
            uint8_t idx = voices[i].phase >> 24;
            uint8_t frac = (voices[i].phase >> 16) & 0xFF;
            int32_t a = sine_table[idx];
            int32_t b = sine_table[(idx + 1) & 0xFF];
            int32_t sample = a + (((b - a) * frac) >> 8);

            // Scale by velocity and envelope
            sample = (sample * voices[i].velocity) >> 9;
            sample = (sample * voices[i].envelope) >> 12;
            mix += sample;
        }

        // 15% master volume
        mix = (mix * 19) >> 7;

        // Soft clipping
        mix = soft_clip(mix);

        // Final hard limit
        if (mix > 32767) mix = 32767;
        if (mix < -32767) mix = -32767;

        buf[s]     = (int16_t)mix;
        buf[s + 1] = (int16_t)mix;
    }
}

// --- DMA1 Stream4 ISR (SPI2_TX) ---
void DMA1_Stream4_IRQHandler(void) {
    if (DMA1->HISR & DMA_HISR_HTIF4) {
        DMA1->HIFCR = DMA_HIFCR_CHTIF4;
        fill_buffer(dma_buf, DMA_BUF_SIZE / 2);
    } else if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4;
        fill_buffer(dma_buf + DMA_BUF_SIZE / 2, DMA_BUF_SIZE / 2);
    } else {
        DMA1->HIFCR = DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;
    }
}

// --- Hardware init ---

static void init_clocks(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR = FLASH_ACR_LATENCY_2WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE
                 | (25 << RCC_PLLCFGR_PLLM_Pos)
                 | (336 << RCC_PLLCFGR_PLLN_Pos)
                 | (1 << RCC_PLLCFGR_PLLP_Pos)
                 | (7 << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR = RCC_CFGR_SW_PLL | RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

static void init_gpio(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER8) | (1 << GPIO_MODER_MODER8_Pos);
    GPIOA->BSRR = GPIO_BSRR_BR8;

    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12))
                 | (2 << GPIO_MODER_MODER11_Pos) | (2 << GPIO_MODER_MODER12_Pos);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xF << 12 | 0xF << 16))
                  | (10 << 12) | (10 << 16);
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12;

    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER13) | (1 << GPIO_MODER_MODER13_Pos);
    GPIOC->BSRR = GPIO_BSRR_BS13;

    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER13 | GPIO_MODER_MODER15))
                 | (2 << GPIO_MODER_MODER12_Pos) | (2 << GPIO_MODER_MODER13_Pos) | (2 << GPIO_MODER_MODER15_Pos);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xF << 16 | 0xF << 20 | 0xF << 28))
                  | (5 << 16) | (5 << 20) | (5 << 28);
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED15;
}

static void init_i2s_dma(void) {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // PLLI2S: 1MHz * 192 = 192MHz VCO, /5 = 38.4MHz I2SCLK
    RCC->PLLI2SCFGR = (192 << RCC_PLLI2SCFGR_PLLI2SN_Pos)
                    | (5 << RCC_PLLI2SCFGR_PLLI2SR_Pos);
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY));

    // I2S2: master TX, 16-bit channel, 16-bit data
    SPI2->I2SCFGR = SPI_I2SCFGR_I2SMOD | (2 << SPI_I2SCFGR_I2SCFG_Pos);
    // 38.4MHz / ((2*12+1) * 32) = 48000 Hz
    SPI2->I2SPR = 12 | SPI_I2SPR_ODD;

    for (int i = 0; i < DMA_BUF_SIZE; i++) dma_buf[i] = 0;

    DMA1_Stream4->CR = 0;
    while (DMA1_Stream4->CR & DMA_SxCR_EN);
    DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4
                | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4;

    DMA1_Stream4->PAR = (uint32_t)&SPI2->DR;
    DMA1_Stream4->M0AR = (uint32_t)dma_buf;
    DMA1_Stream4->NDTR = DMA_BUF_SIZE;
    DMA1_Stream4->FCR = 0;
    DMA1_Stream4->CR = (0 << DMA_SxCR_CHSEL_Pos)
                     | DMA_SxCR_DIR_0
                     | DMA_SxCR_MINC
                     | DMA_SxCR_CIRC
                     | (1 << DMA_SxCR_MSIZE_Pos)
                     | (1 << DMA_SxCR_PSIZE_Pos)
                     | (2 << DMA_SxCR_PL_Pos)
                     | DMA_SxCR_HTIE
                     | DMA_SxCR_TCIE;

    NVIC_SetPriority(DMA1_Stream4_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    DMA1_Stream4->CR |= DMA_SxCR_EN;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void OTG_FS_IRQHandler(void) {
    tud_int_handler(0);
}

int main(void) {
    for (int i = 0; i < NUM_VOICES; i++) {
        voices[i].active = 0;
        voices[i].envelope = 0;
        voices[i].releasing = 0;
        voices[i].phase = 0;
        voices[i].phase_inc = 0;
    }
    for (int i = 0; i < DMA_BUF_SIZE; i++) dma_buf[i] = 0;

    init_phase_table();
    init_clocks();
    init_gpio();
    init_i2s_dma();

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    NVIC_SetPriority(OTG_FS_IRQn, 2);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    GPIOC->BSRR = GPIO_BSRR_BR13;

    for (volatile int i = 0; i < 500000; i++);
    GPIOA->BSRR = GPIO_BSRR_BS8;

    while (1) {
        tud_task();

        if (tud_midi_mounted()) {
            uint8_t packet[4];
            while (tud_midi_available()) {
                if (tud_midi_packet_read(packet)) {
                    uint8_t msg = packet[1] & 0xF0;
                    uint8_t note = packet[2];
                    uint8_t vel = packet[3];

                    if (note > 127) continue;

                    if (msg == 0x90 && vel > 0) {
                        midi_ring_push(0x90, note, vel);
                    } else if (msg == 0x80 || (msg == 0x90 && vel == 0)) {
                        midi_ring_push(0x80, note, 0);
                    }
                }
            }
        }
    }
}

// --- Startup code ---
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;

void __attribute__((naked, noreturn)) Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;

    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    main();
    while (1);
}

void Default_Handler(void) { while (1); }
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

__attribute__((section(".isr_vector")))
const void *vector_table[98] = {
    [0]  = (void *)&_estack,
    [1]  = (void *)Reset_Handler,
    [2]  = (void *)NMI_Handler,
    [3]  = (void *)HardFault_Handler,
    [4]  = (void *)MemManage_Handler,
    [5]  = (void *)BusFault_Handler,
    [6]  = (void *)UsageFault_Handler,
    [11] = (void *)SVC_Handler,
    [14] = (void *)PendSV_Handler,
    [15] = (void *)SysTick_Handler,
    [31] = (void *)DMA1_Stream4_IRQHandler,
    [83] = (void *)OTG_FS_IRQHandler,
};
