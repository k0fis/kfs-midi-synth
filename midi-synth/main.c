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
#define DMA_BUF_SIZE   256
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

static int32_t dc_x1 = 0;
static int32_t dc_y1 = 0;

static void midi_ring_push(uint8_t type, uint8_t note, uint8_t vel) {
    uint8_t next = (midi_ring_head + 1) % MIDI_RING_SIZE;
    if (next == midi_ring_tail) return;
    midi_ring[midi_ring_head].type = type;
    midi_ring[midi_ring_head].note = note;
    midi_ring[midi_ring_head].vel = vel;
    __DMB();
    midi_ring_head = next;
}

// --- Sine lookup table (1024 samples, full scale) ---
static const int16_t sine_table[1024] = {
         0,    201,    402,    603,    804,   1005,   1206,   1407,
      1608,   1809,   2009,   2210,   2410,   2611,   2811,   3012,
      3212,   3412,   3612,   3811,   4011,   4210,   4410,   4609,
      4808,   5007,   5205,   5404,   5602,   5800,   5998,   6195,
      6393,   6590,   6786,   6983,   7179,   7375,   7571,   7767,
      7962,   8157,   8351,   8545,   8739,   8933,   9126,   9319,
      9512,   9704,   9896,  10087,  10278,  10469,  10659,  10849,
     11039,  11228,  11417,  11605,  11793,  11980,  12167,  12353,
     12539,  12725,  12910,  13094,  13279,  13462,  13645,  13828,
     14010,  14191,  14372,  14553,  14732,  14912,  15090,  15269,
     15446,  15623,  15800,  15976,  16151,  16325,  16499,  16673,
     16846,  17018,  17189,  17360,  17530,  17700,  17869,  18037,
     18204,  18371,  18537,  18703,  18868,  19032,  19195,  19357,
     19519,  19680,  19841,  20000,  20159,  20317,  20475,  20631,
     20787,  20942,  21096,  21250,  21403,  21554,  21705,  21856,
     22005,  22154,  22301,  22448,  22594,  22739,  22884,  23027,
     23170,  23311,  23452,  23592,  23731,  23870,  24007,  24143,
     24279,  24413,  24547,  24680,  24811,  24942,  25072,  25201,
     25329,  25456,  25582,  25708,  25832,  25955,  26077,  26198,
     26319,  26438,  26556,  26674,  26790,  26905,  27019,  27133,
     27245,  27356,  27466,  27575,  27683,  27790,  27896,  28001,
     28105,  28208,  28310,  28411,  28510,  28609,  28706,  28803,
     28898,  28992,  29085,  29177,  29268,  29358,  29447,  29534,
     29621,  29706,  29791,  29874,  29956,  30037,  30117,  30195,
     30273,  30349,  30424,  30498,  30571,  30643,  30714,  30783,
     30852,  30919,  30985,  31050,  31113,  31176,  31237,  31297,
     31356,  31414,  31470,  31526,  31580,  31633,  31685,  31736,
     31785,  31833,  31880,  31926,  31971,  32014,  32057,  32098,
     32137,  32176,  32213,  32250,  32285,  32318,  32351,  32382,
     32412,  32441,  32469,  32495,  32521,  32545,  32567,  32589,
     32609,  32628,  32646,  32663,  32678,  32692,  32705,  32717,
     32728,  32737,  32745,  32752,  32757,  32761,  32765,  32766,
     32767,  32766,  32765,  32761,  32757,  32752,  32745,  32737,
     32728,  32717,  32705,  32692,  32678,  32663,  32646,  32628,
     32609,  32589,  32567,  32545,  32521,  32495,  32469,  32441,
     32412,  32382,  32351,  32318,  32285,  32250,  32213,  32176,
     32137,  32098,  32057,  32014,  31971,  31926,  31880,  31833,
     31785,  31736,  31685,  31633,  31580,  31526,  31470,  31414,
     31356,  31297,  31237,  31176,  31113,  31050,  30985,  30919,
     30852,  30783,  30714,  30643,  30571,  30498,  30424,  30349,
     30273,  30195,  30117,  30037,  29956,  29874,  29791,  29706,
     29621,  29534,  29447,  29358,  29268,  29177,  29085,  28992,
     28898,  28803,  28706,  28609,  28510,  28411,  28310,  28208,
     28105,  28001,  27896,  27790,  27683,  27575,  27466,  27356,
     27245,  27133,  27019,  26905,  26790,  26674,  26556,  26438,
     26319,  26198,  26077,  25955,  25832,  25708,  25582,  25456,
     25329,  25201,  25072,  24942,  24811,  24680,  24547,  24413,
     24279,  24143,  24007,  23870,  23731,  23592,  23452,  23311,
     23170,  23027,  22884,  22739,  22594,  22448,  22301,  22154,
     22005,  21856,  21705,  21554,  21403,  21250,  21096,  20942,
     20787,  20631,  20475,  20317,  20159,  20000,  19841,  19680,
     19519,  19357,  19195,  19032,  18868,  18703,  18537,  18371,
     18204,  18037,  17869,  17700,  17530,  17360,  17189,  17018,
     16846,  16673,  16499,  16325,  16151,  15976,  15800,  15623,
     15446,  15269,  15090,  14912,  14732,  14553,  14372,  14191,
     14010,  13828,  13645,  13462,  13279,  13094,  12910,  12725,
     12539,  12353,  12167,  11980,  11793,  11605,  11417,  11228,
     11039,  10849,  10659,  10469,  10278,  10087,   9896,   9704,
      9512,   9319,   9126,   8933,   8739,   8545,   8351,   8157,
      7962,   7767,   7571,   7375,   7179,   6983,   6786,   6590,
      6393,   6195,   5998,   5800,   5602,   5404,   5205,   5007,
      4808,   4609,   4410,   4210,   4011,   3811,   3612,   3412,
      3212,   3012,   2811,   2611,   2410,   2210,   2009,   1809,
      1608,   1407,   1206,   1005,    804,    603,    402,    201,
         0,   -201,   -402,   -603,   -804,  -1005,  -1206,  -1407,
     -1608,  -1809,  -2009,  -2210,  -2410,  -2611,  -2811,  -3012,
     -3212,  -3412,  -3612,  -3811,  -4011,  -4210,  -4410,  -4609,
     -4808,  -5007,  -5205,  -5404,  -5602,  -5800,  -5998,  -6195,
     -6393,  -6590,  -6786,  -6983,  -7179,  -7375,  -7571,  -7767,
     -7962,  -8157,  -8351,  -8545,  -8739,  -8933,  -9126,  -9319,
     -9512,  -9704,  -9896, -10087, -10278, -10469, -10659, -10849,
    -11039, -11228, -11417, -11605, -11793, -11980, -12167, -12353,
    -12539, -12725, -12910, -13094, -13279, -13462, -13645, -13828,
    -14010, -14191, -14372, -14553, -14732, -14912, -15090, -15269,
    -15446, -15623, -15800, -15976, -16151, -16325, -16499, -16673,
    -16846, -17018, -17189, -17360, -17530, -17700, -17869, -18037,
    -18204, -18371, -18537, -18703, -18868, -19032, -19195, -19357,
    -19519, -19680, -19841, -20000, -20159, -20317, -20475, -20631,
    -20787, -20942, -21096, -21250, -21403, -21554, -21705, -21856,
    -22005, -22154, -22301, -22448, -22594, -22739, -22884, -23027,
    -23170, -23311, -23452, -23592, -23731, -23870, -24007, -24143,
    -24279, -24413, -24547, -24680, -24811, -24942, -25072, -25201,
    -25329, -25456, -25582, -25708, -25832, -25955, -26077, -26198,
    -26319, -26438, -26556, -26674, -26790, -26905, -27019, -27133,
    -27245, -27356, -27466, -27575, -27683, -27790, -27896, -28001,
    -28105, -28208, -28310, -28411, -28510, -28609, -28706, -28803,
    -28898, -28992, -29085, -29177, -29268, -29358, -29447, -29534,
    -29621, -29706, -29791, -29874, -29956, -30037, -30117, -30195,
    -30273, -30349, -30424, -30498, -30571, -30643, -30714, -30783,
    -30852, -30919, -30985, -31050, -31113, -31176, -31237, -31297,
    -31356, -31414, -31470, -31526, -31580, -31633, -31685, -31736,
    -31785, -31833, -31880, -31926, -31971, -32014, -32057, -32098,
    -32137, -32176, -32213, -32250, -32285, -32318, -32351, -32382,
    -32412, -32441, -32469, -32495, -32521, -32545, -32567, -32589,
    -32609, -32628, -32646, -32663, -32678, -32692, -32705, -32717,
    -32728, -32737, -32745, -32752, -32757, -32761, -32765, -32766,
    -32767, -32766, -32765, -32761, -32757, -32752, -32745, -32737,
    -32728, -32717, -32705, -32692, -32678, -32663, -32646, -32628,
    -32609, -32589, -32567, -32545, -32521, -32495, -32469, -32441,
    -32412, -32382, -32351, -32318, -32285, -32250, -32213, -32176,
    -32137, -32098, -32057, -32014, -31971, -31926, -31880, -31833,
    -31785, -31736, -31685, -31633, -31580, -31526, -31470, -31414,
    -31356, -31297, -31237, -31176, -31113, -31050, -30985, -30919,
    -30852, -30783, -30714, -30643, -30571, -30498, -30424, -30349,
    -30273, -30195, -30117, -30037, -29956, -29874, -29791, -29706,
    -29621, -29534, -29447, -29358, -29268, -29177, -29085, -28992,
    -28898, -28803, -28706, -28609, -28510, -28411, -28310, -28208,
    -28105, -28001, -27896, -27790, -27683, -27575, -27466, -27356,
    -27245, -27133, -27019, -26905, -26790, -26674, -26556, -26438,
    -26319, -26198, -26077, -25955, -25832, -25708, -25582, -25456,
    -25329, -25201, -25072, -24942, -24811, -24680, -24547, -24413,
    -24279, -24143, -24007, -23870, -23731, -23592, -23452, -23311,
    -23170, -23027, -22884, -22739, -22594, -22448, -22301, -22154,
    -22005, -21856, -21705, -21554, -21403, -21250, -21096, -20942,
    -20787, -20631, -20475, -20317, -20159, -20000, -19841, -19680,
    -19519, -19357, -19195, -19032, -18868, -18703, -18537, -18371,
    -18204, -18037, -17869, -17700, -17530, -17360, -17189, -17018,
    -16846, -16673, -16499, -16325, -16151, -15976, -15800, -15623,
    -15446, -15269, -15090, -14912, -14732, -14553, -14372, -14191,
    -14010, -13828, -13645, -13462, -13279, -13094, -12910, -12725,
    -12539, -12353, -12167, -11980, -11793, -11605, -11417, -11228,
    -11039, -10849, -10659, -10469, -10278, -10087,  -9896,  -9704,
     -9512,  -9319,  -9126,  -8933,  -8739,  -8545,  -8351,  -8157,
     -7962,  -7767,  -7571,  -7375,  -7179,  -6983,  -6786,  -6590,
     -6393,  -6195,  -5998,  -5800,  -5602,  -5404,  -5205,  -5007,
     -4808,  -4609,  -4410,  -4210,  -4011,  -3811,  -3612,  -3412,
     -3212,  -3012,  -2811,  -2611,  -2410,  -2210,  -2009,  -1809,
     -1608,  -1407,  -1206,  -1005,   -804,   -603,   -402,   -201,
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
    uint16_t min_env = 0xFFFF;

    for (int i = 0; i < NUM_VOICES; i++) {
        if (voices[i].envelope < min_env) {
            min_env = voices[i].envelope;
            slot = i;
        }
    }

    voices[slot].note = note;
    voices[slot].velocity = velocity;
    //voices[slot].phase = 0;
    voices[slot].phase ^= 0x7FFFFFFF;
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
            uint16_t idx = (voices[i].phase >> 22) & 0x3FF;
            uint8_t frac = (voices[i].phase >> 14) & 0xFF;
            int32_t a = sine_table[idx];
            int32_t b = sine_table[(idx + 1) & 0x3FF];
            int32_t sample = a + (((b - a) * frac) >> 8);

            // Scale by velocity and envelope
            sample = (sample * voices[i].velocity) >> 9;
            sample = (sample * voices[i].envelope) >> 12;
            mix += sample;
        }

        // DC Blocker
        int32_t y = mix - dc_x1 + ((dc_y1 * 255) >> 8);
        dc_x1 = mix;
        dc_y1 = y;
        mix = y;

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
