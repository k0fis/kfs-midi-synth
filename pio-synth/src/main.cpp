// MIDI Synth — PlatformIO/Arduino STM32
// USB MIDI IN → I2S DMA audio out → PCM5102 DAC
// 4-voice polyphony, sine wave, attack/release envelope

#include <Arduino.h>
#include <USB-MIDI.h>
#include <stm32f4xx_hal_i2s.h>

USBMIDI_CREATE_DEFAULT_INSTANCE();

// --- Audio ---
#define SAMPLE_RATE    48000
#define AUDIO_BUF_SIZE 256  // DMA buffer (L+R interleaved)
#define NUM_VOICES     4

static int16_t audioBuf[2][AUDIO_BUF_SIZE]; // double buffer
static volatile uint8_t bufReady = 0;       // which buffer to fill

// I2S handle
static I2S_HandleTypeDef hi2s2;
static DMA_HandleTypeDef hdma_i2s2;

// Sine table
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

// Phase increment for MIDI notes (pre-computed)
// phase_inc = freq * 65536 / SAMPLE_RATE
static const uint32_t base_inc[12] = {
    // C0=8.18Hz: 8.18*65536/48000 = 11.17 → *256 = 2860
    2860, 3030, 3211, 3402, 3604, 3817,
    4044, 4284, 4539, 4809, 5094, 5397
};

typedef struct {
    uint8_t note;
    uint8_t velocity;
    uint32_t phase;
    uint32_t phase_inc;
    uint16_t envelope; // 0-256
    uint8_t active;
    uint8_t releasing;
} Voice;

static Voice voices[NUM_VOICES];

static uint32_t get_phase_inc(uint8_t note) {
    uint8_t octave = note / 12;
    uint8_t semi = note % 12;
    return (base_inc[semi] << octave) >> 8;
}

static void noteOn(uint8_t note, uint8_t vel) {
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
    voices[slot].velocity = vel;
    voices[slot].phase = 0;
    voices[slot].phase_inc = get_phase_inc(note);
    voices[slot].envelope = 0;
    voices[slot].active = 1;
    voices[slot].releasing = 0;
}

static void noteOff(uint8_t note) {
    for (int i = 0; i < NUM_VOICES; i++) {
        if (voices[i].active && voices[i].note == note) {
            voices[i].releasing = 1;
            break;
        }
    }
}

// Fill audio buffer (called from DMA interrupt context)
static void fillBuffer(int16_t *buf, int len) {
    for (int s = 0; s < len; s += 2) {
        int32_t mix = 0;
        for (int v = 0; v < NUM_VOICES; v++) {
            if (!voices[v].active) continue;

            // Envelope
            if (!voices[v].releasing && voices[v].envelope < 256) {
                voices[v].envelope += 2; // attack ~2.7ms
                if (voices[v].envelope > 256) voices[v].envelope = 256;
            }
            if (voices[v].releasing) {
                if (voices[v].envelope > 1) {
                    voices[v].envelope -= 1; // release ~5.3ms
                } else {
                    voices[v].envelope = 0;
                    voices[v].active = 0;
                    continue;
                }
            }

            voices[v].phase += voices[v].phase_inc;
            uint8_t idx = (voices[v].phase >> 8) & 0xFF;
            int32_t sample = sine_table[idx];
            sample = (sample * voices[v].velocity) >> 7;
            sample = (sample * voices[v].envelope) >> 8;
            mix += sample;
        }

        // Volume ~25%
        mix >>= 2;
        if (mix > 32767) mix = 32767;
        if (mix < -32768) mix = -32768;

        buf[s] = (int16_t)mix;     // L
        buf[s + 1] = (int16_t)mix; // R
    }
}

// --- DMA callbacks ---
extern "C" void DMA1_Stream4_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_i2s2);
}

extern "C" void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    fillBuffer(audioBuf[0], AUDIO_BUF_SIZE / 2);
}

extern "C" void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
    fillBuffer(audioBuf[0] + AUDIO_BUF_SIZE / 2, AUDIO_BUF_SIZE / 2);
}

// --- MIDI callbacks ---
void handleNoteOn(byte channel, byte note, byte velocity) {
    if (velocity > 0) noteOn(note, velocity);
    else noteOff(note);
}

void handleNoteOff(byte channel, byte note, byte velocity) {
    noteOff(note);
}

// --- I2S + DMA init ---
static void initI2S(void) {
    // Enable clocks
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PB12=WS, PB13=CK, PB15=SD (AF5)
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &gpio);

    // PLLI2S: assuming 25MHz HSE, PLLM=25 → input=1MHz
    // PLLI2SN=192, PLLI2SR=2 → 96 MHz I2S clock
    RCC->PLLI2SCFGR = (192 << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (2 << RCC_PLLI2SCFGR_PLLI2SR_Pos);
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY));

    // I2S config
    hi2s2.Instance = SPI2;
    hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
    hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s2.Init.CPOL = I2S_CPOL_LOW;
    hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
    HAL_I2S_Init(&hi2s2);

    // DMA config (DMA1 Stream4 Channel0 for SPI2_TX)
    hdma_i2s2.Instance = DMA1_Stream4;
    hdma_i2s2.Init.Channel = DMA_CHANNEL_0;
    hdma_i2s2.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2s2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2s2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2s2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_i2s2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_i2s2.Init.Mode = DMA_CIRCULAR;
    hdma_i2s2.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2s2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_i2s2);

    __HAL_LINKDMA(&hi2s2, hdmatx, hdma_i2s2);

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    // Start DMA circular transfer
    memset(audioBuf, 0, sizeof(audioBuf));
    HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t *)audioBuf[0], AUDIO_BUF_SIZE);
}

void setup() {
    // PA8 = XSMT unmute (start muted)
    pinMode(PA8, OUTPUT);
    digitalWrite(PA8, LOW);

    // LED
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW); // on

    // Init I2S with DMA
    initI2S();

    // Delay then unmute (pop suppression)
    delay(100);
    digitalWrite(PA8, HIGH);

    // USB MIDI
    MIDI.begin(MIDI_CHANNEL_OMNI);
    MIDI.setHandleNoteOn(handleNoteOn);
    MIDI.setHandleNoteOff(handleNoteOff);
}

void loop() {
    MIDI.read();
}
