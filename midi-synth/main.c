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
#define NUM_VOICES     6
#define DMA_BUF_SIZE   512
#define ENV_MAX        4096

// Waveform types
#define WF_SINE     0
#define WF_TRIANGLE 1
#define WF_SAW      2
#define WF_SQUARE   3

// Filter types
#define FILT_LP   0
#define FILT_HP   1
#define FILT_BP   2

// Envelope states
#define ENV_OFF     0
#define ENV_ATTACK  1
#define ENV_DECAY   2
#define ENV_SUSTAIN 3
#define ENV_RELEASE 4

static int16_t dma_buf[DMA_BUF_SIZE];
static volatile uint8_t current_waveform = WF_SINE;
static volatile uint8_t master_volume = 19;  // 0-127, default ~15% (19/128)

// ADSR parameters (shared by all voices)
static volatile uint16_t env_attack_rate = 25;    // ~3.4ms attack (CC ~10)
static volatile uint16_t env_decay_rate = 10;     // ~8.5ms decay (CC ~60)
static volatile uint16_t env_sustain_level = 3072; // ~75% of ENV_MAX
static volatile uint8_t env_release_mult = 253;   // multiplier /256

// Filter parameters (Chamberlin SVF)
static volatile uint16_t filter_cutoff = 32767;  // fully open (no filtering)
static volatile uint16_t filter_reso = 256;      // no resonance (flat)

// LFO parameters (global, shared by all voices)
static uint32_t lfo_phase = 0;
static volatile uint32_t lfo_rate = 379217;      // ~4 Hz default
static volatile uint8_t lfo_depth = 0;           // off until user moves slider
static volatile uint8_t lfo_target = 0;          // 0=vibrato, 1=tremolo

// Detune (spreads voices apart for fuller sound)
static volatile uint8_t detune_amount = 0;       // 0-127, CC #94

// Portamento (glide between notes)
static volatile uint8_t porta_rate = 0;          // 0=off, 127=slow glide, CC #5

// Filter type (LP/HP/BP)
static volatile uint8_t filter_type = FILT_LP;   // CC #79

// Filter envelope amount
static volatile uint8_t filter_env_amount = 0;   // CC #80, 0=off, 127=max modulation

// Sub-oscillator level (one octave below)
static volatile uint8_t sub_osc_level = 0;       // CC #81, 0=off, 127=full

// Pulse width (for square wave)
static volatile uint8_t pulse_width = 64;        // CC #82, 64=50% duty, 0=thin, 127=thin

// Noise mix level
static volatile uint8_t noise_level = 0;         // CC #83, 0=off, 127=full

// Overdrive/distortion amount
static volatile uint8_t drive_amount = 0;        // CC #84, 0=clean, 127=heavy distortion

// Waveform morph (continuous blend between waveforms)
static volatile uint8_t morph_enabled = 0;       // CC #85 >= 64 = on
static volatile uint8_t morph_position = 0;      // CC #86, 0=sine, 42=tri, 85=saw, 127=square

// Stereo spread (pan voices across L/R)
static volatile uint8_t stereo_spread = 64;      // CC #87, 0=mono, 127=full stereo

// Key tracking (filter follows note pitch)
static volatile uint8_t key_track = 0;           // CC #88, 0=off, 127=full tracking

// Chorus (short modulated delay)
#define CHORUS_BUF_SIZE 2048
static int16_t chorus_buf[CHORUS_BUF_SIZE];
static uint16_t chorus_write_pos = 0;
static uint32_t chorus_lfo_phase = 0;
static volatile uint8_t chorus_depth = 0;        // CC #89, 0=off, 127=deep
static volatile uint8_t chorus_rate = 40;        // CC #90, speed of modulation

// Pitch bend (±2 semitones, 14-bit)
static volatile int16_t pitch_bend = 0;          // -8192..+8191 from MIDI

// Forward declarations
static void midi_ring_push(uint8_t type, uint8_t note, uint8_t vel);
static volatile uint8_t send_state_flag = 0;

// --- Flash presets (sector 7: 0x08060000) ---
#define PRESET_FLASH_ADDR  0x08060000
#define PRESET_MAGIC       0x4B465350  // "KFSP"
#define NUM_PRESETS        16
#define NUM_CC_PARAMS      22

typedef struct {
    uint32_t magic;
    uint8_t params[NUM_CC_PARAMS];  // CC values in fixed order
    uint8_t padding[2];             // align to 28 bytes
} FlashPreset;

// CC numbers in storage order
static const uint8_t preset_cc_map[NUM_CC_PARAMS] = {
    1, 7, 73, 75, 70, 72, 74, 71, 76, 77, 78,
    94, 5, 79, 80, 81, 82, 83, 84, 85, 86, 87
};

static uint8_t get_cc_value(uint8_t cc) {
    switch (cc) {
    case 1:  return current_waveform == WF_SINE ? 0 : current_waveform == WF_TRIANGLE ? 42 : current_waveform == WF_SAW ? 64 : 127;
    case 7:  return master_volume;
    case 73: return 127 - (env_attack_rate - 2) * 5;
    case 75: return 127 - (env_decay_rate - 2) * 5;
    case 70: return env_sustain_level / 32;
    case 72: return (env_release_mult - 200) * 127 / 55;
    case 74: return filter_cutoff >= 32767 ? 127 : (filter_cutoff - 200) / 187;
    case 71: return (256 - filter_reso) / 2;
    case 76: return (lfo_rate - 44739) / 6700;
    case 77: return lfo_depth;
    case 78: return lfo_target ? 127 : 0;
    case 94: return detune_amount;
    case 5:  return porta_rate;
    case 79: return filter_type == FILT_LP ? 0 : filter_type == FILT_HP ? 64 : 127;
    case 80: return filter_env_amount;
    case 81: return sub_osc_level;
    case 82: return pulse_width;
    case 83: return noise_level;
    case 84: return drive_amount;
    case 85: return morph_enabled ? 127 : 0;
    case 86: return morph_position;
    case 87: return stereo_spread;
    default: return 0;
    }
}

static void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}

static void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_erase_sector7(void) {
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR = FLASH_CR_SER | (7 << FLASH_CR_SNB_Pos) | (2 << FLASH_CR_PSIZE_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR = 0;
}

static void flash_write_word(uint32_t addr, uint32_t data) {
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR = FLASH_CR_PG | (2 << FLASH_CR_PSIZE_Pos);
    *(volatile uint32_t *)addr = data;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR = 0;
}

static void save_preset(uint8_t slot) {
    if (slot >= NUM_PRESETS) return;

    // Read all existing presets into RAM
    FlashPreset all[NUM_PRESETS];
    const FlashPreset *flash_presets = (const FlashPreset *)PRESET_FLASH_ADDR;
    for (int i = 0; i < NUM_PRESETS; i++) {
        all[i] = flash_presets[i];
    }

    // Update the target slot
    all[slot].magic = PRESET_MAGIC;
    for (int i = 0; i < NUM_CC_PARAMS; i++) {
        all[slot].params[i] = get_cc_value(preset_cc_map[i]);
    }

    // Erase and rewrite all
    flash_unlock();
    flash_erase_sector7();
    uint32_t *src = (uint32_t *)all;
    uint32_t dest = PRESET_FLASH_ADDR;
    uint32_t words = (NUM_PRESETS * sizeof(FlashPreset)) / 4;
    for (uint32_t i = 0; i < words; i++) {
        flash_write_word(dest, src[i]);
        dest += 4;
    }
    flash_lock();
}

static void load_preset(uint8_t slot) {
    if (slot >= NUM_PRESETS) return;
    const FlashPreset *p = &((const FlashPreset *)PRESET_FLASH_ADDR)[slot];
    if (p->magic != PRESET_MAGIC) return;  // empty slot

    // Push all CC values through the ring buffer
    for (int i = 0; i < NUM_CC_PARAMS; i++) {
        midi_ring_push(0xB0, preset_cc_map[i], p->params[i]);
    }
    // Trigger state sync to update host UI
    send_state_flag = 1;
}

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

// Noise LFSR (32-bit, xorshift)
static uint32_t noise_lfsr = 0xDEADBEEF;

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
    uint32_t target_phase_inc;
    uint8_t active;
    uint16_t envelope;
    uint8_t env_state;
    uint16_t filt_envelope;
    uint8_t filt_env_state;
    int32_t filt_low;
    int32_t filt_band;
} Voice;

static Voice voices[NUM_VOICES];

static void voice_note_on(uint8_t note, uint8_t velocity) {
    int slot = -1;
    for (int i = 0; i < NUM_VOICES; i++) {
        if (!voices[i].active) { slot = i; break; }
    }
    if (slot < 0) {
        for (int i = 0; i < NUM_VOICES; i++) {
            if (voices[i].env_state == ENV_RELEASE) { slot = i; break; }
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
    voices[slot].target_phase_inc = midi_phase_inc[note];
    if (porta_rate == 0 || !voices[slot].active) {
        voices[slot].phase_inc = midi_phase_inc[note];
    }
    voices[slot].phase ^= 0x7FFFFFFF;
    voices[slot].active = 1;
    voices[slot].envelope = 0;
    voices[slot].env_state = ENV_ATTACK;
    voices[slot].filt_envelope = 0;
    voices[slot].filt_env_state = ENV_ATTACK;
    voices[slot].filt_low = 0;
    voices[slot].filt_band = 0;
}

static void voice_note_off(uint8_t note) {
    for (int i = 0; i < NUM_VOICES; i++) {
        if (voices[i].active && voices[i].note == note) {
            voices[i].env_state = ENV_RELEASE;
            voices[i].filt_env_state = ENV_RELEASE;
            break;
        }
    }
}

static void all_notes_off(void);

static void process_midi_events(void) {
    while (midi_ring_tail != midi_ring_head) {
        MidiEvent ev = midi_ring[midi_ring_tail];
        __DMB();
        midi_ring_tail = (midi_ring_tail + 1) % MIDI_RING_SIZE;

        if (ev.type == 0xB0) {
            if (ev.note == 0 && ev.vel < NUM_PRESETS) {
                save_preset(ev.vel);
            } else if (ev.note == 1) {
                if (ev.vel < 32) current_waveform = WF_SINE;
                else if (ev.vel < 64) current_waveform = WF_TRIANGLE;
                else if (ev.vel < 96) current_waveform = WF_SAW;
                else current_waveform = WF_SQUARE;
            } else if (ev.note == 7) {
                master_volume = ev.vel;
            } else if (ev.note == 73) {
                env_attack_rate = 2 + (127 - ev.vel) / 5;
            } else if (ev.note == 75) {
                env_decay_rate = 2 + (127 - ev.vel) / 5;
            } else if (ev.note == 70) {
                env_sustain_level = (uint16_t)ev.vel * 32;
            } else if (ev.note == 72) {
                env_release_mult = 200 + (ev.vel * 55 / 127);
            } else if (ev.note == 74) {
                if (ev.vel >= 127)
                    filter_cutoff = 32767;  // bypass
                else
                    filter_cutoff = 200 + (uint16_t)ev.vel * 187;  // max ~24000
            } else if (ev.note == 71) {
                uint16_t q = 256 - ev.vel * 2;
                filter_reso = q < 4 ? 4 : q;
            } else if (ev.note == 123 || ev.note == 120) {
                all_notes_off();
            } else if (ev.note == 76) {
                lfo_rate = 44739 + (uint32_t)ev.vel * 6700;
            } else if (ev.note == 77) {
                lfo_depth = ev.vel;
            } else if (ev.note == 78) {
                lfo_target = ev.vel >= 64 ? 1 : 0;
            } else if (ev.note == 94) {
                detune_amount = ev.vel;
            } else if (ev.note == 5) {
                porta_rate = ev.vel;
            } else if (ev.note == 79) {
                if (ev.vel < 43) filter_type = FILT_LP;
                else if (ev.vel < 85) filter_type = FILT_HP;
                else filter_type = FILT_BP;
            } else if (ev.note == 80) {
                filter_env_amount = ev.vel;
            } else if (ev.note == 81) {
                sub_osc_level = ev.vel;
            } else if (ev.note == 82) {
                pulse_width = ev.vel;
            } else if (ev.note == 83) {
                noise_level = ev.vel;
            } else if (ev.note == 84) {
                drive_amount = ev.vel;
            } else if (ev.note == 85) {
                morph_enabled = ev.vel >= 64 ? 1 : 0;
            } else if (ev.note == 86) {
                morph_position = ev.vel;
            } else if (ev.note == 87) {
                stereo_spread = ev.vel;
            } else if (ev.note == 88) {
                key_track = ev.vel;
            } else if (ev.note == 89) {
                chorus_depth = ev.vel;
            } else if (ev.note == 90) {
                chorus_rate = ev.vel;
            }
        } else if (ev.type == 0x90 && ev.vel > 0) {
            voice_note_on(ev.note, ev.vel);
        } else if (ev.type == 0xE0) {
            pitch_bend = ((int16_t)ev.vel << 7 | ev.note) - 8192;
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
        int32_t mix_l = 0;
        int32_t mix_r = 0;

        // LFO: compute once per sample, shared across voices
        lfo_phase += lfo_rate;
        uint16_t lfo_idx = (lfo_phase >> 22) & 0x3FF;
        int32_t lfo_value = sine_table[lfo_idx];  // -32767..+32767

        for (int i = 0; i < NUM_VOICES; i++) {
            if (!voices[i].active) continue;

            // ADSR envelope state machine (amplitude)
            switch (voices[i].env_state) {
            case ENV_ATTACK:
                voices[i].envelope += env_attack_rate;
                if (voices[i].envelope >= ENV_MAX) {
                    voices[i].envelope = ENV_MAX;
                    voices[i].env_state = ENV_DECAY;
                }
                break;
            case ENV_DECAY:
                if (voices[i].envelope > env_sustain_level + env_decay_rate) {
                    voices[i].envelope -= env_decay_rate;
                } else {
                    voices[i].envelope = env_sustain_level;
                    if (env_sustain_level == 0) {
                        voices[i].active = 0;
                        voices[i].env_state = ENV_OFF;
                        continue;
                    }
                    voices[i].env_state = ENV_SUSTAIN;
                }
                break;
            case ENV_SUSTAIN:
                voices[i].envelope = env_sustain_level;
                break;
            case ENV_RELEASE:
                voices[i].envelope = (voices[i].envelope * env_release_mult) >> 8;
                if (voices[i].envelope < 4) {
                    voices[i].envelope = 0;
                    voices[i].active = 0;
                    voices[i].env_state = ENV_OFF;
                    continue;
                }
                break;
            default:
                voices[i].active = 0;
                continue;
            }

            // Filter envelope (same ADSR shape, independent state)
            switch (voices[i].filt_env_state) {
            case ENV_ATTACK:
                voices[i].filt_envelope += env_attack_rate;
                if (voices[i].filt_envelope >= ENV_MAX) {
                    voices[i].filt_envelope = ENV_MAX;
                    voices[i].filt_env_state = ENV_DECAY;
                }
                break;
            case ENV_DECAY:
                if (voices[i].filt_envelope > env_sustain_level + env_decay_rate) {
                    voices[i].filt_envelope -= env_decay_rate;
                } else {
                    voices[i].filt_envelope = env_sustain_level;
                    voices[i].filt_env_state = ENV_SUSTAIN;
                }
                break;
            case ENV_SUSTAIN:
                voices[i].filt_envelope = env_sustain_level;
                break;
            case ENV_RELEASE:
                voices[i].filt_envelope = (voices[i].filt_envelope * env_release_mult) >> 8;
                if (voices[i].filt_envelope < 4) voices[i].filt_envelope = 0;
                break;
            default:
                break;
            }

            // Portamento: glide phase_inc toward target
            if (voices[i].phase_inc != voices[i].target_phase_inc) {
                int32_t diff = (int32_t)(voices[i].target_phase_inc - voices[i].phase_inc);
                uint8_t shift = 1 + (porta_rate >> 4);  // 1-8, higher = slower
                voices[i].phase_inc += diff >> shift;
            }

            // DDS: full 32-bit phase accumulator + vibrato + detune + pitch bend
            uint32_t phase_inc = voices[i].phase_inc;

            // Pitch bend: ±2 semitones (shift phase_inc proportionally)
            if (pitch_bend != 0) {
                int32_t bend_factor = (int32_t)pitch_bend * (int32_t)(phase_inc >> 13);
                phase_inc = (uint32_t)((int32_t)phase_inc + (bend_factor >> 13));
            }

            if (lfo_depth > 0 && lfo_target == 0) {
                int32_t mod = (int32_t)(((int64_t)phase_inc * lfo_value * lfo_depth) >> 22);
                phase_inc = (uint32_t)((int32_t)phase_inc + mod);
            }
            if (detune_amount > 0) {
                int32_t spread = (int32_t)(phase_inc >> 10) * (int32_t)detune_amount * (i - 2);
                phase_inc = (uint32_t)((int32_t)phase_inc + (spread >> 7));
            }
            voices[i].phase += phase_inc;

            int32_t sample;
            if (morph_enabled) {
                // Morph mode: blend between adjacent waveforms
                // 0..42 = sine→tri, 43..84 = tri→saw, 85..127 = saw→square
                uint32_t p = voices[i].phase;
                int32_t s_sine, s_tri, s_saw, s_sqr;
                uint8_t mp = morph_position;

                if (mp < 43) {
                    // Blend sine → triangle
                    uint16_t idx = (p >> 22) & 0x3FF;
                    uint8_t frac = (p >> 14) & 0xFF;
                    int32_t a = sine_table[idx];
                    int32_t b = sine_table[(idx + 1) & 0x3FF];
                    s_sine = a + (((b - a) * frac) >> 8);

                    if (p < 0x80000000)
                        s_tri = (int32_t)(p >> 15) - 32768;
                    else
                        s_tri = 32767 - (int32_t)((p - 0x80000000) >> 15);

                    uint8_t blend = mp * 6;  // 0..252
                    sample = s_sine + (((s_tri - s_sine) * blend) >> 8);
                } else if (mp < 85) {
                    // Blend triangle → saw
                    if (p < 0x80000000)
                        s_tri = (int32_t)(p >> 15) - 32768;
                    else
                        s_tri = 32767 - (int32_t)((p - 0x80000000) >> 15);

                    s_saw = (int32_t)(p >> 16) - 32768;

                    uint8_t blend = (mp - 43) * 6;
                    sample = s_tri + (((s_saw - s_tri) * blend) >> 8);
                } else {
                    // Blend saw → square
                    s_saw = (int32_t)(p >> 16) - 32768;

                    uint32_t pw_threshold = ((uint32_t)pulse_width + 1) << 24;
                    s_sqr = (p > pw_threshold) ? -32767 : 32767;

                    uint8_t blend = (mp - 85) * 6;
                    sample = s_saw + (((s_sqr - s_saw) * blend) >> 8);
                }
            } else {
                // Discrete waveform mode
                switch (current_waveform) {
                case WF_SAW:
                    sample = (int32_t)(voices[i].phase >> 16) - 32768;
                    break;
                case WF_SQUARE: {
                    uint32_t pw_threshold = ((uint32_t)pulse_width + 1) << 24;
                    sample = (voices[i].phase > pw_threshold) ? -32767 : 32767;
                    break;
                }
                case WF_TRIANGLE:
                    if (voices[i].phase < 0x80000000)
                        sample = (int32_t)(voices[i].phase >> 15) - 32768;
                    else
                        sample = 32767 - (int32_t)((voices[i].phase - 0x80000000) >> 15);
                    break;
                default: {
                    uint16_t idx = (voices[i].phase >> 22) & 0x3FF;
                    uint8_t frac = (voices[i].phase >> 14) & 0xFF;
                    int32_t a = sine_table[idx];
                    int32_t b = sine_table[(idx + 1) & 0x3FF];
                    sample = a + (((b - a) * frac) >> 8);
                    break;
                }
                }
            }

            // Sub-oscillator: one octave below (half phase_inc = phase bit 31 for square sub)
            if (sub_osc_level > 0) {
                int32_t sub = (voices[i].phase & 0x80000000) ?
                    ((voices[i].phase << 1) >> 16) - 32768 :
                    32767 - ((voices[i].phase << 1) >> 16);
                sample += (sub * sub_osc_level) >> 7;
            }

            // Noise mix
            if (noise_level > 0) {
                noise_lfsr ^= noise_lfsr << 13;
                noise_lfsr ^= noise_lfsr >> 17;
                noise_lfsr ^= noise_lfsr << 5;
                int32_t noise_sample = (int32_t)(noise_lfsr >> 16) - 32768;
                sample += (noise_sample * noise_level) >> 8;
            }

            // Chamberlin SVF filter (bypass when fully open and no filter env)
            if (filter_cutoff < 32767 || filter_env_amount > 0) {
                uint16_t f = filter_cutoff;
                // Filter envelope modulation: add envelope to cutoff
                if (filter_env_amount > 0) {
                    int32_t env_mod = ((int32_t)voices[i].filt_envelope * filter_env_amount) >> 5;
                    int32_t f_mod = (int32_t)f + env_mod;
                    f = f_mod > 24000 ? 24000 : (uint16_t)f_mod;
                }
                // Key tracking: higher notes = brighter
                if (key_track > 0) {
                    int32_t kt_mod = ((int32_t)(voices[i].note) - 60) * key_track * 2;
                    int32_t f_kt = (int32_t)f + kt_mod;
                    if (f_kt < 200) f_kt = 200;
                    if (f_kt > 24000) f_kt = 24000;
                    f = (uint16_t)f_kt;
                }
                if (f > 24000) f = 24000;

                voices[i].filt_low += (int32_t)((int32_t)f * voices[i].filt_band) >> 15;
                int32_t high = sample - voices[i].filt_low - ((filter_reso * voices[i].filt_band) >> 8);
                voices[i].filt_band += (int32_t)((int32_t)f * high) >> 15;

                switch (filter_type) {
                case FILT_HP: sample = high; break;
                case FILT_BP: sample = voices[i].filt_band; break;
                default:      sample = voices[i].filt_low; break;
                }
            }

            // Scale by velocity and envelope
            sample = (sample * voices[i].velocity) >> 9;
            sample = (sample * voices[i].envelope) >> 12;

            // Tremolo (volume modulation)
            if (lfo_depth > 0 && lfo_target == 1) {
                int32_t trem = 32767 + ((lfo_value * lfo_depth) >> 8);
                sample = (sample * trem) >> 15;
            }

            // Stereo spread: pan voice across L/R
            if (stereo_spread > 0) {
                // pan: -128..+127 based on voice index (6 voices: -2.5..+2.5 → scaled)
                int32_t pan = ((int32_t)i * 2 - (NUM_VOICES - 1)) * stereo_spread;  // -635..+635
                int32_t gain_l = 256 - (pan >> 2);  // ~128..384
                int32_t gain_r = 256 + (pan >> 2);
                if (gain_l < 0) gain_l = 0;
                if (gain_r < 0) gain_r = 0;
                mix_l += (sample * gain_l) >> 8;
                mix_r += (sample * gain_r) >> 8;
            } else {
                mix_l += sample;
                mix_r += sample;
            }
        }

        // Overdrive: boost signal then soft-clip
        if (drive_amount > 0) {
            int32_t gain = 128 + (int32_t)drive_amount * 3;
            mix_l = (mix_l * gain) >> 7;
            mix_r = (mix_r * gain) >> 7;
            if (mix_l > 32767) mix_l = 32767;
            if (mix_l < -32767) mix_l = -32767;
            if (mix_r > 32767) mix_r = 32767;
            if (mix_r < -32767) mix_r = -32767;
            int32_t x, x3;
            x = mix_l; x3 = (x * ((x * x) >> 15)) >> 15; mix_l = x - (x3 / 3);
            x = mix_r; x3 = (x * ((x * x) >> 15)) >> 15; mix_r = x - (x3 / 3);
        }

        // Master volume (CC #7)
        mix_l = (mix_l * master_volume) >> 7;
        mix_r = (mix_r * master_volume) >> 7;

        // Chorus: modulated delay mixed into output
        if (chorus_depth > 0) {
            int16_t mono = (int16_t)(((mix_l + mix_r) >> 1));
            chorus_buf[chorus_write_pos] = mono;
            chorus_write_pos = (chorus_write_pos + 1) & (CHORUS_BUF_SIZE - 1);

            // Chorus LFO: modulate delay time
            chorus_lfo_phase += 22369 + (uint32_t)chorus_rate * 3350;  // ~0.25..5 Hz
            uint16_t c_idx = (chorus_lfo_phase >> 22) & 0x3FF;
            int32_t c_lfo = sine_table[c_idx];  // -32767..+32767

            // Delay range: 200..1500 samples (~4..31ms)
            int32_t delay_samples = 850 + ((c_lfo * 650) >> 15);
            uint16_t read_pos = (chorus_write_pos - delay_samples) & (CHORUS_BUF_SIZE - 1);
            int32_t wet = chorus_buf[read_pos];

            int32_t wet_amount = (int32_t)chorus_depth * 2;  // 0..254
            mix_l += (wet * wet_amount) >> 8;
            mix_r -= (wet * wet_amount) >> 9;  // inverted for stereo width
        }

        // Soft clipping
        mix_l = soft_clip(mix_l);
        mix_r = soft_clip(mix_r);

        // Final hard limit
        if (mix_l > 32767) mix_l = 32767;
        if (mix_l < -32767) mix_l = -32767;
        if (mix_r > 32767) mix_r = 32767;
        if (mix_r < -32767) mix_r = -32767;

        buf[s]     = (int16_t)mix_l;
        buf[s + 1] = (int16_t)mix_r;
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

static void all_notes_off(void) {
    for (int i = 0; i < NUM_VOICES; i++) {
        voices[i].active = 0;
        voices[i].envelope = 0;
        voices[i].env_state = ENV_OFF;
    }
    midi_ring_tail = midi_ring_head;
}

static volatile uint32_t midi_guard = 0;

void tud_umount_cb(void) { all_notes_off(); }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; all_notes_off(); }
void tud_mount_cb(void) { all_notes_off(); send_state_flag = 1; }

static void send_cc(uint8_t cc, uint8_t val) {
    uint8_t packet[4] = {0x0B, 0xB0, cc, val};
    tud_midi_packet_write(packet);
}

static void send_synth_state(void) {
    send_cc(7, master_volume);
    send_cc(73, 127 - (env_attack_rate - 2) * 5);
    send_cc(75, 127 - (env_decay_rate - 2) * 5);
    send_cc(70, env_sustain_level / 32);
    send_cc(72, (env_release_mult - 200) * 127 / 55);
    send_cc(74, filter_cutoff >= 32767 ? 127 : (filter_cutoff - 200) / 187);
    send_cc(71, (256 - filter_reso) / 2);
    send_cc(76, (lfo_rate - 44739) / 6700);
    send_cc(77, lfo_depth);
    send_cc(78, lfo_target ? 127 : 0);
    send_cc(94, detune_amount);
    send_cc(5, porta_rate);
    send_cc(79, filter_type == FILT_LP ? 0 : filter_type == FILT_HP ? 64 : 127);
    send_cc(80, filter_env_amount);
    send_cc(81, sub_osc_level);
    send_cc(82, pulse_width);
    send_cc(83, noise_level);
    send_cc(84, drive_amount);
    send_cc(85, morph_enabled ? 127 : 0);
    send_cc(86, morph_position);
    send_cc(87, stereo_spread);
    send_cc(88, key_track);
    send_cc(89, chorus_depth);
    send_cc(90, chorus_rate);
    uint8_t wf_cc;
    if (current_waveform == WF_SINE) wf_cc = 0;
    else if (current_waveform == WF_TRIANGLE) wf_cc = 42;
    else if (current_waveform == WF_SAW) wf_cc = 64;
    else wf_cc = 127;
    send_cc(1, wf_cc);
}

int main(void) {
    for (int i = 0; i < NUM_VOICES; i++) {
        voices[i].active = 0;
        voices[i].envelope = 0;
        voices[i].env_state = ENV_OFF;
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

    for (volatile int i = 0; i < 5000000; i++);
    GPIOA->BSRR = GPIO_BSRR_BS8;

    while (1) {
        tud_task();

        if (tud_midi_mounted()) {
            if (send_state_flag) {
                send_state_flag = 0;
                send_synth_state();
            }

            uint8_t packet[4];
            while (tud_midi_available()) {
                if (tud_midi_packet_read(packet)) {
                    uint8_t cin = packet[0] & 0x0F;
                    uint8_t msg = packet[1] & 0xF0;
                    uint8_t note = packet[2];
                    uint8_t vel = packet[3];

                    // Validate: CIN must match message type
                    if (cin == 0x09 && msg == 0x90 && note <= 127 && vel <= 127) {
                        if (vel > 0)
                            midi_ring_push(0x90, note, vel);
                        else
                            midi_ring_push(0x80, note, 0);
                    } else if (cin == 0x08 && msg == 0x80 && note <= 127) {
                        midi_ring_push(0x80, note, 0);
                    } else if (cin == 0x0B && msg == 0xB0 && note <= 127 && vel <= 127) {
                        midi_ring_push(0xB0, note, vel);
                    } else if (cin == 0x0E && msg == 0xE0) {
                        midi_ring_push(0xE0, note, vel);
                    } else if (cin == 0x0C && (packet[1] & 0xF0) == 0xC0) {
                        // Program Change
                        load_preset(packet[2] & 0x0F);
                    }
                    // All other packets (garbage, SysEx, etc.) silently dropped
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
