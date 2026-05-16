# STM32 MIDI Synthesizer

6-voice polyphonic USB MIDI synthesizer on STM32F411 BlackPill with I2S DAC output.

## Hardware

```
STM32F411 BlackPill          GY-PCM5102 I2S DAC
┌──────────────────┐        ┌─────────────────┐
│  PB12 (I2S_WS)  ├────────┤ LRCK            │
│  PB13 (I2S_CK)  ├────────┤ BCK             │
│  PB15 (I2S_SD)  ├────────┤ DIN             │
│  PA8 (GPIO)     ├────────┤ XSMT            │
│  3V3            ├────────┤ VCC             │
│  GND            ├────────┤ GND             │
└──────────────────┘        │  SCK  → GND     │
                            │  FMT  → GND     │
                            │  DEMP → GND     │
                            │  FLT  → GND     │
                            └─────────────────┘
```

PCM5102 module jumpers: SCK→GND (internal PLL), FMT→GND (I2S standard), XSMT controlled by PA8 (HIGH=unmute).

## Structure

```
sine-test/     — basic I2S sine wave test (HW validation, no USB)
midi-synth/    — USB MIDI polyphonic synth (main project)
pio-synth/     — PlatformIO/Arduino variant (alternative build)
lib/tinyusb/   — TinyUSB submodule (USB MIDI stack)
```

## Building (midi-synth)

Requirements: `arm-none-eabi-gcc`, `st-flash` (stlink)

```sh
cd midi-synth
make          # compile
make flash    # flash via ST-Link
```

After flashing, the BlackPill shows up as "KFS-MIDI Synth" USB MIDI device.

## Testing

Open `midi-synth/keyboard.html` in Chrome — it connects to the synth via Web MIDI and provides an on-screen keyboard with full parameter control.

Keyboard mapping: A–L = white keys, W–P = black keys.

## Features

### Audio Engine
- 48000 Hz sample rate (exact, via PLLI2S)
- 6-voice polyphony with voice stealing
- DDS (Direct Digital Synthesis) with 32-bit phase accumulator
- 1024-point sine wavetable with linear interpolation
- DMA double-buffered stereo audio output
- Lock-free MIDI event ring buffer (USB task → DMA ISR)

### Oscillators
- 4 waveforms: Sine, Triangle, Saw, Square
- Waveform morphing (continuous blend between shapes)
- Variable pulse width (square wave)
- Sub-oscillator (one octave below)
- Noise generator (white noise mix)
- Detune / unison spread across voices

### Modulation
- ADSR amplitude envelope
- Filter envelope (modulates cutoff)
- LFO with vibrato (pitch) and tremolo (volume) modes
- Pitch bend (±2 semitones, standard MIDI)
- Portamento / glide

### Filter
- Chamberlin State Variable Filter (LP / HP / BP)
- Resonance control
- Key tracking (filter follows note pitch)

### Effects
- Stereo spread (pan voices across L/R)
- Chorus (modulated delay with stereo width)
- Overdrive (cubic soft-clip with variable gain)

### Presets
- 10 built-in JS presets (Init, Supersaw, Bass, Pluck, Pad, Organ, Retro, Perc, Dirty, Morph)
- 8 user-saveable Flash presets (survive power cycles and firmware updates)
- Program Change for preset recall, CC#0 for save

### MIDI Control

All parameters controllable via standard MIDI CC:

| CC | Parameter | Default |
|----|-----------|---------|
| 1 | Waveform (0=sine, 42=tri, 64=saw, 127=square) | 0 |
| 5 | Portamento rate | 0 |
| 7 | Master volume | 19 |
| 70 | Sustain level | 96 |
| 71 | Filter resonance | 0 |
| 72 | Release time | 40 |
| 73 | Attack time | 10 |
| 74 | Filter cutoff | 127 |
| 75 | Decay time | 40 |
| 76 | LFO rate | 50 |
| 77 | LFO depth | 0 |
| 78 | LFO target (0=vibrato, 127=tremolo) | 0 |
| 79 | Filter type (0=LP, 64=HP, 127=BP) | 0 |
| 80 | Filter envelope amount | 0 |
| 81 | Sub-oscillator level | 0 |
| 82 | Pulse width | 64 |
| 83 | Noise level | 0 |
| 84 | Overdrive | 0 |
| 85 | Morph enable (≥64 = on) | 0 |
| 86 | Morph position (shape blend) | 0 |
| 87 | Stereo spread | 64 |
| 88 | Key tracking | 0 |
| 89 | Chorus depth | 0 |
| 90 | Chorus rate | 40 |
| 0 | Save preset to slot (value 0-7) | — |

Bidirectional: synth sends current state to host on USB connect.

## Flash Presets

User presets are stored in Flash sector 7 (0x08060000), which is not overwritten during normal firmware flash. Use Program Change 0–7 to load, CC#0 with value 0–7 to save current state.

## Dependencies

After cloning, fetch TinyUSB's STM32F4 dependencies:

```sh
cd lib/tinyusb
python3 tools/get_deps.py stm32f4
```
