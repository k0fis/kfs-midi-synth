# STM32 MIDI Synthesizer

4-voice polyphonic USB MIDI synthesizer on STM32F411 BlackPill with I2S DAC output.

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

Open `midi-synth/keyboard.html` in Chrome — it connects to the synth via Web MIDI and provides an on-screen keyboard (computer keys A–L = white keys, W–P = black keys).

## Features

- 48000 Hz sample rate (exact, via PLLI2S)
- 4-voice polyphony with voice stealing
- 1024-point sine wavetable with linear interpolation
- DDS (Direct Digital Synthesis) with 32-bit phase accumulator
- Exponential release envelope
- DMA double-buffered audio output
- Lock-free MIDI event ring buffer (USB task → DMA ISR)

## Dependencies

After cloning, fetch TinyUSB's STM32F4 dependencies:

```sh
cd lib/tinyusb
python3 tools/get_deps.py stm32f4
```
