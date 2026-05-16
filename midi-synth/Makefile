CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

TOP = ../lib/tinyusb
BSP = $(TOP)/hw/bsp/stm32f4
BOARD = $(BSP)/boards/stm32f411blackpill
ST_CMSIS = $(TOP)/hw/mcu/st/cmsis_device_f4
ST_HAL = $(TOP)/hw/mcu/st/stm32f4xx_hal_driver
CMSIS = $(TOP)/lib/CMSIS_5/CMSIS/Core

CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
         -DSTM32F411xE -DHSE_VALUE=25000000 \
         -DCFG_TUSB_MCU=OPT_MCU_STM32F4 \
         -Os -Wall -ffunction-sections -fdata-sections

INCLUDES = -I. \
           -I$(TOP)/src \
           -I$(CMSIS)/Include \
           -I$(ST_CMSIS)/Include \
           -I$(ST_HAL)/Inc

LDFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
          -T$(BOARD)/STM32F411CEUx_FLASH.ld \
          -nostartfiles -Wl,--gc-sections -lnosys

# TinyUSB source files
TUSB_SRC = $(TOP)/src/tusb.c \
           $(TOP)/src/common/tusb_fifo.c \
           $(TOP)/src/device/usbd.c \
           $(TOP)/src/class/midi/midi_device.c \
           $(TOP)/src/portable/synopsys/dwc2/dcd_dwc2.c \
           $(TOP)/src/portable/synopsys/dwc2/dwc2_common.c

# App source
APP_SRC = main.c usb_descriptors.c

all: midi_synth.bin

midi_synth.elf: $(APP_SRC) $(TUSB_SRC)
	$(CC) $(CFLAGS) $(INCLUDES) $(LDFLAGS) -o $@ $^

midi_synth.bin: midi_synth.elf
	$(OBJCOPY) -O binary $< $@
	@echo "Size: $$(stat -f%z $@) bytes"

flash: midi_synth.bin
	st-flash --connect-under-reset write $< 0x08000000
	st-flash reset

clean:
	rm -f midi_synth.elf midi_synth.bin

.PHONY: all flash clean
