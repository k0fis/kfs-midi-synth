CC = arm-none-eabi-gcc
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -O2 -Wall
LDFLAGS = -T link.ld -nostartfiles -nostdlib

all: sine_test.bin

sine_test.elf: main.c link.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ main.c

sine_test.bin: sine_test.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: sine_test.bin
	st-flash write sine_test.bin 0x08000000

clean:
	rm -f sine_test.elf sine_test.bin

.PHONY: all flash clean
