CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# UnityMbed passes PROJECT=<project name>; a plain `make` builds build/firmware.*
PROJECT ?= firmware
TARGET   = $(PROJECT)
OUTDIR   = build
CPU      = cortex-m3

#   src/startup.c         vector table + Reset_Handler
#   src/main.c            clocks, LED, runs the self-tests, parks safely
#   src/tim1.c            three-phase centre-aligned PWM with dead-time
#   src/foc.c             Clarke / Park / inverse Park / SVPWM in Q15
#   src/foc_sin_table.c   generated Q15 sine table
#   src/selftest.c        dead-time, ADC trigger timing, math round trip
SRCS := $(wildcard src/*.c)

CFLAGS  = -mcpu=$(CPU) -mthumb -Os -g -ffreestanding -Wall -Wextra \
          -Iinc -Icmsis -DSTM32F103xB $(CFLAGS_EXTRA)
LDFLAGS = -nostdlib -Wl,-T,linker.ld -Wl,--gc-sections -Wl,-Map=$(OUTDIR)/$(TARGET).map

all: $(OUTDIR)/$(TARGET).elf $(OUTDIR)/$(TARGET).bin $(OUTDIR)/$(TARGET).hex

$(OUTDIR)/$(TARGET).elf: $(SRCS) linker.ld
	@mkdir -p $(OUTDIR)
	$(CC) $(CFLAGS) $(SRCS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(OUTDIR)/$(TARGET).bin: $(OUTDIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(OUTDIR)/$(TARGET).hex: $(OUTDIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(OUTDIR)/$(TARGET).hex
	openocd -f openocd.cfg -c "program $< verify reset exit"

# Address of the self-test result struct, for reading it over SWD
addr: $(OUTDIR)/$(TARGET).elf
	@arm-none-eabi-nm $< | grep -i selftest

clean:
	rm -rf $(OUTDIR)

.PHONY: all clean flash addr
