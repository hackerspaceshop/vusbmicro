###############################################################################
# Makefile for the project CapsLocker
###############################################################################

## General Flags
PROJECT = CapsLocker
MCU = attiny45
TARGET = capslocker.elf
CC = avr-gcc


FUSE_L  = 0xE1
FUSE_H  = 0xDD
AVRDUDE = avrdude -c usbtiny -p $(MCU) # edit this line for your programmer




## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99             -DF_CPU=16500000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=capslocker.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"./usbdrv" -I"." 

## Objects that must be built in order to link
OBJECTS = main.o oddebug.o usbdrv.o usbdrvasm.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) capslocker.hex capslocker.eep capslocker.lss size

## Compile
usbdrvasm.o: ./usbdrv/usbdrvasm.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

main.o: ./main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

oddebug.o: ./usbdrv/oddebug.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usbdrv.o: ./usbdrv/usbdrv.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
#	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) capslocker.elf dep/* capslocker.hex capslocker.eep capslocker.lss capslocker.map

# rule for programming fuse bits:
fuse: 
	sudo $(AVRDUDE)  -U hfuse:w:$(FUSE_H):m -U lfuse:w:$(FUSE_L):m

flash: capslocker.hex
	sudo $(AVRDUDE) -U flash:w:capslocker.hex:i



## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

