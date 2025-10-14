PRG            ?= main
SRCS          = $(shell find . -name '*.c' -o -name '*.S')
OBJS           = $(SRCS:.c=.o)
OBJS 				   := $(OBJS:.S=.o)
	
MCU_TARGET      ?= atmega32u4
PROGRAMMER      ?= avr109
PROGRAMMER_PORT ?= /dev/ttyACM0
OPTIMIZE       = -O2          # options are 1, 2, 3, s
CC             = avr-gcc
AS						 = avr-gcc
F_CPU          = 8000000UL

DEFS         = -DF_CPU=$(F_CPU)
CFLAGS       = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
LDFLAGS      = -Wl,-Map,$(PRG).map
ASFLAGS 		 = $(DEFS)

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean: 
	rm -rf *.o $(PRG).elf *.bin *.hex *.srec *.bak  
	rm -rf $(PRG)_eeprom.bin $(PRG)_eeprom.hex $(PRG)_eeprom.srec
	rm -rf *.lst *.map 

program: $(PRG).hex
	sudo avrdude -c $(PROGRAMMER) -p m32u4 -P $(PROGRAMMER_PORT) -e -U flash:w:$(PRG).hex -v

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@

