###############################################################################
# Makefile for the project TPRM
###############################################################################

## General Flags
PROJECT = TPRM
MCU = atmega32
TARGET = TPRM.elf
CC = avr-gcc.exe

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                 -DF_CPU=8000000UL -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=TPRM.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Libraries
LIBS = -lc -lm 

## Objects that must be built in order to link
OBJECTS = TPRM.o AvrXFifo.o avrx_canceltimer.o avrx_canceltimermessage.o avrx_eeprom.o avrx_generatesinglestepinterrupt.o avrx_halt.o avrx_message.o avrx_priority.o avrx_recvmessage.o avrx_reschedule.o avrx_resetsemaphore.o avrx_semaphores.o avrx_singlestep.o avrx_starttimermessage.o avrx_suspend.o avrx_tasking.o avrx_terminate.o avrx_testsemaphore.o avrx_timequeue.o ks0108.o twimaster.o avrx_systemtimer0.o I2CInterface.o PressureSens.o RTC.o TempRHSens.o HardwareInfo.o DisplayIO.o DeviceIO.o eeprom_i2c.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) TPRM.hex TPRM.eep TPRM.lss size

## Compile
avrx_canceltimer.o: ../avrx/avrx_canceltimer.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_canceltimermessage.o: ../avrx/avrx_canceltimermessage.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_eeprom.o: ../avrx/avrx_eeprom.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_generatesinglestepinterrupt.o: ../avrx/avrx_generatesinglestepinterrupt.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_halt.o: ../avrx/avrx_halt.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_message.o: ../avrx/avrx_message.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_priority.o: ../avrx/avrx_priority.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_recvmessage.o: ../avrx/avrx_recvmessage.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_reschedule.o: ../avrx/avrx_reschedule.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_resetsemaphore.o: ../avrx/avrx_resetsemaphore.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_semaphores.o: ../avrx/avrx_semaphores.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_singlestep.o: ../avrx/avrx_singlestep.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_starttimermessage.o: ../avrx/avrx_starttimermessage.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_suspend.o: ../avrx/avrx_suspend.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_tasking.o: ../avrx/avrx_tasking.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_terminate.o: ../avrx/avrx_terminate.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_testsemaphore.o: ../avrx/avrx_testsemaphore.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

avrx_timequeue.o: ../avrx/avrx_timequeue.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

TPRM.o: ../TPRM.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

AvrXFifo.o: ../avrx/AvrXFifo.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ks0108.o: ../LCD/ks0108.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

twimaster.o: ../TWImaster/twimaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

avrx_systemtimer0.o: ../avrx_systemtimer0.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

I2CInterface.o: ../I2CInterface.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

PressureSens.o: ../PressureSens.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

RTC.o: ../RTC.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

TempRHSens.o: ../TempRHSens.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

HardwareInfo.o: ../HardwareInfo.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

DisplayIO.o: ../DisplayIO.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

DeviceIO.o: ../DeviceIO.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

eeprom_i2c.o: ../eeprom_i2c.c
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
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) TPRM.elf dep/* TPRM.hex TPRM.eep TPRM.lss TPRM.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

