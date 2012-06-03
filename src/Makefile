EXECUTABLE=flight_controller.elf
BIN_IMAGE=flight_controller.bin
STM32LIB=STM32F4xx_StdPeriph_Driver

CC=arm-eabi-gcc
OBJCOPY=arm-eabi-objcopy

CFLAGS=-g -O2 -mlittle-endian -mthumb
CFLAGS+=-mcpu=cortex-m4
CFLAGS+=-ffreestanding -nostdlib

# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

# Includes
CFLAGS+=-I./

# stm32f4_discovery lib
CFLAGS+=-I$(STM32LIB)/inc
CFLAGS+=-I$(STM32LIB)/inc/device_support
CFLAGS+=-I$(STM32LIB)/inc/core_support

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c system_stm32f4xx.c startup_stm32f4xx.s stm32f4xx_it.c

	$(CC) $(CFLAGS) $^ -o $@  -L$(STM32LIB)/build -lSTM32F4xx_StdPeriph_Driver

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

.PHONY: all clean

