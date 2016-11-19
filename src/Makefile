BINARY = usbmidi
#CSTD = -std=gnu99
SRCS = $(filter-out $(BINARY).c,$(wildcard *.c))
OBJS = $(subst .c,.o,$(SRCS))

OPENCM3_DIR = ../libopencm3
LDSCRIPT = $(OPENCM3_DIR)/lib/stm32/f1/stm32f103x8.ld
#LDSCRIPT = ./stm32f103x8_DFU.ld

include ../libopencm3.target.mk

