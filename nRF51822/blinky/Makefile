TEMPLATEROOT = ..

BOARD ?= BOARD_CUSTOM

SIMPLE_BLINK ?= 0 

# compilation flags for gdb
CFLAGS  += -O0 -g
CFLAGS  += -DNRF51822
CFLAGS  += -D$(BOARD) -DSIMPLE_BLINK=$(SIMPLEBLINK)
ASFLAGS += -g 

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

# object files

OBJS =  system_nrf51.o blinky.o boards.o
#OBJS += nrf_delay.o

# include common make file
include $(TEMPLATEROOT)/Makefile.sdk123
