TEMPLATEROOT = ..

BOARD ?= BOARD_CUSTOM

# compilation flags for gdb
CFLAGS  += -O0 -g
CFLAGS  += -DNRF51822
CFLAGS  += -D$(BOARD)

ASFLAGS += -g 

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld


#use only necessary includes and source files

# object files

OBJS =  main.o system_nrf51.o

# include common make file
include $(TEMPLATEROOT)/Makefile.nosdk

