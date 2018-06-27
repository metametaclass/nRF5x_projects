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
OBJS =  main.o payload.o ow.o leds.o rtc.o bit_swap.o radio_tx.o adc.o system_nrf51.o $(NOSDK_PATH)/drivers_nrf/common/nrf_drv_common.o $(NOSDK_PATH)/drivers_nrf/hal/nrf_adc.o

# use delay
INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/delay
INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/common

# include common make file
include $(TEMPLATEROOT)/Makefile.nosdk

