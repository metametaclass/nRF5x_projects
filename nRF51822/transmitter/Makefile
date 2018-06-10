TEMPLATEROOT = ..

BOARD ?= BOARD_CUSTOM

# compilation flags for gdb
CFLAGS  += -O0 -g
CFLAGS  += -DNRF51822
CFLAGS  += -D$(BOARD)

ASFLAGS += -g 

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld


# object files

OBJS =  main.o 
OBJS += system_nrf51.o
OBJS += app_timer.o app_error.o app_error_weak.o app_util_platform.o
OBJS += app_button.o nrf_drv_gpiote.o
OBJS += nrf_log_frontend.o nrf_log_backend_serial.o
OBJS += nrf_drv_uart.o nrf_drv_common.o
OBJS += radio_config.o
OBJS += bsp.o boards.o
# include common make file

include $(TEMPLATEROOT)/Makefile.common
