TEMPLATEROOT = ..

BOARD ?= BOARD_CUSTOM

SIMPLE_BLINK ?= 0 

# compilation flags for gdb
CFLAGS  += -O0 -g
CFLAGS  += -DNRF51822
CFLAGS  += -D$(BOARD) -DSIMPLE_BLINK=$(SIMPLE_BLINK)
ASFLAGS += -g 

LDSCRIPT = $(TEMPLATEROOT)/blank_nrf51822_QFAA.ld

# object files
OBJS = system_nrf51.o main.o 
OBJS += $(NOSDK_PATH)/drivers_nrf/clock/nrf_drv_clock.o
OBJS += $(NOSDK_PATH)/drivers_nrf/rtc/nrf_drv_rtc.o
#OBJS += $(NOSDK_PATH)/libraries/timer/app_timer.o 
OBJS += $(NOSDK_PATH)/libraries/util/app_error.o $(NOSDK_PATH)/libraries/util/app_error_weak.o $(NOSDK_PATH)/libraries/util/app_util_platform.o
#OBJS += $(NOSDK_PATH)/libraries/button/app_button.o 
#OBJS += $(NOSDK_PATH)/drivers_nrf/gpiote/nrf_drv_gpiote.o
OBJS += $(NOSDK_PATH)/drivers_nrf/common/nrf_drv_common.o
#OBJS += $(NOSDK_PATH)/libraries/log/src/nrf_log_frontend.o $(NOSDK_PATH)/libraries/log/src/nrf_log_backend_serial.o
#OBJS += $(NOSDK_PATH)/drivers_nrf/uart/nrf_drv_uart.o 
OBJS += $(NOSDK_PATH)/libraries/bsp/bsp.o $(NOSDK_PATH)/boards/boards.o


# include paths
INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/rtc
INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/clock
INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/delay
INC_PATHS += -I$(NOSDK_PATH)/boards
INC_PATHS += -I$(NOSDK_PATH)/libraries/timer
INC_PATHS += -I$(NOSDK_PATH)/libraries/bsp
INC_PATHS += -I$(NOSDK_PATH)/libraries/button
INC_PATHS += -I$(NOSDK_PATH)/libraries/log
INC_PATHS += -I$(NOSDK_PATH)/libraries/log/src
INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/gpiote
INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/common
#INC_PATHS += -I$(NOSDK_PATH)/drivers_nrf/uart


# include common make file
include $(TEMPLATEROOT)/Makefile.nosdk
