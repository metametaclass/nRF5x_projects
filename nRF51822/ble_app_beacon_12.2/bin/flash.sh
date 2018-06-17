#!/bin/sh
openocd -d2 -f ../../openocd.cfg -c 'init_reset halt; init; halt; nrf51 mass_erase; program ../../../nordic/nRF5_SDK_12.3.0/components/softdevice/s130/hex/s130_nrf51_2.0.1_softdevice.hex verify; program ble_app_beacon_12.2.hex verify; reset; exit'
