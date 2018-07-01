#pragma once

#include "board_config.h"

#ifdef BOARD_CONFIG_ONE_WIRE

//onewire_start called before previous poll finished
#define ONE_WIRE_ERROR_NOT_IDLE 1

//unknown FSM state
#define ONE_WIRE_ERROR_INVALID_STATE 2

//no RESET answer on 1-wire bus
#define ONE_WIRE_ERROR_NO_DEVICES 3

//polling not finished yet
#define ONE_WIRE_ERROR_BUSY 4

//onewire_read_result called when polling not started
//#define ONE_WIRE_ERROR_NOT_STARTED 5

//invalid high-level FSM state
#define ONE_WIRE_ERROR_INVALID_DS18B20_STATE 6

//too large read result
#define ONE_WIRE_ERROR_TOO_LARGE_DATA 7

//invalid CRC in ds18b20 ROM or RAM 
#define ONE_WIRE_ERROR_INVALID_CRC 8

//waiting T conversion more than 
#define ONE_WIRE_ERROR_CONVERSION_TIMEOUT 9

//too fast compare event
#define ONE_WIRE_ERROR_TOO_FAST_TIMER 10

void onewire_init();

//int onewire_reset();

void onewire_start();

int onewire_read_result(uint8_t *result, size_t len, int *real_len);

#endif //BOARD_CONFIG_ONE_WIRE