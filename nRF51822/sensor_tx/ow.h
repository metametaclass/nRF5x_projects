#pragma once

#define ONE_WIRE_ERROR_NOT_IDLE 1
#define ONE_WIRE_ERROR_INVALID_STATE 2
#define ONE_WIRE_ERROR_NO_DEVICES 3
#define ONE_WIRE_ERROR_BUSY 4
#define ONE_WIRE_ERROR_NOT_STARTED 5
#define ONE_WIRE_ERROR_INVALID_DS18B20_STATE 6
#define ONE_WIRE_ERROR_TOO_LARGE_DATA 7
#define ONE_WIRE_CRC_ERROR 8

void onewire_init();

//int onewire_reset();

void onewire_start();

int onewire_read_result(uint8_t *result, size_t len, int *real_len);