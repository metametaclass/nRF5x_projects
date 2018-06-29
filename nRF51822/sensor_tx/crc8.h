#pragma once

#include <stdint.h>
#include <stddef.h>

void update_crc8_1wire(uint8_t *crc, uint8_t data);

uint8_t calc_crc8_1wire(uint8_t *data, size_t len);
