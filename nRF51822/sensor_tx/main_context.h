#pragma once

#include <stdint.h>

typedef struct main_context_s {
	int one_wire;
	uint8_t packet_counter;
  	uint8_t wake_up_counter;
} main_context_t;