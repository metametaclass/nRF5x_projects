#pragma once

#include <stdint.h>

typedef struct main_context_s {
	uint32_t wake_up_counter;
	int one_wire_error;
	uint8_t real_len;
	uint8_t packet_counter;  	
  	uint8_t onewire_rom[8];
} main_context_t;