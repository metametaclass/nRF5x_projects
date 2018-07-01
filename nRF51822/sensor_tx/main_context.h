#pragma once

#include <stdint.h>

#define ONEWIRE_RESULT_SIZE 32

typedef struct main_context_s {
	uint32_t wake_up_counter;
	int one_wire_error;
	uint8_t real_len;
	uint8_t packet_counter;  	
  	uint8_t onewire_rom[ONEWIRE_RESULT_SIZE];
} main_context_t;

