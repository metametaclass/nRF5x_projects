#pragma once

#include "nrf.h"

#include "board_config.h"

#ifdef BOARD_CONFIG_DEBUG_PIN
void debug_pin_init();

__STATIC_INLINE void debug_pin_set(){
	NRF_GPIO->OUTSET = (1 << BOARD_CONFIG_DEBUG_PIN);
}

__STATIC_INLINE void debug_pin_clear(){
	NRF_GPIO->OUTCLR = (1 << BOARD_CONFIG_DEBUG_PIN);
}

#else

#define debug_pin_init()

#define debug_pin_set()

#define debug_pin_clear()

#endif